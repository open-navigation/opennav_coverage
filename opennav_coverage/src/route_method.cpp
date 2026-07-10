// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <limits>
#include <vector>

#include "opennav_coverage/route_method.hpp"

namespace opennav_coverage
{

namespace
{

F2CMultiPoint reversedConnection(const F2CMultiPoint & mp)
{
  std::vector<F2CPoint> pts;
  pts.reserve(mp.size());
  for (int i = static_cast<int>(mp.size()) - 1; i >= 0; --i) {
    pts.emplace_back(mp[i]);
  }
  return F2CMultiPoint(pts);
}

// Reverses group order, each swath, and each connection so a cell can be entered from either end.
F2CRoute reversedRoute(const F2CRoute & route)
{
  const auto & groups = route.getVectorSwaths();
  const auto & conns = route.getConnections();
  F2CRoute out;
  for (int k = static_cast<int>(groups.size()) - 1; k >= 0; --k) {
    F2CSwaths swaths = groups[k];
    swaths.reverse();
    for (size_t s = 0; s < swaths.size(); ++s) {
      swaths.at(s).reverse();
    }
    const size_t conn_after = static_cast<size_t>(k) + 1;
    out.addConnectedSwaths(
      conn_after < conns.size() ? reversedConnection(conns[conn_after]) : F2CMultiPoint(),
      swaths);
  }
  return out;
}

}  // namespace

F2CRoute SwathOrderMethod::plan(
  const F2CCells & travel_cells,
  const F2CCells & swath_cells,
  const F2CSwathsByCells & swaths_by_cells,
  const opennav_coverage_msgs::msg::RouteMode & settings,
  const std::optional<F2CPoint> & start_end)
{
  (void)travel_cells;
  (void)start_end;  // TSP-only concept; the generator already warns the user

  // These orderers assume a single cell; multi-cell input breaks their ordering.
  if (swath_cells.size() > 1) {
    throw CoverageException(
            "Non-TSP route modes are not supported with field decomposition; "
            "use route_mode TSP or disable decomposition.");
  }

  if (type_ == RouteType::SPIRAL) {
    dynamic_cast<f2c::rp::SpiralOrder *>(orderer_.get())->setSpiralSize(settings.spiral_n);
  } else if (type_ == RouteType::CUSTOM) {
    std::vector<size_t> custom_order(settings.custom_order.begin(), settings.custom_order.end());
    dynamic_cast<f2c::rp::CustomOrder *>(orderer_.get())->setCustomOrder(custom_order);
  }

  F2CSwaths ordered = orderer_->genSortedSwaths(swaths_by_cells.flatten());

  // Wrap ordered swaths in a single-group route so every mode returns F2CRoute.
  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), ordered);
  return route;
}

F2CRoute TspRouteMethod::plan(
  const F2CCells & travel_cells,
  const F2CCells & swath_cells,
  const F2CSwathsByCells & swaths_by_cells,
  const opennav_coverage_msgs::msg::RouteMode & settings,
  const std::optional<F2CPoint> & start_end)
{
  const bool redirect_swaths = settings.tsp_redirect_swaths;
  const long int time_limit = settings.tsp_time_limit;  // NOLINT
  const bool search_for_optimum = settings.tsp_search_for_optimum;
  const double d_tol = settings.tsp_d_tol;

  RCLCPP_DEBUG(
    logger_,
    "Generating TSP route: redirect=%s time_limit=%ld optimum=%s d_tol=%f",
    redirect_swaths ? "true" : "false", time_limit,
    search_for_optimum ? "true" : "false", d_tol);

  // Single cell: one genRoute call, which honors the start/end point exactly.
  if (swath_cells.size() <= 1) {
    f2c::rp::RoutePlannerBase rp;
    if (start_end) {
      rp.setStartAndEndPoint(*start_end);
    }
    return rp.genRoute(
      swath_cells, swaths_by_cells, false, d_tol, redirect_swaths, time_limit,
      search_for_optimum);
  }

  // Multi-cell: solve each cell alone to avoid interleaving and the all-pairs path matrix.
  std::vector<F2CRoute> cell_routes(swath_cells.size());
  for (size_t i = 0; i < swath_cells.size() && i < swaths_by_cells.size(); ++i) {
    if (swaths_by_cells.at(i).size() == 0) {
      continue;
    }
    F2CCells cell(swath_cells.getGeometry(i));
    F2CSwathsByCells cell_swaths;
    cell_swaths.emplace_back(swaths_by_cells.at(i));
    f2c::rp::RoutePlannerBase rp;
    cell_routes[i] = rp.genRoute(
      cell, cell_swaths, false, d_tol, redirect_swaths, time_limit, search_for_optimum);
  }

  std::vector<size_t> remaining;
  for (size_t i = 0; i < cell_routes.size(); ++i) {
    if (!cell_routes[i].isEmpty()) {
      remaining.push_back(i);
    }
  }
  if (remaining.empty()) {
    return F2CRoute();
  }

  // Visit cells in nearest-neighbor order, entering each from whichever end of
  // its route is closer (reversing the cell route when needed), so the bridges
  // between cells stay short instead of jumping across the field.
  const auto pickNearest =
    [&cell_routes, &remaining](const F2CPoint & from, bool & reversed) {
      size_t best_idx = remaining[0];
      double best_dist = std::numeric_limits<double>::max();
      for (const size_t idx : remaining) {
        const double d_fwd = from.distance(cell_routes[idx].startPoint());
        const double d_rev = from.distance(cell_routes[idx].endPoint());
        if (d_fwd < best_dist) {
          best_dist = d_fwd;
          best_idx = idx;
          reversed = false;
        }
        if (d_rev < best_dist) {
          best_dist = d_rev;
          best_idx = idx;
          reversed = true;
        }
      }
      return best_idx;
    };

  size_t current = remaining[0];
  bool current_reversed = false;
  if (start_end) {
    RCLCPP_WARN(
      logger_,
      "Multi-cell route: start_pose picks the nearest cell; the route starts at "
      "that cell's own start, not at the exact point.");
    current = pickNearest(*start_end, current_reversed);
  }

  // Bridges follow the travel-cell pair's border graph, not a line that could cut a void.
  const auto buildBridge =
    [&travel_cells, d_tol](
    const F2CSwath & from_swath, const F2CSwath & to_swath,
    const F2CPoint & from, const F2CPoint & to) {
      F2CSwaths end_swaths;
      end_swaths.emplace_back(from_swath);
      end_swaths.emplace_back(to_swath);
      F2CSwathsByCells bridge_swaths;
      bridge_swaths.emplace_back(end_swaths);

      const auto viaGraph =
        [&bridge_swaths, d_tol, &from, &to](const F2CCells & graph_cells) {
          std::vector<F2CPoint> pts;
          try {
            f2c::rp::RoutePlannerBase rp;
            F2CGraph2D graph = rp.createShortestGraph(graph_cells, bridge_swaths, d_tol);
            pts = graph.shortestPath(from, to);
          } catch (const std::exception &) {
            pts.clear();
          }
          return pts;
        };

      F2CCells pair_cells;
      const Field cell_a = travel_cells.getCellWherePoint(from);
      if (cell_a.size() > 0) {
        pair_cells.addGeometry(cell_a);
      }
      const Field cell_b = travel_cells.getCellWherePoint(to);
      if (cell_b.size() > 0) {
        pair_cells.addGeometry(cell_b);
      }

      std::vector<F2CPoint> bridge;
      if (pair_cells.size() > 0) {
        bridge = viaGraph(pair_cells);
      }
      if (bridge.size() < 2) {
        bridge = viaGraph(travel_cells);
      }
      if (bridge.size() < 2) {
        bridge = {from, to};
      }
      return bridge;
    };

  F2CRoute merged;
  std::optional<F2CSwath> last_swath;
  while (true) {
    remaining.erase(std::find(remaining.begin(), remaining.end(), current));
    const F2CRoute cell_route =
      current_reversed ? reversedRoute(cell_routes[current]) : cell_routes[current];
    if (!merged.isEmpty() && last_swath) {
      merged.addConnection(
        buildBridge(
          *last_swath, cell_route.getVectorSwaths().front().at(0),
          merged.endPoint(), cell_route.startPoint()));
    }
    const auto & vec_swaths = cell_route.getVectorSwaths();
    const auto & connections = cell_route.getConnections();
    for (size_t k = 0; k < vec_swaths.size(); ++k) {
      merged.addConnectedSwaths(
        k < connections.size() ? connections[k] : F2CMultiPoint(), vec_swaths[k]);
    }
    last_swath = cell_route.getVectorSwaths().back().back();

    if (remaining.empty()) {
      break;
    }
    current = pickNearest(merged.endPoint(), current_reversed);
  }
  return merged;
}

}  // namespace opennav_coverage
