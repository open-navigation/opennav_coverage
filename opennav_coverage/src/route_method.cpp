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

#include <vector>

#include "opennav_coverage/route_method.hpp"

namespace opennav_coverage
{

// Above this many total swaths, TspRouteMethod falls back to per-cell TSP to avoid
// F2C's O(N^2) all-pairs path matrix that OOMs on large decompositions.
static constexpr size_t kMaxSwathsForGlobalRoute = 300;

F2CRoute SwathOrderMethod::plan(
  const F2CCells & cells,
  const F2CSwathsByCells & swaths_by_cells,
  const opennav_coverage_msgs::msg::RouteMode & settings)
{
  // The orderers assume a single cell; multi-cell (decomposed) input breaks their
  // ordering, so only TSP handles it. Reject rather than produce a bad route.
  if (cells.size() > 1) {
    throw CoverageException(
            "Non-TSP route modes are not supported with field decomposition; "
            "use route_mode TSP or disable decomposition.");
  }

  // The F2C orderers operate on a single flat swath list.
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
  const F2CCells & cells,
  const F2CSwathsByCells & swaths_by_cells,
  const opennav_coverage_msgs::msg::RouteMode & settings)
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

  // One global genRoute keeps inter-cell connections along the shared borders/
  // headland. Capped by swath count: F2C's all-pairs path matrix grows with the
  // square of swath count and can run out of memory on large decompositions.
  size_t total_swaths = 0;
  for (size_t i = 0; i < swaths_by_cells.size(); ++i) {
    total_swaths += swaths_by_cells.at(i).size();
  }
  if (total_swaths <= kMaxSwathsForGlobalRoute) {
    f2c::rp::RoutePlannerBase rp;
    return rp.genRoute(
      cells, swaths_by_cells,
      /*show_log=*/false, d_tol, redirect_swaths, time_limit, search_for_optimum);
  }

  // Fallback for large decompositions: solve each cell alone and stitch the
  // routes together with a straight-line bridge between cells.
  F2CRoute merged;
  for (size_t i = 0; i < cells.size(); ++i) {
    if (i >= swaths_by_cells.size() || swaths_by_cells.at(i).size() == 0) {
      continue;
    }
    F2CCells cell(cells.getGeometry(i));
    F2CSwathsByCells cell_swaths;
    cell_swaths.emplace_back(swaths_by_cells.at(i));

    f2c::rp::RoutePlannerBase rp;
    F2CRoute cell_route = rp.genRoute(
      cell, cell_swaths,
      /*show_log=*/ false,
      d_tol,
      redirect_swaths,
      time_limit,
      search_for_optimum);
    if (cell_route.isEmpty()) {
      continue;
    }

    if (!merged.isEmpty()) {
      merged.addConnection(
        std::vector<F2CPoint>{merged.endPoint(), cell_route.startPoint()});
    }
    const auto & vec_swaths = cell_route.getVectorSwaths();
    const auto & connections = cell_route.getConnections();
    for (size_t k = 0; k < vec_swaths.size(); ++k) {
      merged.addConnectedSwaths(
        k < connections.size() ? connections[k] : F2CMultiPoint(), vec_swaths[k]);
    }
  }
  return merged;
}

}  // namespace opennav_coverage
