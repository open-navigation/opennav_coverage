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

#ifndef OPENNAV_COVERAGE__ROUTE_METHOD_HPP_
#define OPENNAV_COVERAGE__ROUTE_METHOD_HPP_

#include <memory>
#include <utility>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage_msgs/msg/route_mode.hpp"
#include "opennav_coverage/types.hpp"

namespace opennav_coverage
{

/**
 * @class RouteMethod
 * @brief Unifies F2C's two unrelated route planners (SingleCellSwathsOrderBase
 *        and RoutePlannerBase) behind one polymorphic call returning F2CRoute,
 *        so callers never branch on the route mode.
 */
class RouteMethod
{
public:
  virtual ~RouteMethod() = default;

  /**
   * @brief Plan an ordered route over the swaths.
   * @param cells Travel cells whose borders the route connections may follow
   * @param swaths_by_cells Per-cell swaths to be covered
   * @param settings Fully-resolved RouteMode (server has already applied defaults)
   * @return Ordered route: swath groups plus any headland connections
   */
  virtual F2CRoute plan(
    const F2CCells & cells,
    const F2CSwathsByCells & swaths_by_cells,
    const opennav_coverage_msgs::msg::RouteMode & settings) = 0;
};

/**
 * @class SwathOrderMethod
 * @brief Adapts the F2C swath-ordering modes (BOUSTROPHEDON, SNAKE, SPIRAL,
 *        CUSTOM). Flattens the per-cell swaths, orders them with the wrapped
 *        `SingleCellSwathsOrderBase`, and wraps the ordered swaths into a
 *        single-group `F2CRoute` (no connections) so the output type matches TSP.
 */
class SwathOrderMethod : public RouteMethod
{
public:
  SwathOrderMethod(
    RouteType type, std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> orderer)
  : type_(type), orderer_(std::move(orderer)) {}

  F2CRoute plan(
    const F2CCells & cells,
    const F2CSwathsByCells & swaths_by_cells,
    const opennav_coverage_msgs::msg::RouteMode & settings) override;

private:
  RouteType type_;
  std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> orderer_;
};

/**
 * @class TspRouteMethod
 * @brief Adapts F2C's `RoutePlannerBase` (OR-Tools TSP). Solves each cell
 *        separately and stitches the per-cell routes in sweep order, avoiding the
 *        all-pairs path matrix that exhausts memory on decomposed multi-cell input.
 */
class TspRouteMethod : public RouteMethod
{
public:
  explicit TspRouteMethod(const rclcpp::Logger & logger)
  : logger_(logger) {}

  F2CRoute plan(
    const F2CCells & cells,
    const F2CSwathsByCells & swaths_by_cells,
    const opennav_coverage_msgs::msg::RouteMode & settings) override;

private:
  rclcpp::Logger logger_;
};

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__ROUTE_METHOD_HPP_
