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
#include <string>

#include "opennav_coverage/route_generator.hpp"
#include "opennav_coverage/route_method.hpp"

namespace opennav_coverage
{

F2CRoute RouteGenerator::generateRoute(
  const F2CCells & cells, const F2CSwathsByCells & swaths_by_cells,
  const opennav_coverage_msgs::msg::RouteMode & settings)
{
  RouteType action_type = toType(settings.mode);

  // Resolve the method and fill in defaults for any knobs the request left unset.
  RouteGeneratorPtr method;
  opennav_coverage_msgs::msg::RouteMode eff = settings;
  if (action_type == RouteType::UNKNOWN) {
    method = default_generator_;
    eff.spiral_n = default_spiral_n_;
    eff.custom_order.assign(default_custom_order_.begin(), default_custom_order_.end());
    eff.tsp_redirect_swaths = default_tsp_redirect_swaths_;
    eff.tsp_time_limit = default_tsp_time_limit_;
    eff.tsp_search_for_optimum = default_tsp_search_for_optimum_;
    eff.tsp_d_tol = default_tsp_d_tol_;
  } else {
    method = createGenerator(action_type);
  }

  if (!method) {
    throw CoverageException(
            "No valid route mode set! Options: BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM, TSP.");
  }

  RCLCPP_DEBUG(logger_, "Generating route: %s", toString(resolveType(settings)).c_str());
  return method->plan(cells, swaths_by_cells, eff);
}

void RouteGenerator::setMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
  default_generator_ = createGenerator(default_type_);
}

RouteGeneratorPtr RouteGenerator::createGenerator(const RouteType & type)
{
  switch (type) {
    case RouteType::BOUSTROPHEDON:
      return std::make_shared<SwathOrderMethod>(
        type, std::make_shared<f2c::rp::BoustrophedonOrder>());
    case RouteType::SNAKE:
      return std::make_shared<SwathOrderMethod>(
        type, std::make_shared<f2c::rp::SnakeOrder>());
    case RouteType::SPIRAL:
      return std::make_shared<SwathOrderMethod>(
        type, std::make_shared<f2c::rp::SpiralOrder>());
    case RouteType::CUSTOM:
      return std::make_shared<SwathOrderMethod>(
        type, std::make_shared<f2c::rp::CustomOrder>());
    case RouteType::TSP:
      return std::make_shared<TspRouteMethod>(logger_);
    default:
      RCLCPP_WARN(logger_, "Unknown route type set!");
      return RouteGeneratorPtr{nullptr};
  }
}

std::string RouteGenerator::toString(const RouteType & type)
{
  switch (type) {
    case RouteType::BOUSTROPHEDON:
      return "Boustrophedon";
    case RouteType::SNAKE:
      return "Snake";
    case RouteType::SPIRAL:
      return "Spiral";
    case RouteType::CUSTOM:
      return "Custom";
    case RouteType::TSP:
      return "TSP";
    default:
      return "Unknown";
  }
}

RouteType RouteGenerator::toType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "BOUSTROPHEDON") {
    return RouteType::BOUSTROPHEDON;
  } else if (mode_str == "SNAKE") {
    return RouteType::SNAKE;
  } else if (mode_str == "SPIRAL") {
    return RouteType::SPIRAL;
  } else if (mode_str == "CUSTOM") {
    return RouteType::CUSTOM;
  } else if (mode_str == "TSP") {
    return RouteType::TSP;
  } else {
    return RouteType::UNKNOWN;
  }
}

}  // namespace opennav_coverage
