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

#ifndef COMPLETE_COVERAGE__ROUTE_MODE_HPP_
#define COMPLETE_COVERAGE__ROUTE_MODE_HPP_

#include <vector>
#include <string>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "complete_coverage/utils.hpp"
#include "complete_coverage/types.hpp"

namespace complete_coverage
{

/**
 * @class Route mode and state
 */
class RouteMode
{
  /**
   * @brief Constructor for swath mode
   * @param node A node to get the swath type from
   */
  template<typename NodeT>
  explicit RouteMode(const NodeT & node)
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "RouteType", rclcpp::ParameterValue("BOUSTROPHEDON"));
    std::string type_str = node->get_parameter("RouteType").as_string();
    toUpper(type_str);

    if (type_str == "BOUSTROPHEDON") {
      type_ = RouteType::BOUSTROPHEDON;
    } else if (type_str == "SNAKE") {
      type_ = RouteType::SNAKE;
    } else if (type_str == "SPIRAL") {
      type_ = RouteType::SPIRAL;
    } else if (type_str == "CUSTOM") {
      type_ = RouteType::CUSTOM;
    } else {
      type_ = RouteType::UNKNOWN;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, "default_spiral_n", rclcpp::ParameterValue(4));
    default_spiral_n_ = node->get_parameter("default_spiral_n").as_int();

    nav2_util::declare_parameter_if_not_declared(
      node, "default_custom_order", rclcpp::PARAMETER_INTEGER_ARRAY);
    try {
      default_custom_order_ = node->get_parameter("default_custom_order").as_int_array();
    } catch (...) {
      RCLCPP_WARN(
        node->get_logger(),
        "default_custom_order was not set! "
        "If using Custom Route mode, the custom order must be set per-request!");
    }
  }

  /**
   * @brief Sets the mode manually of the route for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & mode)
  {
    if (mode == "BOUSTROPHEDON") {
      type_ = RouteType::BOUSTROPHEDON;
    } else if (mode == "SNAKE") {
      type_ = RouteType::SNAKE;
    } else if (mode == "SPIRAL") {
      type_ = RouteType::SPIRAL;
    } else if (mode == "CUSTOM") {
      type_ = RouteType::CUSTOM;
    } else {
      type_ = RouteType::UNKNOWN;
    }
  }

  /**
   * @brief Converts the route mode into a string for publication
   * @return String of mode
   */
  std::string toString()
  {
    switch (type_) {
      case RouteType::BOUSTROPHEDON:
        return "Boustrophedon";
      case RouteType::SNAKE:
        return "Snake";
      case RouteType::SPIRAL:
        return "Spiral";
      case RouteType::CUSTOM:
        return "Custom";
      default:
        return "Unknown";
    }
  }

  RouteType type_;
  std::vector<size_t> default_custom_order_;
  size_t default_spiral_n_;
};

}  // namespace complete_coverage

#endif  // COMPLETE_COVERAGE__ROUTE_MODE_HPP_
