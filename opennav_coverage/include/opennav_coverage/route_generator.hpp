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

#ifndef OPENNAV_COVERAGE__ROUTE_GENERATOR_HPP_
#define OPENNAV_COVERAGE__ROUTE_GENERATOR_HPP_

#include <vector>
#include <string>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_coverage_msgs/msg/route_mode.hpp"
#include "opennav_coverage/utils.hpp"
#include "opennav_coverage/types.hpp"

namespace opennav_coverage
{

/**
 * @class Route mode and state
 */
class RouteGenerator
{
public:
  /**
   * @brief Constructor for swath mode
   * @param node A node to get the swath type from
   */
  template<typename NodeT>
  explicit RouteGenerator(const NodeT & node)
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "default_route_type", rclcpp::ParameterValue("BOUSTROPHEDON"));
    std::string type_str = node->get_parameter("default_route_type").as_string();
    default_type_ = toType(type_str);
    default_generator_ = createGenerator(default_type_);

    nav2_util::declare_parameter_if_not_declared(
      node, "default_spiral_n", rclcpp::ParameterValue(4));
    default_spiral_n_ = node->get_parameter("default_spiral_n").as_int();

    nav2_util::declare_parameter_if_not_declared(
      node, "default_custom_order", rclcpp::PARAMETER_INTEGER_ARRAY);
    try {
      // Get the custom order and cast to size_t
      auto order = node->get_parameter("default_custom_order").as_integer_array();
      default_custom_order_ = std::vector<size_t>(order.begin(), order.end());
    } catch (...) {
      RCLCPP_WARN(
        node->get_logger(),
        "default_custom_order was not set! "
        "If using Custom Route mode, the custom order must be set per-request!");
    }
  }

  /**
   * @brief Main method to generate route
   * @param Swaths swaths to generate route from
   * @param request Action request information
   * @return Swaths ordered swaths
   */
  Swaths generateRoute(
    const Swaths & swaths, const opennav_coverage_msgs::msg::RouteMode & settings);

  /**
   * @brief Sets the mode manually of the Route for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & new_mode);

  /**
   * @brief Sets the spiral N manually for dynamic parameters
   * @param mode String for mode to use
   */
  void setSpiralN(const int & n) {default_spiral_n_ = n;}

  /**
   * @brief Sets the custom order manually for dynamic parameters
   * @param mode String for mode to use
   */
  void setCustomOrder(const std::vector<long int> & order)  // NOLINT
  {
    default_custom_order_ = std::vector<size_t>(order.begin(), order.end());
  }

protected:
  /**
   * @brief Creates generator pointer of a requested type
   * @param type Route generator type to create
   * @return Generator to use
   */
  RouteGeneratorPtr createGenerator(const RouteType & type);

  /**
   * @brief Converts the route mode into a string for publication
   * @param Type of mode
   * @return String of mode
   */
  std::string toString(const RouteType & type);

  /**
   * @brief Converts the route string into a mode for handling
   * @param String of mode
   * @return Type of mode
   */
  RouteType toType(const std::string & str);

  RouteType default_type_;
  std::vector<size_t> default_custom_order_;
  size_t default_spiral_n_;
  RouteGeneratorPtr default_generator_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("RouteGenerator")};
};

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__ROUTE_GENERATOR_HPP_
