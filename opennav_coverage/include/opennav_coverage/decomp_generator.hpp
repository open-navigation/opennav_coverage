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

#ifndef OPENNAV_COVERAGE__DECOMP_GENERATOR_HPP_
#define OPENNAV_COVERAGE__DECOMP_GENERATOR_HPP_

#include <string>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "opennav_coverage_msgs/msg/decomp_mode.hpp"
#include "opennav_coverage/utils.hpp"
#include "opennav_coverage/types.hpp"

namespace opennav_coverage
{

/**
 * @class Decomposition mode and state
 */
class DecompGenerator
{
public:
  /**
   * @brief Constructor for Decomposition mode
   * @param node A node to get the Decomposition type from
   */
  template<typename NodeT>
  explicit DecompGenerator(const NodeT & node)
  {
    logger_ = node->get_logger();

    nav2::declare_parameter_if_not_declared(
      node, "default_decomp_type", rclcpp::ParameterValue("NONE"));
    std::string type_str = node->get_parameter("default_decomp_type").as_string();
    default_type_ = toType(type_str);

    nav2::declare_parameter_if_not_declared(
      node, "default_decomp_split_angle", rclcpp::ParameterValue(0.0));
    default_split_angle_ = node->get_parameter("default_decomp_split_angle").as_double();
  }

  /**
   * @brief Main method to decompose a field into simpler sub-cells
   * @param cells Cells to decompose
   * @param settings Action request information
   * @return Decomposed cells (may be a single cell if NONE)
   */
  F2CCells decompose(
    const F2CCells & cells,
    const opennav_coverage_msgs::msg::DecompMode & settings);

  /**
   * @brief Sets the mode manually of the Decomposition for dynamic parameters
   * @param new_mode String for mode to use
   */
  void setMode(const std::string & new_mode);

  /**
   * @brief Sets the split angle manually for dynamic parameters
   * @param angle Split angle in radians
   */
  void setSplitAngle(double angle) {default_split_angle_ = angle;}

protected:
  /**
   * @brief Converts the Decomposition mode into a string for publication
   * @param type Type of mode
   * @return String of mode
   */
  std::string toString(const DecompType & type);

  /**
   * @brief Converts the Decomposition string into a mode for handling
   * @param str String of mode
   * @return Type of mode
   */
  DecompType toType(const std::string & str);

  DecompType default_type_;
  double default_split_angle_;
  rclcpp::Logger logger_{rclcpp::get_logger("DecompGenerator")};
};

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__DECOMP_GENERATOR_HPP_
