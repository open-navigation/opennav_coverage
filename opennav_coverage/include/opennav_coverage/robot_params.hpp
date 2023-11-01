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

#ifndef OPENNAV_COVERAGE__ROBOT_PARAMS_HPP_
#define OPENNAV_COVERAGE__ROBOT_PARAMS_HPP_

#include <vector>
#include <string>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_coverage/utils.hpp"
#include "opennav_coverage/types.hpp"

namespace opennav_coverage
{

/**
 * @class Robot's mode and state
 */
class RobotParams
{
public:
  /**
   * @brief Constructor for robot mode
   * @param node A node to get the robot parameters from
   */
  template<typename NodeT>
  explicit RobotParams(const NodeT & node)
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "robot_width", rclcpp::ParameterValue(2.1));
    robot_.robot_width = node->get_parameter("robot_width").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, "operation_width", rclcpp::ParameterValue(2.5));
    robot_.op_width = node->get_parameter("operation_width").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, "min_turning_radius", rclcpp::ParameterValue(0.4));
    robot_.setMinRadius(node->get_parameter("min_turning_radius").as_double());

    nav2_util::declare_parameter_if_not_declared(
      node, "linear_curv_change", rclcpp::ParameterValue(2.0));
    robot_.linear_curv_change = node->get_parameter("linear_curv_change").as_double();
  }

  /**
   * @brief Get the robot's width
   * @return robot width in m
   */
  double getWidth()
  {
    return robot_.robot_width;
  }

  /**
   * @brief Get the robot's operational width
   * @return robot's operational width in m
   */
  double getOperationWidth()
  {
    return robot_.op_width;
  }

  /**
   * @brief Get the robot's F2C model
   * @return Robot F2C typed model
   */
  Robot & getRobot()
  {
    return robot_;
  }

  Robot robot_;
};

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__ROBOT_PARAMS_HPP_
