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

#ifndef NAV2_COVERAGE__PATH_MODE_HPP_
#define NAV2_COVERAGE__PATH_MODE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_coverage/utils.hpp"
#include "nav2_coverage/types.hpp"
#include "nav2_coverage/robot.hpp"

namespace nav2_coverage
{

/**
 * @class Path mode and state
 */
class PathMode
{
public:
  /**
   * @brief Constructor for swath mode
   * @param node A node to get the swath type from
   */
  template<typename NodeT>
  explicit PathMode(const NodeT & node, RobotMode * robot)
  {
    logger_ = node->get_logger();
    robot_ = robot;

    nav2_util::declare_parameter_if_not_declared(
      node, "default_path_type", rclcpp::ParameterValue("DUBIN"));
    std::string type_str = node->get_parameter("default_path_type").as_string();
    default_type_ = toType(type_str);

    nav2_util::declare_parameter_if_not_declared(
      node, "default_path_continuity_type", rclcpp::ParameterValue("CONTINUOUS"));
    std::string type_cont_str = node->get_parameter("default_path_continuity_type").as_string();
    default_continuity_type_ = toContinuityType(type_cont_str);

    // Path Generator requires no changes at runtime
    generator_ = std::make_unique<f2c::pp::PathPlanning>();
    default_curve_ = createCurve(default_type_, default_continuity_type_);
  }

  /**
   * @brief Main method to generate path
   * @param Swaths swaths to generate path from
   * @param request Action request information
   * @return Path complete path
   */
  Path generatePath(const Swaths & swaths /*, (void) request*/);

  /**
   * @brief Sets the mode manually of the paths for dynamic parameters
   * @param mode String for mode to use
   */
  void setPathMode(const std::string & new_mode);

  /**
   * @brief Sets the mode manually of the path continuity for dynamic parameters
   * @param mode String for mode to use
   */
  void setPathContinuityMode(const std::string & new_mode);

protected:
  /**
   * @brief Creates generator pointer of a requested type
   * @param type curve generator type to create
   * @return Generator to use
   */
  TurningBasePtr createCurve(const PathType & type, const PathContinuityType & c_type);

  /**
   * @brief Converts the path mode into a string for publication
   * @return String of mode
   */
  std::string toString(const PathType & type, const PathContinuityType & c_type);

  /**
   * @brief Converts the Path string into a mode for handling
   * @param String of mode
   * @return Type of mode
   */
  PathType toType(std::string & str);

  /**
   * @brief Converts the Path string into a mode for handling
   * @param String of mode
   * @return Type of mode
   */
  PathContinuityType toContinuityType(std::string & str);

  PathType default_type_;
  PathContinuityType default_continuity_type_;
  TurningBasePtr default_curve_;
  std::unique_ptr<f2c::pp::PathPlanning> generator_;
  RobotMode * robot_;
  rclcpp::Logger logger_{rclcpp::get_logger("SwathGenerator")};
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__PATH_MODE_HPP_
