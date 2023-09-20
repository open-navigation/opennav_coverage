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

#ifndef NAV2_COVERAGE__SWATH_MODE_HPP_
#define NAV2_COVERAGE__SWATH_MODE_HPP_

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
 * @class Swath mode and state
 */
class SwathMode
{
public:
  /**
   * @brief Constructor for swath mode
   * @param node A node to get the swath type from
   */
  template<typename NodeT>
  explicit SwathMode(const NodeT & node, RobotMode * robot)
  {
    logger_ = node->get_logger();
    robot_ = robot;

    nav2_util::declare_parameter_if_not_declared(
      node, "default_swath_type", rclcpp::ParameterValue("LENGTH"));
    std::string type_str = node->get_parameter("default_swath_type").as_string();
    default_type_ = toType(type_str);

    nav2_util::declare_parameter_if_not_declared(
      node, "default_swath_angle_type", rclcpp::ParameterValue("BRUTE_FORCE"));
    std::string angle_str = node->get_parameter("default_swath_angle_type").as_string();
    default_angle_type_ = toAngleType(type_str);

    nav2_util::declare_parameter_if_not_declared(
      node, "default_swath_angle", rclcpp::ParameterValue(0.0));
    default_swath_angle_ = node->get_parameter("default_swath_angle").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, "default_allow_overlap", rclcpp::ParameterValue(false));
    default_allow_overlap_ = node->get_parameter("default_allow_overlap").as_double();

    // Swath Generator BruteForce can also accept non-brute force specific
    // angle requests from SwathGeneratorBase, thus no need to change generators
    generator_ = std::make_unique<f2c::sg::BruteForce>();
    default_objective_ = createObjective(default_type_);
  }

  /**
   * @brief Main method to generate swaths in field
   * @param field Field to generate swaths from
   * @param request Action request information
   */
  Swaths generateSwaths(const Field & field /*, request*/);
  /**
   * @brief Sets the mode manually of the swath for dynamic parameters
   * @param mode String for mode to use
   */
  void setSwathMode(const std::string & new_mode);

  /**
   * @brief Sets the mode manually of the swath angle for dynamic parameters
   * @param mode String for mode to use
   */
  void setSwathAngleMode(const std::string & new_mode);

protected:
  /**
   * @brief Creates objective pointer of a requested type
   * @param type Swaths generator type to create
   * @return Generator to use
   */
  SwathObjectivePtr createObjective(const SwathType & type);

  /**
   * @brief Converts the swatch mode into a string for publication
   * @param Swath type
   * @param Swath angle type
   * @return String of mode
   */
  std::string toString(const SwathType & type, const SwathAngleType & angle_type);

  /**
   * @brief Converts the swatch string into a type for handling
   * @param str Type string to convert
   * @return Type of mode
   */
  SwathType toType(std::string & str);

  /**
   * @brief Converts the swatch angle string into a type for handling
   * @param str Type string to convert
   * @return Type of mode
   */
  SwathAngleType toAngleType(std::string & str);
  SwathType default_type_;
  SwathAngleType default_angle_type_;
  SwathObjectivePtr default_objective_;
  double default_swath_angle_;
  bool default_allow_overlap_;
  std::unique_ptr<f2c::sg::BruteForce> generator_;
  RobotMode * robot_;
  rclcpp::Logger logger_{rclcpp::get_logger("SwathGenerator")};
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__SWATH_MODE_HPP_
