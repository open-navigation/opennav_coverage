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

#ifndef COMPLETE_COVERAGE__SWATH_MODE_HPP_
#define COMPLETE_COVERAGE__SWATH_MODE_HPP_

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
 * @class Swath mode and state
 */
class SwathMode
{
  /**
   * @brief Constructor for swath mode
   * @param node A node to get the swath type from
   */
  template<typename NodeT>
  explicit SwathMode(const NodeT & node)
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "SwathType", rclcpp::ParameterValue("LENGTH"));
    std::string type_str = node->get_parameter("SwathType").as_string();
    toUpper(type_str);

    nav2_util::declare_parameter_if_not_declared(
      node, "SwathAngleType", rclcpp::ParameterValue("BRUTE_FORCE"));
    std::string angle_str = node->get_parameter("SwathAngleType").as_string();
    toUpper(angle_str);

    if (type_str == "LENGTH") {
      type_ = SwathType::LENGTH;
    } else if (type_str == "NUMBER") {
      type_ = SwathType::NUMBER;
    } else if (type_str == "COVERAGE") {
      type_ = SwathType::COVERAGE;
    } else {
      type_ = SwathType::UNKNOWN;
    }

    if (angle_str == "BRUTE_FORCE") {
      angle_type_ = SwathAngleType::BRUTE_FORCE;
    } else if (type_str == "SET_ANGLE") {
      angle_type_ = SwathAngleType::SET_ANGLE;
    } else {
      angle_type_ = SwathAngleType::UNKNOWN;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, "default_swath_angle", rclcpp::ParameterValue(0.0));
    default_swath_angle_ = node->get_parameter("default_swath_angle").as_double();
  }

  /**
   * @brief Sets the mode manually of the swath for dynamic parameters
   * @param mode String for mode to use
   */
  void setSwathMode(const std::string & mode)
  {
    if (mode == "LENGTH") {
      type_ = SwathType::LENGTH;
    } else if (mode == "NUMBER") {
      type_ = SwathType::NUMBER;
    } else if (mode == "COVERAGE") {
      type_ = SwathType::COVERAGE;
    } else {
      type_ = SwathType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the swath angle for dynamic parameters
   * @param mode String for mode to use
   */
  void setSwathAngleMode(const std::string & mode)
  {
    if (mode == "SET_ANGLE") {
      angle_type_ = SwathAngleType::SET_ANGLE;
    } else if (mode == "BRUTE_FORCE") {
      angle_type_ = SwathAngleType::BRUTE_FORCE;
    } else {
      angle_type_ = SwathAngleType::UNKNOWN;
    }
  }

  /**
   * @brief Converts the swatch mode into a string for publication
   * @return String of mode
   */
  std::string toString()
  {
    std::string str;
    switch (type_) {
      case SwathType::LENGTH:
        str = "Length";
        break;
      case SwathType::NUMBER:
        str = "Number";
        break;
      case SwathType::COVERAGE:
        str = "Coverage";
        break;
      default:
        str = "Unknown";
        break;
    }

    str += " Objective and ";

    switch (angle_type_) {
      case SwathAngleType::SET_ANGLE:
        str = "Set Angle";
        break;
      case SwathAngleType::BRUTE_FORCE:
        str = "Brute Force";
        break;
      default:
        str = "Unknown";
        break;
    }

    str += " angle.";
    return str;
  }

  SwathType type_;
  SwathAngleType angle_type_;
  double default_swath_angle_;
};

}  // namespace complete_coverage

#endif  // COMPLETE_COVERAGE__SWATH_MODE_HPP_
