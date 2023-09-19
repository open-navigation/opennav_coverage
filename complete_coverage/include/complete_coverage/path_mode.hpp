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

#ifndef COMPLETE_COVERAGE__PATH_MODE_HPP_
#define COMPLETE_COVERAGE__PATH_MODE_HPP_

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
 * @class Path mode and state
 */
class PathMode
{
  /**
   * @brief Constructor for swath mode
   * @param node A node to get the swath type from
   */
  template<typename NodeT>
  explicit PathMode(const NodeT & node)
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "PathType", rclcpp::ParameterValue("DUBIN"));
    std::string type_str = node->get_parameter("PathType").as_string();
    toUpper(type_str);

    if (type_str == "DUBIN") {
      type_ = PathType::DUBIN;
    } else if (type_str == "REEDS_SHEPP") {
      type_ = PathType::REEDS_SHEPP;
    } else {
      type_ = PathType::UNKNOWN;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, "PathContinuityType", rclcpp::ParameterValue("CONTINUOUS"));
    std::string type_cont_str = node->get_parameter("PathContinuityType").as_string();
    toUpper(type_str);

    if (type_cont_str == "CONTINUOUS") {
      continuity_type_ = PathContinuityType::CONTINUOUS;
    } else if (type_cont_str == "DISCONTINUOUS") {
      continuity_type_ = PathContinuityType::DISCONTINUOUS;
    } else {
      continuity_type_ = PathContinuityType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the planner for dynamic parameters
   * @param mode String for mode to use
   */
  void setPathMode(const std::string & mode)
  {
    if (mode == "DUBIN") {
      type_ = PathType::DUBIN;
    } else if (mode == "REEDS_SHEPP") {
      type_ = PathType::REEDS_SHEPP;
    } else {
      type_ = PathType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the planner for dynamic parameters
   * @param mode String for mode to use
   */
  void setPathContinuityMode(const std::string & mode)
  {
    if (mode == "CONTINUOUS") {
      continuity_type_ = PathContinuityType::CONTINUOUS;
    } else if (mode == "DISCONTINUOUS") {
      continuity_type_ = PathContinuityType::DISCONTINUOUS;
    } else {
      continuity_type_ = PathContinuityType::UNKNOWN;
    }
  }

  /**
   * @brief Converts the path mode into a string for publication
   * @return String of mode
   */
  std::string toString()
  {
    std::string str;
    switch (type_) {
      case PathType::DUBIN:
        str = "Dubin";
        break;
      case PathType::REEDS_SHEPP:
        str = "Reeds-Shepp";
        break;
      default:
        str = "Unknown";
        break;
    }

    str += " Mode and ";

    switch (continuity_type_) {
      case PathContinuityType::CONTINUOUS:
        str = "Continuous";
        break;
      case PathContinuityType::DISCONTINUOUS:
        str = "Discontinuous";
        break;
      default:
        str = "Unknown";
        break;
    }

    str += " connections.";
    return str;
  }

  PathType type_;
  PathContinuityType continuity_type_;
};

}  // namespace complete_coverage

#endif  // COMPLETE_COVERAGE__PATH_MODE_HPP_
