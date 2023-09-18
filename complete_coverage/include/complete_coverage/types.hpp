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

#ifndef COMPLETE_COVERAGE__TYPES_HPP_
#define COMPLETE_COVERAGE__TYPES_HPP_

#include <vector>
#include <string>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "complete_coverage/utils.hpp"

namespace complete_coverage
{
// TODO(sm) all of these: what about action API parameters? Store things like angle/spiral_n/headlands dist or part of action?
// TODO(sm) robot
// TODO(sm) opt parms
// TODO(sm) ovelap?

/**
 * @enum Headlands computations types
 */
enum class HeadlandsType
{
  UNKNOWN = 0,
  CONSTANT = 1
};

/**
 * @struct Headlands state
 */
struct HeadlandsMode
{
  /**
   * @brief Constructor for headlands mode
   * @param node A node to get the headlands type from
   */
  template<typename NodeT>
  explicit HeadlandsMode(const NodeT & node)
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "HeadlandsType", rclcpp::ParameterValue("CONSTANT"));
    std::string type_str = node->get_parameter("HeadlandsType").as_string();
    toUpper(type_str);

    if (type_str == "CONSTANT") {
      type = HeadlandsType::CONSTANT;
    } else {
      type = HeadlandsType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the headlands for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & mode)
  {
    if (mode == "CONSTANT") {
      type = HeadlandsType::CONSTANT;
    } else {
      type = HeadlandsType::UNKNOWN;
    }
  }

  /**
   * @brief Converts the headlands mode into a string for publication
   * @return String of mode
   */
  std::string toString()
  {
    switch (type) {
      case HeadlandsType::CONSTANT:
        return "Constant";
      default:
        return "Unknown";
    }
  }

  // TODO(sm) store distance to use?
  HeadlandsType type;
};

/**
 * @enum Swath computations types
 */
enum class SwathType
{
  UNKNOWN = 0,
  LENGTH = 1,
  NUMBER = 2,
  COVERAGE = 3
};

/**
 * @enum Swath angle types
 */
enum class SwathAngleType
{
  UNKNOWN = 0,
  SET_ANGLE = 1,
  BRUTE_FORCE = 2
};

/**
 * @struct Swath state
 */
struct SwathMode
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
      type = SwathType::LENGTH;
    } else if (type_str == "NUMBER") {
      type = SwathType::NUMBER;
    } else if (type_str == "COVERAGE") {
      type = SwathType::COVERAGE;
    } else {
      type = SwathType::UNKNOWN;
    }

    if (angle_str == "BRUTE_FORCE") {
      angle_type = SwathAngleType::BRUTE_FORCE;
    } else if (type_str == "SET_ANGLE") {
      angle_type = SwathAngleType::SET_ANGLE;
    } else {
      angle_type = SwathAngleType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the swath for dynamic parameters
   * @param mode String for mode to use
   */
  void setSwathMode(const std::string & mode)
  {
    // TODO(sm) store angle to use too?
    if (mode == "LENGTH") {
      type = SwathType::LENGTH;
    } else if (mode == "NUMBER") {
      type = SwathType::NUMBER;
    } else if (mode == "COVERAGE") {
      type = SwathType::COVERAGE;
    } else {
      type = SwathType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the swath angle for dynamic parameters
   * @param mode String for mode to use
   */
  void setSwathAngleMode(const std::string & mode)
  {
    if (mode == "SET_ANGLE") {
      angle_type = SwathAngleType::SET_ANGLE;
    } else if (mode == "BRUTE_FORCE") {
      angle_type = SwathAngleType::BRUTE_FORCE;
    } else {
      angle_type = SwathAngleType::UNKNOWN;
    }
  }

  /**
   * @brief Converts the swatch mode into a string for publication
   * @return String of mode
   */
  std::string toString()
  {
    std::string str;
    switch (type) {
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

    switch (angle_type) {
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

  SwathType type;
  SwathAngleType angle_type;
};

/**
 * @enum Swath computations types
 */
enum class RouteType
{
  UNKNOWN = 0,
  BOUSTROPHEDON = 1,
  SNAKE = 2,
  SPIRAL = 3,
  CUSTOM = 4
};

struct RouteMode
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
      type = RouteType::BOUSTROPHEDON;
    } else if (type_str == "SNAKE") {
      type = RouteType::SNAKE;
    } else if (type_str == "SPIRAL") {
      type = RouteType::SPIRAL;
    } else if (type_str == "CUSTOM") {
      type = RouteType::CUSTOM;
    } else {
      type = RouteType::UNKNOWN;
    }

    if (type == RouteType::CUSTOM) {
      // TODO(sm) handle throw
      nav2_util::declare_parameter_if_not_declared(
        node, "custom_order", rclcpp::PARAMETER_INTEGER_ARRAY);
      custom_order = node->get_parameter("custom_order").as_int_array();
    } else if (type == RouteType::SPIRAL) {
      nav2_util::declare_parameter_if_not_declared(
        node, "spiral_n", rclcpp::PARAMETER_INTEGER);
      spiral_n = node->get_parameter("spiral_n").as_int();
    }
  }

  /**
   * @brief Sets the mode manually of the route for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & mode, const std::vector<size_t> & new_order = {})
  {
    if (mode == "BOUSTROPHEDON") {
      type = RouteType::BOUSTROPHEDON;
    } else if (mode == "SNAKE") {
      type = RouteType::SNAKE;
    } else if (mode == "SPIRAL") {
      type = RouteType::SPIRAL;
    } else if (mode == "CUSTOM") {
      type = RouteType::CUSTOM;
    } else {
      type = RouteType::UNKNOWN;
    }

    if (type == RouteType::CUSTOM) {
      custom_order = new_order;
    }
  }

  /**
   * @brief Converts the route mode into a string for publication
   * @return String of mode
   */
  std::string toString()
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
      default:
        return "Unknown";
    }
  }

  RouteType type;
  std::vector<size_t> custom_order;
  size_t spiral_n;
};

/**
 * @enum Path computations types
 */
enum class PathType
{
  UNKNOWN = 0,
  REEDS_SHEPP = 1,
  DUBIN = 2,
};

/**
 * @enum Path continuity types
 */
enum class PathContinuityType
{
  UNKNOWN = 0,
  CONTINUOUS = 1,
  DISCONTINUOUS = 2
};

struct PathMode
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
      type = PathType::DUBIN;
    } else if (type_str == "REEDS_SHEPP") {
      type = PathType::REEDS_SHEPP;
    } else {
      type = PathType::UNKNOWN;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, "PathContinuityType", rclcpp::ParameterValue("CONTINUOUS"));
    std::string type_cont_str = node->get_parameter("PathContinuityType").as_string();
    toUpper(type_str);

    if (type_cont_str == "CONTINUOUS") {
      continuity_type = PathContinuityType::CONTINUOUS;
    } else if (type_cont_str == "DISCONTINUOUS") {
      continuity_type = PathContinuityType::DISCONTINUOUS;
    } else {
      continuity_type = PathContinuityType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the planner for dynamic parameters
   * @param mode String for mode to use
   */
  void setPathMode(const std::string & mode)
  {
    if (mode == "DUBIN") {
      type = PathType::DUBIN;
    } else if (mode == "REEDS_SHEPP") {
      type = PathType::REEDS_SHEPP;
    } else {
      type = PathType::UNKNOWN;
    }
  }

  /**
   * @brief Sets the mode manually of the planner for dynamic parameters
   * @param mode String for mode to use
   */
  void setPathContinuityMode(const std::string & mode)
  {
    if (mode == "CONTINUOUS") {
      continuity_type = PathContinuityType::CONTINUOUS;
    } else if (mode == "DISCONTINUOUS") {
      continuity_type = PathContinuityType::DISCONTINUOUS;
    } else {
      continuity_type = PathContinuityType::UNKNOWN;
    }
  }

  /**
   * @brief Converts the path mode into a string for publication
   * @return String of mode
   */
  std::string toString()
  {
    std::string str;
    switch (type) {
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

    switch (continuity_type) {
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

  PathType type;
  PathContinuityType continuity_type;
};

}  // namespace complete_coverage

#endif  // COMPLETE_COVERAGE__TYPES_HPP_
