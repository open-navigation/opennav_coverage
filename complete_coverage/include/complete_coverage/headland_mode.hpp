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

#ifndef COMPLETE_COVERAGE__HEADLAND_MODE_HPP_
#define COMPLETE_COVERAGE__HEADLAND_MODE_HPP_

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
 * @class Headland mode and state
 */
class HeadlandMode
{
  /**
   * @brief Constructor for Headland mode
   * @param node A node to get the Headland type from
   */
  template<typename NodeT>
  explicit HeadlandMode(const NodeT & node)
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "HeadlandType", rclcpp::ParameterValue("CONSTANT"));
    std::string type_str = node->get_parameter("HeadlandType").as_string();
    toUpper(type_str);

    if (type_str == "CONSTANT") {
      type_ = HeadlandType::CONSTANT;
    } else {
      type_ = HeadlandType::UNKNOWN;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, "default_headland_width", rclcpp::ParameterValue(2.0));
    default_headland_width_ = node->get_parameter("default_headland_width").as_double();
  }

  /**
   * @brief Sets the mode manually of the Headland for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & mode)
  {
    if (mode == "CONSTANT") {
      type_ = HeadlandType::CONSTANT;
    } else {
      type_ = HeadlandType::UNKNOWN;
    }
  }

  /**
   * @brief Converts the Headland mode into a string for publication
   * @return String of mode
   */
  std::string toString()
  {
    switch (type_) {
      case HeadlandType::CONSTANT:
        return "Constant";
      default:
        return "Unknown";
    }
  }

  HeadlandType type_;
  double default_headland_width_;
};

}  // namespace complete_coverage

#endif  // COMPLETE_COVERAGE__HEADLAND_MODE_HPP_
