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

#ifndef NAV2_COVERAGE__HEADLAND_MODE_HPP_
#define NAV2_COVERAGE__HEADLAND_MODE_HPP_

#include <vector>
#include <string>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_coverage/utils.hpp"
#include "nav2_coverage/types.hpp"

namespace nav2_coverage
{

/**
 * @class Headland mode and state
 */
class HeadlandMode
{
public:
  /**
   * @brief Constructor for Headland mode
   * @param node A node to get the Headland type from
   */
  template<typename NodeT>
  explicit HeadlandMode(const NodeT & node)
  {
    logger_ = node->get_logger();

    nav2_util::declare_parameter_if_not_declared(
      node, "default_headland_type", rclcpp::ParameterValue("CONSTANT"));
    std::string type_str = node->get_parameter("default_headland_type").as_string();
    default_type_ = toType(type_str);
    default_generator_ = createGenerator(default_type_);

    nav2_util::declare_parameter_if_not_declared(
      node, "default_headland_width", rclcpp::ParameterValue(2.0));
    default_headland_width_ = node->get_parameter("default_headland_width").as_double();
  }

  /**
   * @brief Main method to generate headlands
   * @param field Field to generate headlands from
   * @param request Action request information
   */
  Field generateHeadlands(const Fields & field /*, (void) request*/);

  /**
   * @brief Sets the mode manually of the Headland for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & new_mode);

protected:
  /**
   * @brief Creates generator pointer of a requested type
   * @param type Headland generator type to create
   * @return Generator to use
   */
  HeadlandGeneratorPtr createGenerator(const HeadlandType & type);

  /**
   * @brief Converts the Headland mode into a string for publication
   * @param Type of mode
   * @return String of mode
   */
  std::string toString(const HeadlandType & type);

  /**
   * @brief Converts the Headland string into a mode for handling
   * @param String of mode
   * @return Type of mode
   */
  HeadlandType toType(std::string & str);

  HeadlandType default_type_;
  double default_headland_width_;
  HeadlandGeneratorPtr default_generator_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("HeadlandGenerator")};
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__HEADLAND_MODE_HPP_
