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

#ifndef OPENNAV_COVERAGE__HEADLAND_GENERATOR_HPP_
#define OPENNAV_COVERAGE__HEADLAND_GENERATOR_HPP_

#include <vector>
#include <string>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_coverage_msgs/msg/headland_mode.hpp"
#include "opennav_coverage/utils.hpp"
#include "opennav_coverage/types.hpp"

namespace opennav_coverage
{

/**
 * @class Headland mode and state
 */
class HeadlandGenerator
{
public:
  /**
   * @brief Constructor for Headland mode
   * @param node A node to get the Headland type from
   */
  template<typename NodeT>
  explicit HeadlandGenerator(const NodeT & node)
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
   * @param settings Action request information
   */
  Field generateHeadlands(
    const Field & field, const opennav_coverage_msgs::msg::HeadlandMode & settings);

  /**
   * @brief Sets the mode manually of the Headland for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & new_mode);

  /**
   * @brief Sets the headland width manually for dynamic parameters
   * @param mode String for mode to use
   */
  void setWidth(const double & width) {default_headland_width_ = width;}

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
  HeadlandType toType(const std::string & str);

  HeadlandType default_type_;
  double default_headland_width_;
  HeadlandGeneratorPtr default_generator_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("HeadlandGenerator")};
};

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__HEADLAND_GENERATOR_HPP_
