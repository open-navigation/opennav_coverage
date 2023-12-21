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

#ifndef OPENNAV_COVERAGE__VISUALIZER_HPP_
#define OPENNAV_COVERAGE__VISUALIZER_HPP_

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/types.hpp"
#include "opennav_coverage/utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace opennav_coverage
{

class Visualizer
{
public:
  /**
   * @brief Constructor for visualizations
   * @param node A node to create interfaces from
   */
  Visualizer() = default;

  template<typename NodeT>
  void activate(NodeT node)
  {
    nav_plan_pub_ = rclcpp::create_publisher<nav_msgs::msg::Path>(
      node->get_node_topics_interface(),
      "coverage_server/coverage_plan", rclcpp::QoS(1));
    headlands_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PolygonStamped>(
      node->get_node_topics_interface(),
      "coverage_server/field_boundary", rclcpp::QoS(1));
    planning_field_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PolygonStamped>(
      node->get_node_topics_interface(),
      "coverage_server/planning_field", rclcpp::QoS(1));
    swaths_pub_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
      node->get_node_topics_interface(),
      "coverage_server/swaths", rclcpp::QoS(1));
  }

  void deactivate();

  void visualize(
    const Field & total_field, const Field & no_headland_field,
    const Point & ref_pt, const nav_msgs::msg::Path & path,
    const Swaths swaths, const std_msgs::msg::Header & header);

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr headlands_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr planning_field_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr swaths_pub_;
};

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__VISUALIZER_HPP_
