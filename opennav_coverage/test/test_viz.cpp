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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/visualizer.hpp"
#include "tf2/utils.h"

// Luckily, F2C has very high test coverage so we only need to test what we touch

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_coverage
{

TEST(VizTests, Testlifecycle)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  Visualizer viz;
  viz.activate(node);
  viz.deactivate();
}

TEST(VizTests, TestVizPubs)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::Rate r(2.0);
  bool got_path_msg = false;
  bool got_headland_msg = false;
  bool got_planning_field_msg = false;
  bool got_swaths_msg = false;

  auto path_sub = node->create_subscription<nav_msgs::msg::Path>(
    "coverage_server/coverage_plan", rclcpp::QoS(1), [&](
      const nav_msgs::msg::Path::SharedPtr /*msg*/) {
      got_path_msg = true;
    });

  auto headland_sub = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "coverage_server/field_boundary", rclcpp::QoS(1), [&](
      const geometry_msgs::msg::PolygonStamped::SharedPtr /*msg*/) {
      got_headland_msg = true;
    });

  auto planning_field_sub = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "coverage_server/planning_field", rclcpp::QoS(1), [&](
      const geometry_msgs::msg::PolygonStamped::SharedPtr /*msg*/) {
      got_planning_field_msg = true;
    });

  auto swaths_sub = node->create_subscription<visualization_msgs::msg::Marker>(
    "coverage_server/swaths", rclcpp::QoS(1), [&](
      const visualization_msgs::msg::Marker::SharedPtr /*msg*/) {
      got_swaths_msg = true;
    });

  // give a moment for everything to register
  r.sleep();

  Visualizer viz;
  viz.activate(node);

  Polygon outer_polygon;
  outer_polygon.addPoint(Point(0.0, 1.0));
  outer_polygon.addPoint(Point(1.0, 1.0));
  outer_polygon.addPoint(Point(1.0, 1.0));
  outer_polygon.addPoint(Point(0.0, 1.0));
  Field total_field(outer_polygon);

  std_msgs::msg::Header header;
  const auto result = std::make_shared<typename ComputeCoveragePath::Result>();
  result->nav_path.poses.resize(20);
  result->coverage_path.swaths.resize(20);
  Field no_headland_field = total_field.clone();
  std::vector<Swath> swaths_raw;
  swaths_raw.resize(20);
  Swaths swaths(swaths_raw);
  nav_msgs::msg::Path path;
  viz.visualize(total_field, no_headland_field, Point(), path, swaths, header);

  // Give a moment to process for stability, then check if received
  rclcpp::spin_some(node);
  r.sleep();
  EXPECT_FALSE(got_path_msg);  // Path empty, unpublished
  EXPECT_TRUE(got_headland_msg);
  EXPECT_TRUE(got_planning_field_msg);
  EXPECT_TRUE(got_swaths_msg);
}

}  // namespace opennav_coverage
