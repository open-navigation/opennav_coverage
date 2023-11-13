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
#include "opennav_coverage_navigator/coverage_navigator.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_coverage
{

inline std::vector<std::string> getLibs()
{
  return std::vector<std::string>{
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_assisted_teleop_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node",
    "nav2_back_up_cancel_bt_node",
    "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node",
    "opennav_compute_complete_coverage_action_bt_node",
    "opennav_cancel_complete_coverage_action_bt_node"
  };
}

TEST(CoverageNavigatorTests, TestBasicFunctionality)
{
  opennav_coverage_navigator::CoverageNavigator navigator;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto odom_smoother = std::make_shared<nav2_util::OdomSmoother>(node, 0.3, "odom");
  backported_bt_navigator::NavigatorMuxer plugin_muxer;

  backported_bt_navigator::FeedbackUtils feedback_utils;
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(), node->get_node_timers_interface());
  tf->setCreateTimerInterface(timer_interface);
  tf->setUsingDedicatedThread(true);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, false);
  feedback_utils.tf = tf;
  feedback_utils.global_frame = "map";
  feedback_utils.robot_frame = "base_link";
  feedback_utils.transform_tolerance = 0.1;

  navigator.on_configure(node, getLibs(), feedback_utils, &plugin_muxer, odom_smoother);
  navigator.on_activate();

  // Gets a proper path to a BT
  std::string path = navigator.getDefaultBTFilepath(node);
  EXPECT_EQ(
    path.substr(path.size() - 38),
    std::string("navigate_w_basic_complete_coverage.xml"));

  // get navigator name / action string
  EXPECT_EQ(navigator.getName(), std::string("navigate_complete_coverage"));

  navigator.on_deactivate();
  navigator.on_cleanup();
}

TEST(CoverageNavigatorTests, TestBasicServer)
{
  // Create server
  opennav_coverage_navigator::CoverageNavigator navigator;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto odom_smoother = std::make_shared<nav2_util::OdomSmoother>(node, 0.3, "odom");
  backported_bt_navigator::NavigatorMuxer plugin_muxer;

  backported_bt_navigator::FeedbackUtils feedback_utils;
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(), node->get_node_timers_interface());
  tf->setCreateTimerInterface(timer_interface);
  tf->setUsingDedicatedThread(true);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, false);
  feedback_utils.tf = tf;
  feedback_utils.global_frame = "map";
  feedback_utils.robot_frame = "base_link";
  feedback_utils.transform_tolerance = 0.1;

  navigator.on_configure(node, getLibs(), feedback_utils, &plugin_muxer, odom_smoother);
  navigator.on_activate();
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  // Call server
  auto client_node = std::make_shared<rclcpp::Node>("my_node");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::NavigateCompleteCoverage>(
    client_node, "navigate_complete_coverage");

  auto goal_msg = opennav_coverage_msgs::action::NavigateCompleteCoverage::Goal();
  goal_msg.field_filepath =
    ament_index_cpp::get_package_share_directory("opennav_coverage") + "/test_field.xml";

  // Send it
  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  rclcpp::Rate r1(1.0);
  r1.sleep();

  // Preempt it
  auto future_goal_handle2 = action_client->async_send_goal(goal_msg);
  rclcpp::Rate r2(1.0);
  r2.sleep();

  // Cancel it
  action_client->async_cancel_all_goals();

  navigator.on_deactivate();
  navigator.on_cleanup();
}

}  // namespace opennav_coverage
