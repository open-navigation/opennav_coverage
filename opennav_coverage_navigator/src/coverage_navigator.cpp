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

#include <vector>
#include <string>
#include <memory>
#include <limits>
#include "opennav_coverage_navigator/coverage_navigator.hpp"

namespace opennav_coverage_navigator
{

bool
CoverageNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  if (!node->has_parameter("field_file_blackboard_id")) {
    node->declare_parameter("field_file_blackboard_id", std::string("field_filepath"));
  }
  field_blackboard_id_ = node->get_parameter("field_file_blackboard_id").as_string();

  if (!node->has_parameter("field_polygon_blackboard_id")) {
    node->declare_parameter("field_polygon_blackboard_id", std::string("field_polygon"));
  }
  polygon_blackboard_id_ = node->get_parameter("field_polygon_blackboard_id").as_string();

  if (!node->has_parameter("polygon_frame_blackboard_id")) {
    node->declare_parameter("polygon_frame_blackboard_id", std::string("polygon_frame_id"));
  }
  polygon_frame_blackboard_id_ = node->get_parameter("polygon_frame_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;
  return true;
}

std::string
CoverageNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_coverage_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("opennav_coverage_bt");
    node->declare_parameter<std::string>(
      "default_coverage_bt_xml",
      pkg_share_dir +
      "/behavior_trees/navigate_w_basic_complete_coverage.xml");
  }

  node->get_parameter("default_coverage_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
CoverageNavigator::cleanup()
{
  return true;
}

bool
CoverageNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPose(goal);
  return true;
}

void
CoverageNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
}

void
CoverageNavigator::onLoop()
{
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  auto blackboard = bt_action_server_->getBlackboard();

  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    [[maybe_unused]] auto res = blackboard->get(path_blackboard_id_, current_path);

    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx =
      [&current_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    // Calculate distance on the path
    double distance_remaining =
      nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // Get current speed
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // and at least 10cm to go
    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (...) {
    // Ignore
  }

  int recovery_count = 0;
  [[maybe_unused]] auto result = blackboard->get("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = clock_->now() - start_time_;

  bt_action_server_->publishFeedback(feedback_msg);
}

void
CoverageNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void
CoverageNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  if (goal->field_filepath.size() == 0) {
    RCLCPP_INFO(
      logger_, "Begin coverage navigating with outer field of size: %lu!",
      goal->polygons[0].points.size());
  } else {
    RCLCPP_INFO(
      logger_, "Begin coverage navigating with field %s!",
      goal->field_filepath.c_str());
  }

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the field to cover on the blackboard
  blackboard->set<std::string>(field_blackboard_id_, goal->field_filepath);
  blackboard->set<std::vector<geometry_msgs::msg::Polygon>>(
    polygon_blackboard_id_, goal->polygons);
  blackboard->set<std::string>(polygon_frame_blackboard_id_, goal->frame_id);
}

}  // namespace opennav_coverage_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  opennav_coverage_navigator::CoverageNavigator,
  nav2_core::NavigatorBase)
