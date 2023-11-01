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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "opennav_coverage/coverage_server.hpp"
#include "tf2/utils.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

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

class ServerShim : public opennav_coverage::CoverageServer
{
public:
  ServerShim()
  : CoverageServer()
  {}
  void configure(const rclcpp_lifecycle::State & state)
  {
    this->on_configure(state);
    cartesian_frame_ = false;  // Test files in GPS
  }
  void activate(const rclcpp_lifecycle::State & state) {this->on_activate(state);}
  void deactivate(const rclcpp_lifecycle::State & state) {this->on_deactivate(state);}
  void cleanup(const rclcpp_lifecycle::State & state) {this->on_cleanup(state);}
  void shutdown(const rclcpp_lifecycle::State & state) {this->on_shutdown(state);}

  bool validateGoalShim(typename std::shared_ptr<const typename ComputeCoveragePath::Goal> req)
  {
    return validateGoal(req);
  }
};

TEST(ServerTest, LifecycleTest)
{
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  node->deactivate(state);
  node->cleanup(state);
  node->shutdown(state);
  node.reset();
}

TEST(ServerTest, testUtils)
{
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);

  auto req = std::make_shared<typename ComputeCoveragePath::Goal>();
  EXPECT_TRUE(node->validateGoalShim(req));
  req->generate_route = false;
  EXPECT_FALSE(node->validateGoalShim(req));
}

TEST(ServerTest, testServerTransactions)
{
  // Create server
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  // Send some requests
  auto client_node = std::make_shared<rclcpp::Node>("my_node");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;  // Use file
  goal_msg.gml_field =
    ament_index_cpp::get_package_share_directory("opennav_coverage") + "/test_field.xml";

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      client_node,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = future_goal_handle.get();

  // Wait for the result
  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  // The final result
  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST(ServerTest, testDynamicParams)
{
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("default_headland_width", 0.2),
      rclcpp::Parameter("default_swath_angle", 1.0),
      rclcpp::Parameter("default_step_angle", 1.2),
      rclcpp::Parameter("default_turn_point_distance", 0.25),
      rclcpp::Parameter("robot_width", 1.0),
      rclcpp::Parameter("operation_width", 1.12),
      rclcpp::Parameter("default_headland_type", std::string("hi")),
      rclcpp::Parameter("default_path_type", std::string("hi")),
      rclcpp::Parameter("default_path_continuity_type", std::string("hi")),
      rclcpp::Parameter("default_route_type", std::string("hi")),
      rclcpp::Parameter("default_swath_type", std::string("hi")),
      rclcpp::Parameter("default_swath_angle_type", std::string("hi")),
      rclcpp::Parameter("default_allow_overlap", true),
      rclcpp::Parameter("default_spiral_n", 41),
      rclcpp::Parameter("coordinates_in_cartesian_frame", false),
      rclcpp::Parameter("default_custom_order", std::vector<int>{1, 2, 3})});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("default_headland_width").as_double(), 0.2);
  EXPECT_EQ(node->get_parameter("default_headland_type").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("default_allow_overlap").as_bool(), true);
  EXPECT_EQ(node->get_parameter("default_spiral_n").as_int(), 41);
  EXPECT_EQ(node->get_parameter("coordinates_in_cartesian_frame").as_bool(), false);
}

}  // namespace opennav_coverage
