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

#include <cmath>
#include <filesystem>

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
  void setCartesianFrame(bool v) {cartesian_frame_ = v;}
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
  action_client->wait_for_action_server();

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

TEST(ServerTest, testDecompPath)
{
  // Create server
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_decomp");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;
  goal_msg.generate_decomp = true;
  goal_msg.decomp_mode.mode = "TRAPEZOIDAL";
  goal_msg.generate_headland = true;
  // Covers decompose + headland + swath only; route/path is covered elsewhere.
  goal_msg.generate_route = false;
  goal_msg.generate_path = false;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::filesystem::path share_dir =
    ament_index_cpp::get_package_share_directory("opennav_coverage");
#pragma GCC diagnostic pop
  goal_msg.gml_field = (share_dir / "test_field.xml").string();

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST(ServerTest, testDecompNonTSPRouteRejected)
{
  // Non-TSP route modes only handle a single cell, so combining one with
  // decomposition (multi-cell) must be rejected with INVALID_MODE_SET.
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_decomp_nontsp");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;
  goal_msg.generate_decomp = true;
  goal_msg.decomp_mode.mode = "TRAPEZOIDAL";
  goal_msg.generate_headland = true;
  goal_msg.generate_route = true;
  goal_msg.route_mode.mode = "BOUSTROPHEDON";  // non-TSP -> rejected with decomposition
  goal_msg.generate_path = true;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::filesystem::path share_dir =
    ament_index_cpp::get_package_share_directory("opennav_coverage");
#pragma GCC diagnostic pop
  goal_msg.gml_field = (share_dir / "test_field.xml").string();

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(
    result.result->error_code,
    opennav_coverage_msgs::action::ComputeCoveragePath::Result::INVALID_MODE_SET);
}

TEST(ServerTest, testDecompPathNoHeadland)
{
  // generate_headland=false exercises the decompose-without-headland swath branch
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_decomp_nohl");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;
  goal_msg.generate_decomp = true;
  goal_msg.decomp_mode.mode = "BOUSTROPHEDON";
  goal_msg.generate_headland = false;
  goal_msg.generate_route = false;
  goal_msg.generate_path = false;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::filesystem::path share_dir =
    ament_index_cpp::get_package_share_directory("opennav_coverage");
#pragma GCC diagnostic pop
  goal_msg.gml_field = (share_dir / "test_field.xml").string();

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST(ServerTest, testTSPRouteNoPath)
{
  // TSP with generate_path=false now succeeds (returns ordered swaths) instead of
  // being rejected — the unified route path removed the old guard.
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_tsp_nopath");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;
  goal_msg.generate_route = true;
  goal_msg.generate_path = false;
  goal_msg.route_mode.mode = "TSP";
  goal_msg.route_mode.tsp_time_limit = 1;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::filesystem::path share_dir =
    ament_index_cpp::get_package_share_directory("opennav_coverage");
#pragma GCC diagnostic pop
  goal_msg.gml_field = (share_dir / "test_field.xml").string();

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result.result->coverage_path.swaths.empty());
}

TEST(ServerTest, testTSPDecompHeadlandPath)
{
  // TSP + decompose + headland end-to-end. test_field.xml has enough swaths to
  // exceed the global-genRoute cap, so this exercises the per-cell TSP fallback.
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_tsp_decomp_hl");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;
  goal_msg.generate_decomp = true;
  goal_msg.decomp_mode.mode = "TRAPEZOIDAL";
  goal_msg.generate_headland = true;
  goal_msg.generate_route = true;
  goal_msg.generate_path = true;
  goal_msg.route_mode.mode = "TSP";
  // Keep the per-cell OR-Tools search short so CI stays fast.
  goal_msg.route_mode.tsp_time_limit = 1;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::filesystem::path share_dir =
    ament_index_cpp::get_package_share_directory("opennav_coverage");
#pragma GCC diagnostic pop
  goal_msg.gml_field = (share_dir / "test_field.xml").string();

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result.result->nav_path.poses.empty());
  // Sanity-check the computed path: swaths, connection turns, and task time.
  EXPECT_FALSE(result.result->coverage_path.swaths.empty());
  EXPECT_TRUE(result.result->coverage_path.contains_turns);
  EXPECT_TRUE(std::isfinite(result.result->task_time));
}

TEST(ServerTest, testTSPNoDecompPath)
{
  // TSP on a single (non-decomposed) field: the common case, single genRoute call.
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_tsp_nodecomp");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;
  goal_msg.generate_decomp = false;
  goal_msg.generate_headland = true;
  goal_msg.generate_route = true;
  goal_msg.route_mode.mode = "TSP";
  goal_msg.route_mode.tsp_time_limit = 1;
  goal_msg.generate_path = true;
  goal_msg.use_start_pose = false;  // baseline: start point unset
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::filesystem::path share_dir =
    ament_index_cpp::get_package_share_directory("opennav_coverage");
#pragma GCC diagnostic pop
  goal_msg.gml_field = (share_dir / "test_field.xml").string();

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result.result->nav_path.poses.empty());
  // Sanity-check the computed path: swaths, connection turns, and task time.
  EXPECT_FALSE(result.result->coverage_path.swaths.empty());
  EXPECT_TRUE(result.result->coverage_path.contains_turns);
  EXPECT_TRUE(std::isfinite(result.result->task_time));
}

TEST(ServerTest, testTSPStartPose)
{
  // TSP with use_start_pose starts the path at the given GPS point
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_tsp_startpose");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.use_gml_file = true;
  goal_msg.generate_decomp = false;
  goal_msg.generate_headland = true;
  goal_msg.generate_route = true;
  goal_msg.route_mode.mode = "TSP";
  goal_msg.route_mode.tsp_time_limit = 1;
  goal_msg.generate_path = true;
  goal_msg.use_start_pose = true;
  // First outer-boundary vertex of test_field.xml (GPS); the route/path must begin here.
  goal_msg.start_pose.axis1 = 4.26199990317851;
  goal_msg.start_pose.axis2 = 51.7859704975047;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::filesystem::path share_dir =
    ament_index_cpp::get_package_share_directory("opennav_coverage");
#pragma GCC diagnostic pop
  goal_msg.gml_field = (share_dir / "test_field.xml").string();

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_FALSE(result.result->nav_path.poses.empty());
  // Output is GPS, so the first pose should sit at the requested start vertex
  EXPECT_NEAR(result.result->nav_path.poses.front().pose.position.x, 4.26199990317851, 1e-4);
  EXPECT_NEAR(result.result->nav_path.poses.front().pose.position.y, 51.7859704975047, 1e-4);
}

TEST(ServerTest, testTSPStartPoseCartesian)
{
  // Cartesian start pose must land at (5,5), not shifted by the field ref point
  auto node = std::make_shared<ServerShim>();
  rclcpp_lifecycle::State state;
  node->configure(state);
  node->setCartesianFrame(true);
  node->activate(state);
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  auto client_node = std::make_shared<rclcpp::Node>("my_node_tsp_startpose_cart");
  auto action_client =
    rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(
    client_node, "compute_coverage_path");
  action_client->wait_for_action_server();

  auto goal_msg = opennav_coverage_msgs::action::ComputeCoveragePath::Goal();
  goal_msg.frame_id = "map";
  goal_msg.generate_headland = false;
  goal_msg.generate_route = true;
  goal_msg.generate_path = true;
  goal_msg.route_mode.mode = "TSP";
  goal_msg.route_mode.tsp_time_limit = 1;
  goal_msg.use_start_pose = true;
  goal_msg.start_pose.axis1 = 5.0;
  goal_msg.start_pose.axis2 = 5.0;
  goal_msg.polygons.resize(1);
  for (const auto & xy : {std::pair<double, double>{5.0, 5.0}, {45.0, 5.0}, {45.0, 45.0},
      {5.0, 45.0}, {5.0, 5.0}})
  {
    opennav_coverage_msgs::msg::Coordinate c;
    c.axis1 = xy.first;
    c.axis2 = xy.second;
    goal_msg.polygons[0].coordinates.push_back(c);
  }

  auto future_goal_handle = action_client->async_send_goal(goal_msg);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_goal_handle),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = future_goal_handle.get();

  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_FALSE(result.result->nav_path.poses.empty());
  EXPECT_NEAR(result.result->nav_path.poses.front().pose.position.x, 5.0, 1e-3);
  EXPECT_NEAR(result.result->nav_path.poses.front().pose.position.y, 5.0, 1e-3);
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
      rclcpp::Parameter("default_custom_order", std::vector<int>{1, 2, 3}),
      // B1-T13: TSP dynamic params
      rclcpp::Parameter("default_tsp_redirect_swaths", false),
      rclcpp::Parameter("default_tsp_time_limit", 5),
      rclcpp::Parameter("default_tsp_search_for_optimum", true),
      rclcpp::Parameter("default_tsp_d_tol", 1e-3)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("default_headland_width").as_double(), 0.2);
  EXPECT_EQ(node->get_parameter("default_headland_type").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("default_allow_overlap").as_bool(), true);
  EXPECT_EQ(node->get_parameter("default_spiral_n").as_int(), 41);
  EXPECT_EQ(node->get_parameter("coordinates_in_cartesian_frame").as_bool(), false);
  // B1-T13: verify TSP params are declared and callback-connected
  EXPECT_EQ(node->get_parameter("default_tsp_redirect_swaths").as_bool(), false);
  EXPECT_EQ(node->get_parameter("default_tsp_time_limit").as_int(), 5);
  EXPECT_EQ(node->get_parameter("default_tsp_search_for_optimum").as_bool(), true);
  EXPECT_NEAR(node->get_parameter("default_tsp_d_tol").as_double(), 1e-3, 1e-9);
}

}  // namespace opennav_coverage
