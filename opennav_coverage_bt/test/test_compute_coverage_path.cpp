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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "opennav_coverage_bt/compute_complete_coverage_path.hpp"

class ComputeCompleteCoveragePathActionServer
  : public TestActionServer<opennav_coverage_msgs::action::ComputeCoveragePath>
{
public:
  ComputeCompleteCoveragePathActionServer()
  : TestActionServer("compute_coverage_path")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<opennav_coverage_msgs::action::ComputeCoveragePath>>
    goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result =
      std::make_shared<opennav_coverage_msgs::action::ComputeCoveragePath::Result>();
    result->nav_path.poses.resize(2);
    result->nav_path.poses[1].pose.position.x = 1.0;
    result->nav_path.poses[0].pose.position.x = 0.0;
    goal_handle->succeed(result);
  }
};

class ComputeCoveragePathActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("compute_coverage_path_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(1000));
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<opennav_coverage_bt::ComputeCoveragePathAction>(
          name, "compute_coverage_path", config);
      };

    factory_->registerBuilder<opennav_coverage_bt::ComputeCoveragePathAction>(
      "ComputeCoveragePath", builder);
  }

  static void TearDownTestCase()
  {
    factory_.reset();
    action_server_.reset();
    delete config_;
    config_ = nullptr;
    node_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ComputeCompleteCoveragePathActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ComputeCoveragePathActionTestFixture::node_ = nullptr;
std::shared_ptr<ComputeCompleteCoveragePathActionServer>
ComputeCoveragePathActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * ComputeCoveragePathActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ComputeCoveragePathActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ComputeCoveragePathActionTestFixture::tree_ = nullptr;

TEST_F(ComputeCoveragePathActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ComputeCoveragePath nav_path="{path}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // check if returned path is correct
  nav_msgs::msg::Path path;
  config_->blackboard->get<nav_msgs::msg::Path>("path", path);
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, 0.0);
  EXPECT_EQ(path.poses[1].pose.position.x, 1.0);

  // halt node so another goal can be sent
  tree_->rootNode()->halt();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  ComputeCoveragePathActionTestFixture::action_server_ =
    std::make_shared<ComputeCompleteCoveragePathActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(ComputeCoveragePathActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  // rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
