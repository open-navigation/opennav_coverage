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

#ifndef OPENNAV_ROW_COVERAGE__ROW_COVERAGE_SERVER_HPP_
#define OPENNAV_ROW_COVERAGE__ROW_COVERAGE_SERVER_HPP_

#include <vector>
#include <memory>

#include "fields2cover.h" // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "opennav_row_coverage/row_swath_generator.hpp"
#include "opennav_coverage/route_generator.hpp"
#include "opennav_coverage/path_generator.hpp"
#include "opennav_coverage/visualizer.hpp"

namespace opennav_row_coverage
{
/**
 * @class opennav_row_coverage::RowCoverageServer
 * @brief An action server which implements highly reconfigurable complete
 * coverage planning using the Fields2Cover library with pre-established rows
 */
class RowCoverageServer : public nav2_util::LifecycleNode
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ComputeCoveragePath>;

  /**
   * @brief A constructor for opennav_row_coverage::RowCoverageServer
   * @param options Additional options to control creation of the node.
   */
  explicit RowCoverageServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for opennav_row_coverage::RowCoverageServer
   */
  ~RowCoverageServer() = default;

protected:
  /**
   * @brief Main action callback method to complete action request
   */
  void computeCoveragePath();

  /**
   * @brief Validate the goal settings to know if valid to execute
   * @param Goal request to validate
   * @return SUCCESS or FAILURE
   */
  bool validateGoal(typename std::shared_ptr<const typename ComputeCoveragePath::Goal> req);

  /**
   * @brief Gets a preempted goal if immediately requested
   * @param Goal goal to check or replace if required with preemption
   * @return SUCCESS or FAILURE
   */
  void getPreemptedGoalIfRequested(
    typename std::shared_ptr<const typename ComputeCoveragePath::Goal> goal);

  /**
   * @brief Configure member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  std::unique_ptr<ActionServer> action_server_;

  std::unique_ptr<opennav_coverage::RobotParams> robot_params_;
  std::unique_ptr<RowSwathGenerator> swath_gen_;
  std::unique_ptr<opennav_coverage::RouteGenerator> route_gen_;
  std::unique_ptr<opennav_coverage::PathGenerator> path_gen_;
  std::unique_ptr<opennav_coverage::Visualizer> visualizer_;
  bool cartesian_frame_, order_ids_;
};

}  // namespace opennav_row_coverage

#endif  // OPENNAV_ROW_COVERAGE__ROW_COVERAGE_SERVER_HPP_
