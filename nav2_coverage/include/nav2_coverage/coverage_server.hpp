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

#ifndef NAV2_COVERAGE__COVERAGE_SERVER_HPP_
#define NAV2_COVERAGE__COVERAGE_SERVER_HPP_

#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_coverage/headland_mode.hpp"
#include "nav2_coverage/swath_mode.hpp"
#include "nav2_coverage/route_mode.hpp"
#include "nav2_coverage/path_mode.hpp"

namespace nav2_coverage
{
/**
 * @class nav2_coverage::CoverageServer
 * @brief An action server which implements highly reconfigurable complete
 * coverage planning using the Fields2Cover library
 */
class CoverageServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_coverage::CoverageServer
   * @param options Additional options to control creation of the node.
   */
  explicit CoverageServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for nav2_coverage::CoverageServer
   */
  ~CoverageServer() = default;

  void test();

protected:
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

  std::unique_ptr<RobotMode> robot_;
  std::unique_ptr<HeadlandMode> headland_gen_;
  std::unique_ptr<SwathMode> swath_gen_;
  std::unique_ptr<RouteMode> route_gen_;
  std::unique_ptr<PathMode> path_gen_;
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__COVERAGE_SERVER_HPP_
