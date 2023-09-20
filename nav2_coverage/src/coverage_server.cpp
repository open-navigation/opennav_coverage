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

#include "nav2_coverage/coverage_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_coverage
{

CoverageServer::CoverageServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("coverage_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());
}

nav2_util::CallbackReturn
CoverageServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  robot_ = std::make_unique<RobotParams>(node);
  headland_gen_ = std::make_unique<HeadlandGenerator>(node);
  swath_gen_ = std::make_unique<SwathGenerator>(node, robot_.get());
  route_gen_ = std::make_unique<RouteGenerator>(node);
  path_gen_ = std::make_unique<PathGenerator>(node, robot_.get());
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());
  auto node = shared_from_this();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&CoverageServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  path_gen_.reset();
  route_gen_.reset();
  swath_gen_.reset();
  headland_gen_.reset();
  robot_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

void CoverageServer::test()
{
  Fields fields;  // get a field // Ring object. File / request field.

  // Uses request for type / width where possible
  Field remaining_field = headland_gen_->generateHeadlands(fields);  //, request);
  Swaths swaths = swath_gen_->generateSwaths(remaining_field);  //, request);
  Swaths route = route_gen_->generateRoute(swaths);  //, request);
  Path path = path_gen_->generatePath(route);   //, request);
}

rcl_interfaces::msg::SetParametersResult
CoverageServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    // const auto & type = parameter.get_type();
    // const auto & name = parameter.get_name();

    // TODO(sm) dynamic params including internal default params
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_coverage

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_coverage::CoverageServer)
