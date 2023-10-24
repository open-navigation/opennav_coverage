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

  robot_params_ = std::make_unique<RobotParams>(node);
  headland_gen_ = std::make_unique<HeadlandGenerator>(node);
  swath_gen_ = std::make_unique<SwathGenerator>(node, robot_params_.get());
  route_gen_ = std::make_unique<RouteGenerator>(node);
  path_gen_ = std::make_unique<PathGenerator>(node, robot_params_.get());
  visualizer_ = std::make_unique<Visualizer>();

  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action servers for path planning to a pose and through poses
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "compute_coverage_path",
    std::bind(&CoverageServer::computeCoveragePath, this),
    nullptr,
    std::chrono::milliseconds(500),
    true, server_options);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());
  auto node = shared_from_this();
  action_server_->activate();
  visualizer_->activate(node);

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
  action_server_->deactivate();
  visualizer_->deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  action_server_.reset();
  visualizer_.reset();
  path_gen_.reset();
  route_gen_.reset();
  swath_gen_.reset();
  headland_gen_.reset();
  robot_params_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}


void CoverageServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ComputeCoveragePath::Goal> goal)
{
  if (action_server_->is_preempt_requested()) {
    goal = action_server_->accept_pending_goal();
  }
}

bool CoverageServer::validateGoal(
  typename std::shared_ptr<const typename ComputeCoveragePath::Goal> req)
{
  getPreemptedGoalIfRequested(req);
  if (req->generate_path && !req->generate_route) {
    return false;
  }
  return true;
}

void CoverageServer::computeCoveragePath()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  auto start_time = this->now();

  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<ComputeCoveragePath::Result>();

  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (action_server_->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling coverage planning action.");
    action_server_->terminate_all();
    return;
  }

  if (!validateGoal(goal)) {
    RCLCPP_WARN(get_logger(), "Goal contained invalid configurations!");
    result->error_code = ComputeCoveragePath::Result::INVALID_REQUEST;
    action_server_->terminate_current(result);
  }

  try {
    // (0) Obtain field to use
    Field field;
    std::string frame_id;
    if (goal->use_gml_file) {
      F2CFields parse_field;
      f2c::Parser::importGml(goal->gml_field, parse_field);
      // TODO(SM): replace w/ UTM conversion (and back?) transformSwaths, transformPath
      // Path rtn_path = f2c::Transform::transformPath(path, field, "EPSG:4258");
      f2c::Transform::transform(parse_field[0], "EPSG:28992");
      field = parse_field[goal->gml_field_id].field.getGeometry(0);
      frame_id = parse_field[goal->gml_field_id].coord_sys;
    } else {
      field = util::getFieldFromGoal(goal);
      frame_id = goal->frame_id;
    }

    RCLCPP_INFO(
      get_logger(),
      "Generating coverage path in %s frame for zone with %zu outer nodes and %zu inner polygons.",
      frame_id.c_str(), field.getGeometry(0).size(), field.size() - 1);

    // (1) Optional: Remove headland from polygon field
    Field field_no_headland = field;
    if (goal->generate_headland) {
      field_no_headland = headland_gen_->generateHeadlands(field, goal->headland_mode);
    }

    // (2) Generate swaths to cover polygon field, including internal voids
    Swaths swaths = swath_gen_->generateSwaths(field_no_headland, goal->swath_mode);

    // (3) Optional: Generate an ordered route through the unordered swaths
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id;
    if (goal->generate_route) {
      Swaths route = route_gen_->generateRoute(swaths, goal->route_mode);

      // (4) Optional: Generate connection turns between ordered swaths
      if (goal->generate_path) {
        Path path = path_gen_->generatePath(route, goal->path_mode);
        result->coverage_path = util::toCoveragePathMsg(path, header);
        result->nav_path = util::toNavPathMsg(path, header);
      } else {
        result->coverage_path = util::toCoveragePathMsg(route, true, header);
      }
    } else {
      result->coverage_path = util::toCoveragePathMsg(swaths, false, header);
    }

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;
    visualizer_->visualize(field, field_no_headland, result, header);
    action_server_->succeeded_current(result);
  } catch (CoverageException & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = ComputeCoveragePath::Result::INVALID_MODE_SET;
  } catch (std::invalid_argument & e) {
    RCLCPP_ERROR(get_logger(), "Invalid GML File or Coordinates: %s", e.what());
    result->error_code = ComputeCoveragePath::Result::INVALID_COORDS;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal Fields2Cover error: %s", e.what());
    result->error_code = ComputeCoveragePath::Result::INTERNAL_F2C_ERROR;
  }

  action_server_->terminate_current(result);
}

rcl_interfaces::msg::SetParametersResult
CoverageServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "default_headland_width") {
        headland_gen_->setWidth(parameter.as_double());
      } else if (name == "default_swath_angle") {
        swath_gen_->setSwathAngle(parameter.as_double());
      } else if (name == "default_step_angle") {
        swath_gen_->setStepAngle(parameter.as_double());
      } else if (name == "default_turn_point_distance") {
        path_gen_->setTurnPointDistance(parameter.as_double());
      } else if (name == "robot_width") {
        auto & robot = robot_params_->getRobot();
        robot.robot_width = parameter.as_double();
      } else if (name == "operation_width") {
        auto & robot = robot_params_->getRobot();
        robot.op_width = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "default_headland_type") {
        headland_gen_->setMode(parameter.as_string());
      } else if (name == "default_path_type") {
        path_gen_->setPathMode(parameter.as_string());
      } else if (name == "default_path_continuity_type") {
        path_gen_->setPathContinuityMode(parameter.as_string());
      } else if (name == "default_route_type") {
        route_gen_->setMode(parameter.as_string());
      } else if (name == "default_swath_type") {
        swath_gen_->setSwathMode(parameter.as_string());
      } else if (name == "default_swath_angle_type") {
        swath_gen_->setSwathAngleMode(parameter.as_string());
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "default_allow_overlap") {
        swath_gen_->setOVerlap(parameter.as_bool());
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "default_spiral_n") {
        route_gen_->setSpiralN(parameter.as_int());
      }
    } else if (type == ParameterType::PARAMETER_INTEGER_ARRAY) {
      if (name == "default_custom_order") {
        route_gen_->setCustomOrder(parameter.as_integer_array());
      }
    }
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
