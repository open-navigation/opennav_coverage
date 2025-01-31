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

#include <memory>
#include <string>

#include "opennav_coverage_bt/compute_complete_coverage_path.hpp"

namespace opennav_coverage_bt
{

ComputeCoveragePathAction::ComputeCoveragePathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void ComputeCoveragePathAction::on_tick()
{
  // Get core inputs about what to perform
  getInput("generate_headland", goal_.generate_headland);
  getInput("generate_route", goal_.generate_route);
  getInput("generate_path", goal_.generate_path);

  // Get the field to get coverage for
  std::string gml_filename;
  if (getInput("file_field", gml_filename)) {
    goal_.gml_field = gml_filename;
    goal_.use_gml_file = true;
  } else {
    getInput("polygons_frame_id", goal_.frame_id);

    // Convert from vector of Polygons to coverage sp. message
    std::vector<geometry_msgs::msg::Polygon> polys;
    getInput("polygons", polys);
    goal_.polygons.clear();
    goal_.polygons.resize(polys.size());
    for (unsigned int i = 0; i != polys.size(); i++) {
      for (unsigned int j = 0; j != polys[i].points.size(); j++) {
        opennav_coverage_msgs::msg::Coordinate coord;
        coord.axis1 = polys[i].points[j].x;
        coord.axis2 = polys[i].points[j].y;
        goal_.polygons[i].coordinates.push_back(coord);
      }
    }
  }
}

BT::NodeStatus ComputeCoveragePathAction::on_success()
{
  setOutput("planning_time", result_.result->planning_time);
  setOutput("nav_path", result_.result->nav_path);
  setOutput("coverage_path", result_.result->coverage_path);
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeCoveragePathAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeCoveragePathAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeCoveragePathAction::halt()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  BtActionNode::halt();
}

}  // namespace opennav_coverage_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_coverage_bt::ComputeCoveragePathAction>(
        name, "compute_coverage_path", config);
    };

  factory.registerBuilder<opennav_coverage_bt::ComputeCoveragePathAction>(
    "ComputeCoveragePath", builder);
}
