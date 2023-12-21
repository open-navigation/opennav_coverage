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

#ifndef OPENNAV_COVERAGE_BT__COMPUTE_COMPLETE_COVERAGE_PATH_HPP_
#define OPENNAV_COVERAGE_BT__COMPUTE_COMPLETE_COVERAGE_PATH_HPP_

#include <string>
#include <memory>
#include <vector>

#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "opennav_coverage_bt/utils.hpp"

namespace opennav_coverage_bt
{

/**
 * @brief nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputeCoveragePath
 */
class ComputeCoveragePathAction
  : public nav2_behavior_tree::BtActionNode<
    opennav_coverage_msgs::action::ComputeCoveragePath>
{
  using Action = opennav_coverage_msgs::action::ComputeCoveragePath;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for opennav_coverage_bt::ComputeCoveragePathAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputeCoveragePathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<bool>("generate_headland", true, "Whether to generate headland"),
        BT::InputPort<bool>("generate_route", true, "Whether to ordered route of swaths"),
        BT::InputPort<bool>("generate_path", true, "Whether to generate connected path of routes"),

        BT::InputPort<std::string>("file_field", "Filepath to field GML file"),
        BT::InputPort<int>("file_field_id", 0, "Ordered ID of which field to use in GML File"),
        BT::InputPort<std::vector<geometry_msgs::msg::Polygon>>(
          "polygons", "Port-provided polygon, if not from file"),
        BT::InputPort<std::string>("polygons_frame_id", "map", "Port-provided polygon's frame"),

        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The complete coverage error code"),
        BT::OutputPort<ActionResult::_planning_time_type>(
          "planning_time", "The time to compute coverage plan"),
        BT::OutputPort<nav_msgs::msg::Path>(
          "nav_path", "The coverage plan as a nav_msgs/Path to track directly"),
        BT::OutputPort<ActionResult::_coverage_path_type>(
          "coverage_path", "The coverage plan as an ordered set of swaths and route connections"),
      });
  }
};

}  // namespace opennav_coverage_bt

#endif  // OPENNAV_COVERAGE_BT__COMPUTE_COMPLETE_COVERAGE_PATH_HPP_
