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

#ifndef OPENNAV_COVERAGE_BT__CANCEL_COMPLETE_COVERAGE_PATH_HPP_
#define OPENNAV_COVERAGE_BT__CANCEL_COMPLETE_COVERAGE_PATH_HPP_

#include <memory>
#include <string>

#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "nav2_behavior_tree/bt_cancel_action_node.hpp"
#include "opennav_coverage_bt/utils.hpp"

namespace opennav_coverage_bt
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps
 * opennav_coverage_msgs::action::ComputeCoveragePath
 */
class CoverageCancel
  : public nav2_behavior_tree::BtCancelActionNode<
    opennav_coverage_msgs::action::ComputeCoveragePath>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CoverageCancel
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  CoverageCancel(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
      });
  }
};

}  // namespace opennav_coverage_bt

#endif  // OPENNAV_COVERAGE_BT__CANCEL_COMPLETE_COVERAGE_PATH_HPP_
