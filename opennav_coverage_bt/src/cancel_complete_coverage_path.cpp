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

#include <string>
#include <memory>

#include "opennav_coverage_bt/cancel_complete_coverage_path.hpp"

namespace opennav_coverage_bt
{

CoverageCancel::CoverageCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<opennav_coverage_msgs::action::ComputeCoveragePath>(
    xml_tag_name, action_name, conf)
{
}

}  // namespace opennav_coverage_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_coverage_bt::CoverageCancel>(
        name, "compute_coverage_path", config);
    };

  factory.registerBuilder<opennav_coverage_bt::CoverageCancel>(
    "CancelCoverage", builder);
}
