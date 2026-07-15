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
#include <string>

#include "opennav_coverage/decomp_generator.hpp"

namespace opennav_coverage
{

F2CCells DecompGenerator::decompose(
  const F2CCells & cells,
  const opennav_coverage_msgs::msg::DecompMode & settings)
{
  // "UNKNOWN" -> use default_type_ (same pattern as Headland/Swath generators)
  DecompType type = toType(settings.mode);
  if (type == DecompType::UNKNOWN) {
    type = default_type_;
  }

  double split_angle =
    std::abs(settings.split_angle) < 1e-9 ?
    default_split_angle_ : settings.split_angle;

  if (type == DecompType::NONE) {
    return cells;  // no-op
  }

  RCLCPP_DEBUG(
    logger_, "Decomposing field with type %s, split_angle=%.3f",
    toString(type).c_str(), split_angle);

  if (type == DecompType::TRAPEZOIDAL) {
    f2c::decomp::TrapezoidalDecomp decomp;
    decomp.setSplitAngle(split_angle);
    return decomp.decompose(cells);
  } else if (type == DecompType::BOUSTROPHEDON) {
    f2c::decomp::BoustrophedonDecomp decomp;
    decomp.setSplitAngle(split_angle);
    return decomp.decompose(cells);
  }

  throw CoverageException("Unknown decomp type requested!");
}

std::string DecompGenerator::toString(const DecompType & type)
{
  switch (type) {
    case DecompType::NONE: return "NONE";
    case DecompType::TRAPEZOIDAL: return "TRAPEZOIDAL";
    case DecompType::BOUSTROPHEDON: return "BOUSTROPHEDON";
    default: return "UNKNOWN";
  }
}

DecompType DecompGenerator::toType(const std::string & str)
{
  std::string upper = str;
  util::toUpper(upper);
  if (upper == "NONE") {
    return DecompType::NONE;
  }
  if (upper == "TRAPEZOIDAL") {
    return DecompType::TRAPEZOIDAL;
  }
  if (upper == "BOUSTROPHEDON") {
    return DecompType::BOUSTROPHEDON;
  }
  return DecompType::UNKNOWN;  // left to caller (falls back to default)
}

void DecompGenerator::setMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
}

}  // namespace opennav_coverage
