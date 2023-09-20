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

#include <vector>
#include <string>

#include "nav2_coverage/headland_generator.hpp"

namespace nav2_coverage
{

Field HeadlandGenerator::generateHeadlands(const Fields & field /*, (void) request*/)
{
  HeadlandType action_type;  // = toType(request->headland_type);
  HeadlandGeneratorPtr generator{nullptr};
  float width = 0.0f;

  // If not set by action, use default mode
  if (action_type == HeadlandType::UNKNOWN) {
    action_type = default_type_;
    generator = default_generator_;
    width = default_headland_width_;
  } else {
    generator = createGenerator(action_type);
    // width = request->headland_width;
  }

  if (!generator) {
    throw std::runtime_error("No valid headlands mode set! Options: CONSTANT.");
  }

  RCLCPP_INFO(logger_, "Generating Headland with generator: %s", toString(action_type).c_str());
  return generator->generateHeadlands(field, width).getGeometry(0);
}


void HeadlandGenerator::setMode(const std::string & new_mode)
{
  std::string mode = new_mode;
  default_type_ = toType(mode);
  default_generator_ = createGenerator(default_type_);
}

HeadlandGeneratorPtr HeadlandGenerator::createGenerator(const HeadlandType & type)
{
  switch (type) {
    case HeadlandType::CONSTANT:
      return std::move(std::make_shared<f2c::hg::ConstHL>());
    default:
      RCLCPP_WARN(logger_, "Unknown headland type set!");
      return HeadlandGeneratorPtr{nullptr};
  }
}

std::string HeadlandGenerator::toString(const HeadlandType & type)
{
  switch (type) {
    case HeadlandType::CONSTANT:
      return "Constant";
    default:
      return "Unknown";
  }
}

HeadlandType HeadlandGenerator::toType(std::string & str)
{
  toUpper(str);
  if (str == "CONSTANT") {
    return HeadlandType::CONSTANT;
  } else {
    return HeadlandType::UNKNOWN;
  }
}

}  // namespace nav2_coverage
