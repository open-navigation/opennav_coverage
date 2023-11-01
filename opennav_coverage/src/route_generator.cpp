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

#include "opennav_coverage/route_generator.hpp"

namespace opennav_coverage
{

Swaths RouteGenerator::generateRoute(
  const Swaths & swaths, const opennav_coverage_msgs::msg::RouteMode & settings)
{
  RouteType action_type = toType(settings.mode);
  std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> generator{nullptr};
  size_t spiral_n;
  std::vector<size_t> custom_order;

  // If not set by action, use default mode
  if (action_type == RouteType::UNKNOWN) {
    action_type = default_type_;
    generator = default_generator_;
    spiral_n = default_spiral_n_;
    custom_order = default_custom_order_;
  } else {
    generator = createGenerator(action_type);
    spiral_n = settings.spiral_n;
    custom_order = std::vector<size_t>(settings.custom_order.begin(), settings.custom_order.end());
  }

  if (!generator) {
    throw CoverageException(
            "No valid route mode set! Options: BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM.");
  } else if (action_type == RouteType::SPIRAL) {
    dynamic_cast<f2c::rp::SpiralOrder *>(generator.get())->setSpiralSize(spiral_n);
  } else if (action_type == RouteType::CUSTOM) {
    dynamic_cast<f2c::rp::CustomOrder *>(generator.get())->setCustomOrder(custom_order);
  }

  RCLCPP_DEBUG(logger_, "Generating route with generator: %s", toString(action_type).c_str());
  return generator->genSortedSwaths(swaths);
}

void RouteGenerator::setMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
  default_generator_ = createGenerator(default_type_);
}

RouteGeneratorPtr RouteGenerator::createGenerator(const RouteType & type)
{
  switch (type) {
    case RouteType::BOUSTROPHEDON:
      return std::move(std::make_shared<f2c::rp::BoustrophedonOrder>());
    case RouteType::SNAKE:
      return std::move(std::make_shared<f2c::rp::SnakeOrder>());
    case RouteType::SPIRAL:
      return std::move(std::make_shared<f2c::rp::SpiralOrder>());
    case RouteType::CUSTOM:
      return std::move(std::make_shared<f2c::rp::CustomOrder>());
    default:
      RCLCPP_WARN(logger_, "Unknown route type set!");
      return RouteGeneratorPtr{nullptr};
  }
}

std::string RouteGenerator::toString(const RouteType & type)
{
  switch (type) {
    case RouteType::BOUSTROPHEDON:
      return "Boustrophedon";
    case RouteType::SNAKE:
      return "Snake";
    case RouteType::SPIRAL:
      return "Spiral";
    case RouteType::CUSTOM:
      return "Custom";
    default:
      return "Unknown";
  }
}

RouteType RouteGenerator::toType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "BOUSTROPHEDON") {
    return RouteType::BOUSTROPHEDON;
  } else if (mode_str == "SNAKE") {
    return RouteType::SNAKE;
  } else if (mode_str == "SPIRAL") {
    return RouteType::SPIRAL;
  } else if (mode_str == "CUSTOM") {
    return RouteType::CUSTOM;
  } else {
    return RouteType::UNKNOWN;
  }
}

}  // namespace opennav_coverage
