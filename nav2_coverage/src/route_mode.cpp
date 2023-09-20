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

#include "nav2_coverage/route_mode.hpp"

namespace nav2_coverage
{

Swaths RouteMode::generateRoute(const Swaths & swaths /*, (void) request*/)
{
  RouteType action_type;  // = toType(request->swaths_type);
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
    // spiral_n = request->spiral_n;
    // custom_order = request->custom_order;
  }

  if (!generator) {
    throw std::runtime_error(
            "No valid route mode set! Options: BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM.");
  } else if (action_type == RouteType::SPIRAL) {
    dynamic_cast<f2c::rp::SpiralOrder *>(generator.get())->setSpiralSize(spiral_n);
  } else if (action_type == RouteType::CUSTOM) {
    dynamic_cast<f2c::rp::CustomOrder *>(generator.get())->setCustomOrder(custom_order);
  }

  RCLCPP_INFO(logger_, "Generating route with generator: %s", toString(action_type).c_str());
  return generator->genSortedSwaths(swaths);
}

void RouteMode::setMode(const std::string & new_mode)
{
  std::string mode = new_mode;
  default_type_ = toType(mode);
  default_generator_ = createGenerator(default_type_);
}

RouteGeneratorPtr RouteMode::createGenerator(const RouteType & type)
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

std::string RouteMode::toString(const RouteType & type)
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

RouteType RouteMode::toType(std::string & str)
{
  toUpper(str);
  if (str == "BOUSTROPHEDON") {
    return RouteType::BOUSTROPHEDON;
  } else if (str == "SNAKE") {
    return RouteType::SNAKE;
  } else if (str == "SPIRAL") {
    return RouteType::SPIRAL;
  } else if (str == "CUSTOM") {
    return RouteType::CUSTOM;
  } else {
    return RouteType::UNKNOWN;
  }
}

}  // namespace nav2_coverage
