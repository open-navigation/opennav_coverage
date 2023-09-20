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

#include "nav2_coverage/path_mode.hpp"

namespace nav2_coverage
{

Path PathMode::generatePath(const Swaths & swaths /*, (void) request*/)
{
  PathType action_type;  // = toType(request->path_type);
  PathContinuityType action_continuity_type;  // = toType(request->path_continuity_type);
  std::shared_ptr<f2c::pp::TurningBase> curve{nullptr};

  // If not set by action, use default mode
  if (action_type == PathType::UNKNOWN || action_continuity_type == PathContinuityType::UNKNOWN) {
    action_type = default_type_;
    action_continuity_type = default_continuity_type_;
    curve = default_curve_;
  } else {
    curve = createCurve(action_type, action_continuity_type);
  }

  if (!curve) {
    throw std::runtime_error("No valid path mode set!");
  }

  RCLCPP_INFO(
    logger_,
    "Generating path with curve: %s", toString(action_type, action_continuity_type).c_str());
  return generator_->searchBestPath(robot_->getRobot(), swaths, *curve);
}

void PathMode::setPathMode(const std::string & new_mode)
{
  std::string mode = new_mode;
  default_type_ = toType(mode);
  default_curve_ = createCurve(default_type_, default_continuity_type_);
}

void PathMode::setPathContinuityMode(const std::string & new_mode)
{
  std::string mode = new_mode;
  default_continuity_type_ = toContinuityType(mode);
  default_curve_ = createCurve(default_type_, default_continuity_type_);
}

TurningBasePtr PathMode::createCurve(const PathType & type, const PathContinuityType & c_type)
{
  switch (type) {
    case PathType::DUBIN:
      if (c_type == PathContinuityType::CONTINUOUS) {
        return std::move(std::make_shared<f2c::pp::DubinsCurvesCC>());
      } else if (c_type == PathContinuityType::DISCONTINUOUS) {
        return std::move(std::make_shared<f2c::pp::DubinsCurves>());
      }
      RCLCPP_WARN(logger_, "Unknown continuity type set! Options: CONTINUOUS, DISCONTINUOUS.");
      return TurningBasePtr{nullptr};
    case PathType::REEDS_SHEPP:
      if (c_type == PathContinuityType::CONTINUOUS) {
        return std::move(std::make_shared<f2c::pp::ReedsSheppCurvesHC>());
      } else if (c_type == PathContinuityType::DISCONTINUOUS) {
        return std::move(std::make_shared<f2c::pp::ReedsSheppCurves>());
      }
      RCLCPP_WARN(logger_, "Unknown continuity type set! Options: CONTINUOUS, DISCONTINUOUS.");
      return TurningBasePtr{nullptr};
    default:
      RCLCPP_WARN(logger_, "Unknown path type set! Options: DUBIN, REEDS_SHEPP.");
      return TurningBasePtr{nullptr};
  }
}

std::string PathMode::toString(const PathType & type, const PathContinuityType & c_type)
{
  std::string str;
  switch (type) {
    case PathType::DUBIN:
      str = "Dubin";
      break;
    case PathType::REEDS_SHEPP:
      str = "Reeds-Shepp";
      break;
    default:
      str = "Unknown";
      break;
  }

  str += " Mode and ";

  switch (c_type) {
    case PathContinuityType::CONTINUOUS:
      str = "Continuous";
      break;
    case PathContinuityType::DISCONTINUOUS:
      str = "Discontinuous";
      break;
    default:
      str = "Unknown";
      break;
  }

  str += " connections.";
  return str;
}

PathType PathMode::toType(std::string & str)
{
  toUpper(str);
  if (str == "REEDS_SHEPP") {
    return PathType::REEDS_SHEPP;
  } else if (str == "DUBIN") {
    return PathType::DUBIN;
  } else {
    return PathType::UNKNOWN;
  }
}

PathContinuityType PathMode::toContinuityType(std::string & str)
{
  toUpper(str);
  if (str == "CONTINUOUS") {
    return PathContinuityType::CONTINUOUS;
  } else if (str == "DISCONTINUOUS") {
    return PathContinuityType::DISCONTINUOUS;
  } else {
    return PathContinuityType::UNKNOWN;
  }
}

}  // namespace nav2_coverage
