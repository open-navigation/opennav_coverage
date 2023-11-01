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

#include "opennav_coverage/path_generator.hpp"

namespace opennav_coverage
{

Path PathGenerator::generatePath(
  const Swaths & swaths, const opennav_coverage_msgs::msg::PathMode & settings)
{
  PathType action_type = toType(settings.mode);
  PathContinuityType action_continuity_type = toContinuityType(settings.continuity_mode);
  std::shared_ptr<f2c::pp::TurningBase> curve{nullptr};
  float turn_point_distance;

  // If not set by action, use default mode
  if (action_type == PathType::UNKNOWN || action_continuity_type == PathContinuityType::UNKNOWN) {
    action_type = default_type_;
    action_continuity_type = default_continuity_type_;
    curve = default_curve_;
    turn_point_distance = default_turn_point_distance_;
  } else {
    curve = createCurve(action_type, action_continuity_type);
    turn_point_distance = settings.turn_point_distance;
  }

  if (!curve) {
    throw CoverageException("No valid path mode set!");
  }

  RCLCPP_DEBUG(
    logger_,
    "Generating path with curve: %s", toString(action_type, action_continuity_type).c_str());
  generator_->turn_point_dist = turn_point_distance;
  return generator_->searchBestPath(robot_params_->getRobot(), swaths, *curve);
}

void PathGenerator::setPathMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
  default_curve_ = createCurve(default_type_, default_continuity_type_);
}

void PathGenerator::setPathContinuityMode(const std::string & new_mode)
{
  default_continuity_type_ = toContinuityType(new_mode);
  default_curve_ = createCurve(default_type_, default_continuity_type_);
}

TurningBasePtr PathGenerator::createCurve(const PathType & type, const PathContinuityType & c_type)
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

std::string PathGenerator::toString(const PathType & type, const PathContinuityType & c_type)
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
      str += "Continuous";
      break;
    case PathContinuityType::DISCONTINUOUS:
      str += "Discontinuous";
      break;
    default:
      str += "Unknown";
      break;
  }

  str += " connections.";
  return str;
}

PathType PathGenerator::toType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "REEDS_SHEPP") {
    return PathType::REEDS_SHEPP;
  } else if (mode_str == "DUBIN") {
    return PathType::DUBIN;
  } else {
    return PathType::UNKNOWN;
  }
}

PathContinuityType PathGenerator::toContinuityType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "CONTINUOUS") {
    return PathContinuityType::CONTINUOUS;
  } else if (mode_str == "DISCONTINUOUS") {
    return PathContinuityType::DISCONTINUOUS;
  } else {
    return PathContinuityType::UNKNOWN;
  }
}

}  // namespace opennav_coverage
