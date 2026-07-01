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

#include "opennav_coverage/swath_generator.hpp"

namespace opennav_coverage
{

SwathGenerator::ResolvedSwathParams SwathGenerator::resolveSwathParams(
  const opennav_coverage_msgs::msg::SwathMode & settings)
{
  SwathType action_type = toType(settings.objective);
  SwathAngleType action_angle_type = toAngleType(settings.mode);
  ResolvedSwathParams p;

  // If not set by action, use default mode
  if (action_type == SwathType::UNKNOWN && action_angle_type == SwathAngleType::UNKNOWN) {
    action_type = default_type_;
    p.angle_type = default_angle_type_;
    p.objective = default_objective_;
    p.swath_angle = default_swath_angle_;
    p.step_angle = default_step_angle_;
  } else {
    p.angle_type = action_angle_type;
    p.objective = createObjective(action_type);
    p.swath_angle = settings.best_angle;
    p.step_angle = settings.step_angle;
  }

  RCLCPP_DEBUG(
    logger_, "Generating Swaths with: %s", toString(action_type, p.angle_type).c_str());
  return p;
}

Swaths SwathGenerator::generateSwaths(
  const Field & field, const opennav_coverage_msgs::msg::SwathMode & settings)
{
  ResolvedSwathParams p = resolveSwathParams(settings);
  const double op_width = robot_params_->getOperationWidth();
  generator_->setAllowOverlap(default_allow_overlap_);
  switch (p.angle_type) {
    case SwathAngleType::BRUTE_FORCE:
      if (!p.objective) {
        throw CoverageException(
                "No valid swath mode set! Options: LENGTH, NUMBER, COVERAGE, NUMBER_MODIFIED.");
      }
      generator_->setStepAngle(p.step_angle);
      return generator_->generateBestSwaths(*p.objective, op_width, field);
    case SwathAngleType::SET_ANGLE:
      return generator_->generateSwaths(p.swath_angle, op_width, field);
    default:
      throw CoverageException("No valid swath angle mode set! Options: BRUTE_FORCE, SET_ANGLE.");
  }
}

Swaths SwathGenerator::generateSwaths(
  const F2CCells & cells, const opennav_coverage_msgs::msg::SwathMode & settings)
{
  // Single cell -> use the existing Field path (no flatten needed)
  if (cells.size() == 1) {
    return generateSwaths(cells.getGeometry(0), settings);
  }

  ResolvedSwathParams p = resolveSwathParams(settings);
  const double op_width = robot_params_->getOperationWidth();
  generator_->setAllowOverlap(default_allow_overlap_);
  switch (p.angle_type) {
    case SwathAngleType::BRUTE_FORCE:
      if (!p.objective) {
        throw CoverageException(
                "No valid swath mode set! Options: LENGTH, NUMBER, COVERAGE, NUMBER_MODIFIED.");
      }
      generator_->setStepAngle(p.step_angle);
      return generator_->generateBestSwaths(*p.objective, op_width, cells).flatten();
    case SwathAngleType::SET_ANGLE:
      return generator_->generateSwaths(p.swath_angle, op_width, cells).flatten();
    default:
      throw CoverageException("No valid swath angle mode set! Options: BRUTE_FORCE, SET_ANGLE.");
  }
}

F2CSwathsByCells SwathGenerator::generateSwathsByCells(
  const F2CCells & cells, const opennav_coverage_msgs::msg::SwathMode & settings)
{
  ResolvedSwathParams p = resolveSwathParams(settings);
  const double op_width = robot_params_->getOperationWidth();
  generator_->setAllowOverlap(default_allow_overlap_);
  switch (p.angle_type) {
    case SwathAngleType::BRUTE_FORCE:
      if (!p.objective) {
        throw CoverageException("No valid swath mode set! Options: LENGTH, NUMBER, COVERAGE.");
      }
      generator_->setStepAngle(p.step_angle);
      return generator_->generateBestSwaths(*p.objective, op_width, cells);
    case SwathAngleType::SET_ANGLE:
      return generator_->generateSwaths(p.swath_angle, op_width, cells);
    default:
      throw CoverageException("No valid swath angle mode set! Options: BRUTE_FORCE, SET_ANGLE.");
  }
}

void SwathGenerator::setSwathMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
  default_objective_ = createObjective(default_type_);
}

void SwathGenerator::setSwathAngleMode(const std::string & new_mode)
{
  default_angle_type_ = toAngleType(new_mode);
}

SwathObjectivePtr SwathGenerator::createObjective(const SwathType & type)
{
  switch (type) {
    case SwathType::LENGTH:
      return std::move(std::make_shared<f2c::obj::SwathLength>());
    case SwathType::NUMBER:
      return std::move(std::make_shared<f2c::obj::NSwath>());
    case SwathType::COVERAGE:
      return std::move(std::make_shared<f2c::obj::FieldCoverage>());
    case SwathType::NUMBER_MODIFIED:
      return std::move(std::make_shared<f2c::obj::NSwathModified>());
    default:
      RCLCPP_WARN(logger_, "Unknown swath type set!");
      return SwathObjectivePtr{nullptr};
  }
}

std::string SwathGenerator::toString(const SwathType & type, const SwathAngleType & angle_type)
{
  std::string str;
  switch (type) {
    case SwathType::LENGTH:
      str = "Length";
      break;
    case SwathType::NUMBER:
      str = "Number";
      break;
    case SwathType::COVERAGE:
      str = "Coverage";
      break;
    case SwathType::NUMBER_MODIFIED:
      str = "Number Modified";
      break;
    default:
      str = "Unknown";
      break;
  }

  str += " Objective and ";

  switch (angle_type) {
    case SwathAngleType::SET_ANGLE:
      str += "Set Angle";
      break;
    case SwathAngleType::BRUTE_FORCE:
      str += "Brute Force";
      break;
    default:
      str += "Unknown";
      break;
  }

  str += " angle.";
  return str;
}

SwathType SwathGenerator::toType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "LENGTH") {
    return SwathType::LENGTH;
  } else if (mode_str == "NUMBER") {
    return SwathType::NUMBER;
  } else if (mode_str == "COVERAGE") {
    return SwathType::COVERAGE;
  } else if (mode_str == "NUMBER_MODIFIED") {
    return SwathType::NUMBER_MODIFIED;
  } else {
    return SwathType::UNKNOWN;
  }
}

SwathAngleType SwathGenerator::toAngleType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "SET_ANGLE") {
    return SwathAngleType::SET_ANGLE;
  } else if (mode_str == "BRUTE_FORCE") {
    return SwathAngleType::BRUTE_FORCE;
  } else {
    return SwathAngleType::UNKNOWN;
  }
}


}  // namespace opennav_coverage
