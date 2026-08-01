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

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

#include "opennav_coverage/path_generator.hpp"

namespace opennav_coverage
{

namespace
{

// Perpendicular distance from p to the segment [a, b].
double distanceToSegment(const Point & p, const Point & a, const Point & b)
{
  const double dx = b.getX() - a.getX();
  const double dy = b.getY() - a.getY();
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12) {
    return p.distance(a);
  }
  const double t = std::clamp(
    ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / len2, 0.0, 1.0);
  return std::hypot(p.getX() - (a.getX() + t * dx), p.getY() - (a.getY() + t * dy));
}

}  // namespace

Path PathGenerator::generatePath(
  const F2CRoute & route, const opennav_coverage_msgs::msg::PathMode & settings)
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
  curve->setDiscretization(turn_point_distance);
  Path path = assemblePath(route, *curve);

  // Optionally thin out near-duplicate points (e.g. in turns)
  if (reduce_path_) {
    path.reduce(reduce_min_dist_);
  }
  return path;
}

Path PathGenerator::assemblePath(const F2CRoute & route, f2c::pp::TurningBase & curve)
{
  auto & robot = robot_params_->getRobot();
  Path path;
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    const Swaths prev = (i > 0) ? route.getSwaths(i - 1) : Swaths();
    const F2CMultiPoint connection =
      (i < route.sizeConnections()) ? route.getConnection(i) : F2CMultiPoint();
    appendConnection(path, prev, connection, route.getSwaths(i), curve);
    path += generator_->planPath(robot, route.getSwaths(i), curve);
  }
  if (route.sizeConnections() > route.sizeVectorSwaths()) {
    appendConnection(
      path, route.getLastSwaths(), route.getLastConnection(), Swaths(), curve);
  }
  return path;
}

void PathGenerator::appendConnection(
  Path & path, const Swaths & prev, const F2CMultiPoint & connection,
  const Swaths & next, f2c::pp::TurningBase & curve)
{
  auto & robot = robot_params_->getRobot();
  const bool has_prev = prev.size() > 0;
  const bool has_next = next.size() > 0;
  if (!has_prev && !has_next && connection.size() < 2) {
    return;
  }

  std::vector<Point> pts;
  if (has_prev) {
    pts.push_back(prev.back().endPoint());
  }
  for (size_t i = 0; i < connection.size(); ++i) {
    pts.push_back(connection[i]);
  }
  if (has_next) {
    pts.push_back(next[0].startPoint());
  }
  if (pts.size() < 2) {
    return;
  }

  // Drop attachment spikes: a bridge point past the endpoint that doubles back.
  bool changed = true;
  while (changed && pts.size() > 2) {
    changed = false;
    for (const size_t i : {size_t{1}, pts.size() - 2}) {
      const Point & a = pts[i - 1];
      const Point & b = pts[i];
      const Point & c = pts[i + 1];
      const double dot = (b.getX() - a.getX()) * (c.getX() - b.getX()) +
        (b.getY() - a.getY()) * (c.getY() - b.getY());
      if (dot < 0.0 && a.distance(b) < a.distance(c)) {
        pts.erase(pts.begin() + i);
        changed = true;
        break;
      }
    }
  }

  // Deviation, not length ratio: rounding a long cell's corner is only ~20%
  // longer than cutting its diagonal, yet shares no ground with it.
  double max_dev = 0.0;
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    max_dev = std::max(max_dev, distanceToSegment(pts[i], pts.front(), pts.back()));
  }

  // Straight hop: the ordinary headland u-turn, leave it to the curve planner.
  // A real bend means the graph routed around something worth keeping.
  if (has_prev && has_next && max_dev < 0.5 * robot_params_->getOperationWidth()) {
    path += curve.createTurn(
      robot, prev.back().endPoint(), prev.back().getOutAngle(),
      next[0].startPoint(), next[0].getInAngle());
    return;
  }

  // The polyline detours (e.g. around a concave notch): follow it verbatim as
  // HL_SWATH states so the vehicle stays on the planned border/corridor track.
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double dx = pts[i + 1].getX() - pts[i].getX();
    const double dy = pts[i + 1].getY() - pts[i].getY();
    const double len = std::hypot(dx, dy);
    if (len < 1e-6) {
      continue;
    }
    PathState s;
    s.point = pts[i];
    s.angle = std::atan2(dy, dx);
    s.len = len;
    s.dir = f2c::types::PathDirection::FORWARD;
    s.type = f2c::types::PathSectionType::HL_SWATH;
    s.velocity = robot.getCruiseVel();
    path.addState(s);
  }
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
