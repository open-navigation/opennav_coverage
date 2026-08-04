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
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>
#include <string>

#include "opennav_coverage/path_generator.hpp"

namespace opennav_coverage
{

namespace
{

// Fractions of the operation width.
constexpr double kDirectHopDev = 0.25;  // under this a connection is a plain u-turn
constexpr double kSimplifyTol = 0.1;    // points this close to the track are dropped

constexpr double kLegShare = 0.5;       // a leg is split evenly between its two corners
constexpr double kMinSweep = 0.05;      // corners under ~3deg are driven straight through
constexpr double kRadiusMargin = 1.2;   // room for the clothoid lead-in of a CC turn
constexpr double kMinBackoffRadii = 0.5;  // shallow corners still get a workable approach
constexpr double kLoopSlackRadii = 2.0;   // over this the planner answered with a loop
constexpr double kReversalSweep = 2.0;    // legs this far apart in heading double back
constexpr double kUturnHopWidths = 3.0;   // a u-turn between neighbours spans about one
constexpr double kUturnReachRadii = 4.0;  // how far back a u-turn may start, in radii
constexpr size_t kMaxSpan = 3;            // a spur, the corner it leads to, and a spur out

// Legs run along the border between swaths, so they are headland passes at
// cruise speed rather than in-place turns.
void addStraight(Path & path, Robot & robot, const Point & a, const Point & b)
{
  const double len = a.distance(b);
  if (len < 1e-6) {
    return;
  }
  PathState s;
  s.point = a;
  s.angle = std::atan2(b.getY() - a.getY(), b.getX() - a.getX());
  s.len = len;
  s.dir = f2c::types::PathDirection::FORWARD;
  s.type = f2c::types::PathSectionType::HL_SWATH;
  s.velocity = robot.getCruiseVel();
  path.addState(s);
}

// Smallest radius a turn of `sweep` radians can be driven at. Each curvature
// ramp of a continuous turn consumes curvature^2/(2*rate) of the deflection, so
// below this the planner returns a curve that jumps to full curvature.
double turnRadius(const Robot & robot, double sweep, bool continuous)
{
  const double min_radius = robot.getMinTurningRadius();
  const double rate = robot.getMaxDiffCurv();
  if (!continuous || rate <= 0.0 || sweep <= 0.0) {
    return min_radius;
  }
  return std::max(min_radius, 1.0 / std::sqrt(rate * sweep));
}

// Follow `poly`: straight runs as they are, corners through the turn planner.
// `start_angle`/`end_angle` are the headings the swaths either side hold, unset
// where there is no swath. They stand in for the track's own direction, which
// the robot does not hold there, making each end a corner in its own right.
// Returns the corners left sharp because no maneuver fit within `cut_tol`.
size_t appendRoundedTrack(
  Path & path, const std::vector<Point> & poly, Robot & robot,
  f2c::pp::TurningBase & curve, double op_width, double cut_tol, bool continuous,
  const std::optional<double> & start_angle, const std::optional<double> & end_angle,
  const rclcpp::Logger & logger, std::vector<Path> * turns)
{
  const double radius = robot.getMinTurningRadius();
  const size_t n = poly.size();
  const auto legAngle = [&poly](size_t a, size_t b) {
      return std::atan2(poly[b].getY() - poly[a].getY(), poly[b].getX() - poly[a].getX());
    };

  // Deflection at each corner, which is what sizes the maneuver through it.
  std::vector<double> sweeps(n, 0.0);
  for (size_t k = 1; k + 1 < n; ++k) {
    sweeps[k] = util::sweepBetween(legAngle(k - 1, k), legAngle(k, k + 1));
  }
  if (start_angle) {
    sweeps.front() = util::sweepBetween(*start_angle, legAngle(0, 1));
  }
  if (end_angle) {
    sweeps.back() = util::sweepBetween(legAngle(n - 2, n - 1), *end_angle);
  }

  size_t sharp = 0;
  Point cursor = poly.front();
  size_t i = 0;
  while (i < n) {
    if (sweeps[i] < kMinSweep) {
      addStraight(path, robot, cursor, poly[i]);
      cursor = poly[i];
      ++i;
      continue;
    }

    // Corners closer together than a maneuver needs on either side cannot be
    // taken one at a time: rounding the first leaves the next no approach. Fold
    // those into a single span.
    size_t min_span = 1;
    while (i + min_span < n && min_span < kMaxSpan &&
      sweeps[i + min_span] >= kMinSweep &&
      poly[i + min_span - 1].distance(poly[i + min_span]) < 2.0 * kRadiusMargin * radius)
    {
      ++min_span;
    }

    bool rounded = false;
    // Widening out from there, because a shorter span cuts less corner.
    for (size_t span = min_span; span <= kMaxSpan && i + span <= n && !rounded; ++span) {
      const size_t j = i + span - 1;               // last corner taken in one go
      // At either end the pose is the swath's, fixed there rather than slid
      // along a leg.
      const bool pin_in = (i == 0);
      const bool pin_out = (j + 1 == n);
      if (pin_out && !end_angle) {
        break;
      }
      const Point & first = poly[i];
      const Point & last_corner = poly[j];
      const Point & after = pin_out ? poly[j] : poly[j + 1];

      // Total turning sizes the maneuver's length; net deflection sizes its
      // geometry. They differ on an S, where the two corners cancel: summing
      // magnitudes would read that as a reversal and refuse to plan it.
      double total = 0.0;
      for (size_t k = i; k <= j; ++k) {
        total += sweeps[k];
      }
      const double in_angle = pin_in ? *start_angle : std::atan2(
        first.getY() - cursor.getY(), first.getX() - cursor.getX());
      const double out_angle = pin_out ? *end_angle : std::atan2(
        after.getY() - last_corner.getY(), after.getX() - last_corner.getX());
      const double net = util::sweepBetween(in_angle, out_angle);

      const bool tail = (j + 2 == n) && sweeps[n - 1] < kMinSweep;
      const double back_room = pin_in ? 0.0 : cursor.distance(first);
      const double fwd_room = pin_out ? 0.0 :
        (tail ? 1.0 : kLegShare) * last_corner.distance(after);
      const double room = pin_in ? fwd_room : (pin_out ? back_room :
        std::min(back_room, fwd_room));

      // Turning back on itself over a short hop is a u-turn, not a corner: an
      // arc has no tangent length at 180 degrees, so it must swing past the
      // vertices into the headland. Over a long hop the reversal is a detour.
      const double hop = first.distance(last_corner);
      const bool uturn = net > kReversalSweep && hop < kUturnHopWidths * op_width;

      // Offset of the outgoing leg from the line the robot arrives on. Measured
      // off the entry heading, which the approach segment cannot give when the
      // maneuver starts at the swath.
      const double sep = std::fabs(
        std::cos(in_angle) * (after.getY() - first.getY()) -
        std::sin(in_angle) * (after.getX() - first.getX()));

      // Legs closer than a turning diameter cannot be joined by a half circle,
      // so the maneuver has to loop. That is the connection's geometry.
      if (uturn) {
        const double need = 2.0 * turnRadius(robot, net, continuous);
        if (sep < need) {
          RCLCPP_DEBUG(
            logger,
            "corner %zu span=%zu v=(%.2f,%.2f) u-turn legs %.2fm apart, %.2fm needed to "
            "come round: the turn has to loop",
            i, span, first.getX(), first.getY(), sep, need);
        }
      }

      // One line per attempt, so a corner that stays sharp can be read against
      // its neighbours.
      const auto report =
        [&](const char * outcome, double back_off, double value, double limit) {
          RCLCPP_DEBUG(
            logger,
            "corner %zu span=%zu prev=(%.2f,%.2f) v=(%.2f,%.2f)..(%.2f,%.2f) next=(%.2f,%.2f) "
            "legs=%.2f/%.2f net=%.0f total=%.0f uturn=%d back=%.3f %s %.3f/%.3f",
            i, span, cursor.getX(), cursor.getY(), first.getX(), first.getY(),
            last_corner.getX(), last_corner.getY(), after.getX(), after.getY(),
            cursor.distance(first), last_corner.distance(after),
            net * 180.0 / M_PI, total * 180.0 / M_PI, uturn ? 1 : 0,
            back_off, outcome, value, limit);
        };

      // An arc of radius R meets the legs R*tan(net/2) from the vertex and cuts
      // back_off*tan(net/4) inside it, which bounds the offset at both ends.
      double lo = kMinBackoffRadii * turnRadius(robot, net, continuous);
      double hi = room;
      if (!uturn) {
        if (net > M_PI - 1e-3) {
          report("reversal-too-wide", 0.0, hop, kUturnHopWidths * op_width);
          continue;
        }
        lo = std::max(
          kRadiusMargin * turnRadius(robot, net, continuous) * std::tan(0.5 * net), lo);
        hi = std::min(hi, cut_tol / std::max(std::tan(0.25 * net), 1e-6));
      }
      // On a jog the corners cancel, so tan(net/4) and that bound vanish with
      // them. Bound the offset by the ground the maneuver makes up instead.
      hi = std::min(hi, sep + kRadiusMargin * (1.0 + total) * radius);
      if (lo > hi) {
        report("no-room needs/has", 0.0, lo, hi);
        continue;
      }

      // A fillet is symmetric, so it meets both legs the same distance out. A
      // u-turn only leaves one leg and arrives on the other, so it takes what
      // each has: room is what turns a cusped teardrop into a drivable turn.
      double in_reach = hi;
      double out_reach = hi;
      if (uturn) {
        const double reach = kUturnReachRadii * turnRadius(robot, net, continuous);
        in_reach = std::max(lo, std::min(back_room, reach));
        out_reach = std::max(lo, std::min(fwd_room, reach));
      }
      if (pin_in) {
        in_reach = 0.0;
      }
      if (pin_out) {
        out_reach = 0.0;
      }

      // A continuous curvature turn needs more room than those tangent lengths,
      // by an amount not worth predicting, so widen until the planner answers.
      // A fillet is tried tightest first, where it cuts least; a u-turn widest.
      const std::array<double, 3> fracs =
        uturn ? std::array<double, 3>{1.0, 0.5, 0.0} : std::array<double, 3>{0.0, 0.5, 1.0};
      for (const double frac : fracs) {
        const double back_off = pin_in ? 0.0 : lo + frac * (in_reach - lo);
        const double fwd_off = pin_out ? 0.0 : lo + frac * (out_reach - lo);
        const Point entry = util::pointAlong(first, cursor, back_off);
        const Point exit = util::pointAlong(last_corner, after, fwd_off);
        Path arc = curve.createTurn(robot, entry, in_angle, exit, out_angle);
        if (arc.size() == 0) {
          report("planner-returned-nothing", back_off, 0.0, 0.0);
          continue;
        }

        // A feasible pair of poses can still come back as a loop, which sits
        // well inside a corridor several times its radius, so deviation alone
        // lets it through. Length separates them: a loop costs 2*pi radii more.
        // A u-turn is allowed the teardrop it needs, anything else is not.
        double arc_len = 0.0;
        for (const auto & s : arc.getStates()) {
          arc_len += s.len;
        }
        const double budget = uturn ?
          entry.distance(exit) + (2.0 * M_PI + total) * radius :
          entry.distance(exit) + std::max(back_off, radius) * total + kLoopSlackRadii * radius;
        if (arc_len > budget) {
          report("over-budget arc/max", back_off, arc_len, budget);
          continue;
        }

        std::vector<Point> track{entry};
        for (size_t k = i; k <= j; ++k) {
          track.push_back(poly[k]);
        }
        track.push_back(exit);
        // A u-turn swings past the vertices by up to the diameter it turns on,
        // so it is allowed the corridor plus that rather than exempted.
        const double dev = util::deviationFromTrack(arc, track);
        const double dev_max = uturn ?
          cut_tol + 2.0 * turnRadius(robot, net, continuous) : cut_tol;
        if (dev > dev_max) {
          report("strays dev/max", back_off, dev, dev_max);
          continue;
        }

        report("ROUND arc/max", back_off, arc_len, budget);
        RCLCPP_DEBUG(
          logger, "  -> turn %s", util::turnLabel(turns->size()).c_str());
        addStraight(path, robot, cursor, entry);
        // Left as the planner returned it, so a Reeds-Shepp leg stays reverse at
        // turning speed rather than being reported as a pass at cruise.
        path += arc;
        turns->push_back(arc);
        cursor = exit;
        i = j + 1;
        rounded = true;
        break;
      }
    }

    if (!rounded) {
      RCLCPP_DEBUG(
        logger, "corner %zu (%.2f,%.2f) SHARP", i, poly[i].getX(), poly[i].getY());
      ++sharp;
      addStraight(path, robot, cursor, poly[i]);
      cursor = poly[i];
      ++i;
    }
  }
  addStraight(path, robot, cursor, poly.back());
  return sharp;
}

}  // namespace

Path PathGenerator::generatePath(
  const F2CRoute & route, const opennav_coverage_msgs::msg::PathMode & settings)
{
  // The turns held for visualization are this route's, not everything planned
  // since the server came up.
  connection_turns_.clear();

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
  // The corner maths sizes its turns off this, and it is only resolved here.
  active_continuity_type_ = action_continuity_type;

  RCLCPP_INFO(
    logger_,
    "Generating path with curve: %s", toString(action_type, action_continuity_type).c_str());
  curve->setDiscretization(turn_point_distance);
  size_t sharp_corners = 0;
  Path path = assemblePath(route, *curve, sharp_corners);
  if (sharp_corners > 0) {
    RCLCPP_WARN(
      logger_,
      "%zu connection corner(s) left sharp: no turn fits within %.2fm of the track. "
      "Raise corner_cut_tolerance or lower min_turning_radius if the controller cannot "
      "hold them.",
      sharp_corners, corner_cut_tol_);
  }

  // Optionally thin out near-duplicate points (e.g. in turns)
  if (reduce_path_) {
    path.reduce(reduce_min_dist_);
  }
  return path;
}

Path PathGenerator::assemblePath(
  const F2CRoute & route, f2c::pp::TurningBase & curve, size_t & sharp_corners)
{
  auto & robot = robot_params_->getRobot();
  Path path;
  sharp_corners = 0;
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    const Swaths prev = (i > 0) ? route.getSwaths(i - 1) : Swaths();
    const F2CMultiPoint connection =
      (i < route.sizeConnections()) ? route.getConnection(i) : F2CMultiPoint();
    sharp_corners += appendConnection(path, prev, connection, route.getSwaths(i), curve);
    path += generator_->planPath(robot, route.getSwaths(i), curve);
  }
  if (route.sizeConnections() > route.sizeVectorSwaths()) {
    sharp_corners += appendConnection(
      path, route.getLastSwaths(), route.getLastConnection(), Swaths(), curve);
  }
  return path;
}

size_t PathGenerator::appendConnection(
  Path & path, const Swaths & prev, const F2CMultiPoint & connection,
  const Swaths & next, f2c::pp::TurningBase & curve)
{
  auto & robot = robot_params_->getRobot();
  const bool has_prev = prev.size() > 0;
  const bool has_next = next.size() > 0;
  if (!has_prev && !has_next && connection.size() < 2) {
    return 0;
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
    return 0;
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
    max_dev = std::max(max_dev, util::distanceToSegment(pts[i], pts.front(), pts.back()));
  }

  // The headland u-turn between neighbouring swaths, where the track is the
  // straight hop and there is nothing to follow. A track that runs out to the
  // cell edge instead strays by half an operation width, the offset the
  // outermost swath keeps from it, so the tolerance has to stay well under that
  // figure or those connections shortcut the headland.
  const double op_width = robot_params_->getOperationWidth();
  bool direct = has_prev && has_next && max_dev < kDirectHopDev * op_width;

  // Neighbouring swaths that double back on each other connect through a u-turn,
  // which is not a corner to round: an arc through a 180 degree vertex has no
  // tangent length, so the maneuver has to swing past the vertex into the
  // headland rather than stay inside the track. The hop guards it - a long
  // connection that happens to reverse is a detour to follow, not a corner.
  if (has_prev && has_next && !direct) {
    const double delta = prev.back().getOutAngle() - next[0].getInAngle();
    const double reversal = std::fabs(std::atan2(std::sin(delta), std::cos(delta)));
    const double hop = prev.back().endPoint().distance(next[0].startPoint());
    direct = reversal > kReversalSweep && hop < kUturnHopWidths * op_width;
  }

  if (direct) {
    Path turn = curve.createTurn(
      robot, prev.back().endPoint(), prev.back().getOutAngle(),
      next[0].startPoint(), next[0].getInAngle());
    RCLCPP_DEBUG(
      logger_, "direct hop (%.2f,%.2f)->(%.2f,%.2f) max_dev=%.2f -> turn %s",
      prev.back().endPoint().getX(), prev.back().endPoint().getY(),
      next[0].startPoint().getX(), next[0].startPoint().getY(), max_dev,
      util::turnLabel(connection_turns_.size()).c_str());
    path += turn;
    connection_turns_.push_back(turn);
    return 0;
  }

  // Round the corners of the detour (e.g. around a concave notch) but otherwise
  // follow it, so the vehicle stays on the planned border/corridor track.
  const std::vector<Point> poly = util::simplifyPolyline(pts, kSimplifyTol * op_width);
  if (poly.size() < 2) {
    return 0;
  }

  const bool continuous = active_continuity_type_ == PathContinuityType::CONTINUOUS;
  const std::optional<double> start_angle = has_prev ?
    std::optional<double>(prev.back().getOutAngle()) : std::nullopt;
  const std::optional<double> end_angle = has_next ?
    std::optional<double>(next[0].getInAngle()) : std::nullopt;
  return appendRoundedTrack(
    path, poly, robot, curve, op_width, corner_cut_tol_, continuous, start_angle, end_angle,
    logger_, &connection_turns_);
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
