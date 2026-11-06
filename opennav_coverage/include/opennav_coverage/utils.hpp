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

#ifndef OPENNAV_COVERAGE__UTILS_HPP_
#define OPENNAV_COVERAGE__UTILS_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <utility>

#include "fields2cover.h" // NOLINT
#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/types.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace opennav_coverage
{

namespace util
{

/**
 * @brief Converts F2C Point to ROS Point32
 * @param Point point
 * @return geometry_msgs Point
 */
inline geometry_msgs::msg::Point pointToPoint32(const geometry_msgs::msg::Point32 & pt)
{
  geometry_msgs::msg::Point msg;
  msg.x = pt.x;
  msg.y = pt.y;
  msg.z = pt.z;
  return msg;
}

/**
 * @brief Converts F2C Point to ROS Point32
 * @param Point point
 * @return geometry_msgs Point
 */
inline geometry_msgs::msg::Point32 toMsg(const Point & pt)
{
  geometry_msgs::msg::Point32 msg;
  msg.x = pt.getX();
  msg.y = pt.getY();
  msg.z = pt.getZ();
  return msg;
}

/*
 * @brief Converts F2C Point to ROS Pose Stamped
 * @param Point point
 * @return geometry_msgs Point
 */
inline geometry_msgs::msg::PoseStamped toMsg(const PathState & state)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.pose.position.x = state.point.getX();
  msg.pose.position.y = state.point.getY();
  msg.pose.position.z = state.point.getZ();
  msg.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(state.angle);
  return msg;
}

/**
 * @brief Converts swaths to coverage path message for action client
 * @param swaths Swaths to convert. May be ordered or unordered.
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return PathComponents Info for action server to utilize
 */
inline opennav_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const Swaths & raw_swaths, const F2CField & field,
  const bool ordered, const std_msgs::msg::Header & header, const bool is_cartesian)
{
  opennav_coverage_msgs::msg::PathComponents msg;
  msg.contains_turns = false;
  msg.swaths_ordered = ordered;
  msg.header = header;
  msg.swaths.resize(raw_swaths.size());

  Swaths swaths = raw_swaths;
  if (!is_cartesian) {
    swaths = f2c::Transform::transformToPrevCRS(raw_swaths, field);
  } else {
    swaths.moveTo(field.getRefPoint());
  }

  for (unsigned int i = 0; i != swaths.size(); i++) {
    msg.swaths[i].start = toMsg(swaths[i].startPoint());
    msg.swaths[i].end = toMsg(swaths[i].endPoint());
  }

  return msg;
}

/**
 * @brief Converts an ordered route to coverage path message (ordered swaths only)
 * @param route Route whose swath groups to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return PathComponents Info for action server to utilize
 */
inline opennav_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const F2CRoute & route, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian)
{
  Swaths ordered;
  for (const auto & group : route.getVectorSwaths()) {
    for (const auto & s : group) {
      ordered.emplace_back(s);
    }
  }
  return toCoveragePathMsg(ordered, field, true, header, is_cartesian);
}

/**
 * @brief Converts full path to coverage path message for action client
 * @param path Full path to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return PathComponents Info for action server to utilize
 */
inline opennav_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian)
{
  using f2c::types::PathSectionType;
  opennav_coverage_msgs::msg::PathComponents msg;
  msg.contains_turns = true;
  msg.swaths_ordered = true;
  msg.header = header;

  if (raw_path.size() == 0) {
    return msg;
  }

  Point curr_swath_start(0.0, 0.0);
  nav_msgs::msg::Path * curr_turn = nullptr;

  Path path = raw_path;
  if (!is_cartesian) {
    path = f2c::Transform::transformToPrevCRS(raw_path, field);
  } else {
    path.moveTo(field.getRefPoint());
  }

  // Decomposition produces HL_SWATH states for inter-cell headland passes;
  // treat them like regular SWATH states.
  auto isSwathLike = [](PathSectionType t) {
      return t == PathSectionType::SWATH || t == PathSectionType::HL_SWATH;
    };

  PathSectionType curr_state = path[0].type;
  if (isSwathLike(curr_state)) {
    curr_swath_start = path[0].point;
  } else if (curr_state == PathSectionType::TURN) {
    msg.turns.push_back(nav_msgs::msg::Path());
    msg.turns.back().header = header;
    curr_turn = &msg.turns.back();
  }

  for (unsigned int i = 0; i != path.size(); i++) {
    const bool prev_swath = isSwathLike(curr_state);
    const bool prev_turn = curr_state == PathSectionType::TURN;
    const bool now_swath = isSwathLike(path[i].type);
    const bool now_turn = path[i].type == PathSectionType::TURN;

    if (prev_swath && now_swath) {
      // Continuing swath (SWATH or HL_SWATH) so...
      // (1) no action required.
    } else if (prev_turn && now_turn) {
      // Continuing a turn so...
      // (1) keep populating
      curr_turn->poses.push_back(toMsg(path[i]));
    } else if (prev_swath && now_turn) {
      // Transitioning from a swath to a turn so...
      // (1) Complete the existing swath
      opennav_coverage_msgs::msg::Swath swath;
      swath.start = toMsg(curr_swath_start);
      swath.end = toMsg(path[i - 1].point);
      msg.swaths.push_back(swath);
      // (2) Start a new turn path
      msg.turns.push_back(nav_msgs::msg::Path());
      msg.turns.back().header = header;
      curr_turn = &msg.turns.back();
      curr_turn->poses.push_back(toMsg(path[i]));
    } else if (prev_turn && now_swath) {
      // Transitioning from a turn to a swath so...
      // (1) Update new swath starting point
      curr_swath_start = path[i].point;
    }

    curr_state = path[i].type;

    if (!isSwathLike(path[i].type) &&
      path[i].type != PathSectionType::TURN)
    {
      throw std::runtime_error("Unknown type of path state detected, cannot obtain path!");
    }
  }

  if (isSwathLike(curr_state)) {
    opennav_coverage_msgs::msg::Swath swath;
    swath.start = toMsg(curr_swath_start);
    swath.end = toMsg(path.back().point);
    msg.swaths.push_back(swath);
  }

  return msg;
}

/**
 * @brief Build a drivable HL_SWATH loop around a field boundary, rotated to start
 * at the vertex closest to `anchor` (the route's first point) to minimize the
 * jump when spliced onto the coverage path. Density added later by discretizeSwathLike.
 * @param area Field/cell whose exterior boundary is driven (e.g. field_no_headland)
 * @param velocity Cruise velocity to tag each state with
 * @param anchor Point the loop's start/end should be nearest to
 * @return Path of HL_SWATH states (empty if the boundary has < 2 points)
 */
inline Path toHeadlandPerimeterPath(
  const Field & area, double velocity, const Point & anchor)
{
  Path path;
  const Polygon ring = area.getExteriorRing();  // closed boundary loop (first == last point)
  const size_t n = ring.size();
  if (n < 2) {
    return path;
  }
  const size_t n_unique = n - 1;  // exclude the duplicated closing point

  size_t start_idx = 0;
  double best_dist2 = 0.0;
  for (size_t i = 0; i < n_unique; ++i) {
    const Point p = ring.getGeometry(i);
    const double dx = p.getX() - anchor.getX();
    const double dy = p.getY() - anchor.getY();
    const double d2 = dx * dx + dy * dy;
    if (i == 0 || d2 < best_dist2) {
      best_dist2 = d2;
      start_idx = i;
    }
  }

  for (size_t k = 0; k < n_unique; ++k) {
    const Point p0 = ring.getGeometry((start_idx + k) % n_unique);
    const Point p1 = ring.getGeometry((start_idx + k + 1) % n_unique);
    const double dx = p1.getX() - p0.getX();
    const double dy = p1.getY() - p0.getY();
    PathState s;
    s.point = p0;
    s.angle = std::atan2(dy, dx);
    s.len = std::hypot(dx, dy);
    s.dir = f2c::types::PathDirection::FORWARD;
    s.type = f2c::types::PathSectionType::HL_SWATH;
    s.velocity = velocity;
    path.addState(s);
  }
  return path;
}

/**
 * @brief Like F2C Path::discretizeSwath but also splits HL_SWATH states, so
 * headland passes become dense followable segments. F2C's discretizeSwath only
 * subdivides SWATH; HL_SWATH would otherwise pass through as a single waypoint.
 * @param path Path to densify
 * @param step_size Max spacing between emitted points
 * @return Densified path
 */
inline Path discretizeSwathLike(const Path & path, double step_size)
{
  using f2c::types::PathSectionType;
  Path out;
  const double step = step_size > 0.0 ? step_size : 0.1;
  for (const auto & s : path.getStates()) {
    if (s.type == PathSectionType::SWATH || s.type == PathSectionType::HL_SWATH) {
      double n_steps = std::max(1.0, std::round(std::fabs(s.len / step)));
      Point start2end = s.atEnd() - s.point;
      for (double j = 0.0; j < n_steps; j += 1.0) {
        PathState state = s;
        state.point = s.point + start2end * (j / n_steps);
        state.len /= n_steps;
        out.addState(state);
      }
    } else {
      out.addState(s);
    }
  }
  return out;
}

/**
 * @brief Converts full path to nav_msgs/path message for action client, visualization
 * and use in direct-sending to a controller to replace the planner server. Interpolates
 * the F2C path to make it dense for following semantics.
 * @param path Full path to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @param out_velocities Optional: if non-null, filled with per-pose velocity (m/s), parallel to poses
 * @param out_is_backward Optional: if non-null, filled with per-pose reverse-direction flags
 * @return nav_msgs/Path Path
 */
inline nav_msgs::msg::Path toNavPathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian,
  const float & pt_dist,
  std::vector<double> * out_velocities = nullptr,
  std::vector<bool> * out_is_backward = nullptr)
{
  nav_msgs::msg::Path msg;
  msg.header = header;
  if (out_velocities) {out_velocities->clear();}
  if (out_is_backward) {out_is_backward->clear();}

  if (raw_path.size() == 0) {
    return msg;
  }

  Path path = raw_path;
  if (!is_cartesian) {
    path = f2c::Transform::transformToPrevCRS(raw_path, field);
  } else {
    path.moveTo(field.getRefPoint());
  }

  // Split SWATH and HL_SWATH states at step_size intervals (F2C's discretizeSwath
  // splits only SWATH, leaving headland passes as single waypoints).
  path = discretizeSwathLike(path, static_cast<double>(pt_dist));

  // Reserve up front so the population loop below doesn't reallocate.
  const auto n = path.size();
  msg.poses.reserve(n);
  if (out_velocities) {out_velocities->reserve(n);}
  if (out_is_backward) {out_is_backward->reserve(n);}

  for (const auto & state : path) {
    msg.poses.push_back(toMsg(state));
    if (out_velocities) {out_velocities->push_back(state.velocity);}
    if (out_is_backward) {
      out_is_backward->push_back(state.dir == f2c::types::PathDirection::BACKWARD);
    }
  }

  return msg;
}

/**
 * @brief Converts full path to nav_msgs/path message in cartesian UTM frame
 * @param path Full path to convert
 * @param header header
 * @return nav_msgs/Path Path
 */
inline nav_msgs::msg::Path toCartesianNavPathMsg(
  const Path & raw_path,
  const std_msgs::msg::Header & header, const float & pt_dist)
{
  return toNavPathMsg(raw_path, F2CField(), header, true, pt_dist);
}

/**
 * @brief Converts action goal coordinates into a Field type
 * Note that this may be in GPS or cartesian coordinates!
 * @param goal Goal to pase
 * @return Field field of goal polygons
 */
inline F2CField getFieldFromGoal(
  typename std::shared_ptr<const typename ComputeCoveragePath::Goal> goal)
{
  auto polygons = goal->polygons;
  if (polygons.size() == 0) {
    throw std::invalid_argument("No field set in action goal!");
  } else if (polygons[0].coordinates.front() != polygons[0].coordinates.back()) {
    throw std::invalid_argument("Outer polygon malformed, first element must equal last!");
  }

  // Get the outer most polygon (usually the only one if no voids)
  Polygon outer_polygon;
  for (unsigned int i = 0; i != polygons[0].coordinates.size(); i++) {
    outer_polygon.addPoint(
      Point(polygons[0].coordinates[i].axis1, polygons[0].coordinates[i].axis2));
  }
  Field field(outer_polygon);

  // Now, parse pontential inner rings (e.g. voids)
  for (unsigned int i = 1; i != polygons.size(); i++) {
    auto polygon = polygons[i];
    if (polygon.coordinates.front() != polygon.coordinates.back()) {
      throw std::invalid_argument("Inner polygon malformed, first element must equal last!");
    }

    Polygon inner_polygon;
    for (unsigned int idx = 0; idx != polygon.coordinates.size(); idx++) {
      inner_polygon.addPoint(Point(polygon.coordinates[idx].axis1, polygon.coordinates[idx].axis2));
    }

    field.addRing(inner_polygon);
  }

  return F2CField(Fields(field));
}

/**
 * @brief Converts a goal-frame point into the frame the field's swaths/cells live in
 * @param pt Point in the goal frame (cartesian or GPS)
 * @param field Field already transformed to its working frame
 * @param is_cartesian Whether the goal coordinates are cartesian
 * @return Point in the field's local frame
 */
inline F2CPoint toFieldFrame(const F2CPoint & pt, const F2CField & field, bool is_cartesian)
{
  // F2CField always stores geometry with its ref point subtracted, so the start point needs
  // the same ref point removed (and, for GPS, projecting to UTM first).
  F2CPoint abs = is_cartesian ?
    pt : f2c::Transform::transform(pt, field.getPrevCRS(), field.getCRS());
  return abs - field.getRefPoint();
}

/**
 * @brief Converts a string to uppercase
 * @param string String to change to uppercase
 */
inline void toUpper(std::string & string)
{
  std::transform(string.begin(), string.end(), string.begin(), ::toupper);
}

/**
 * @brief A Path Components iterator object to get next turn/swath from action return
 *
 * Example Use:
    for (opennav_coverage::util::PathComponentsIterator it(msg); it.isValid(); it.advance()) {
      auto curr_row_info = it.getNext();

      // Always should be valid -- Swath
      (void)std::get<0>(curr_row_info)->start;

      if (std::get<1>(curr_row_info)) {
        // Always should be before last -- Turn
        (void)std::get<1>(curr_row_info)->poses;
      }
    }

    auto last_row_info = it.getNext();
    ASSERT(std::get<1>(last_row_info) == nullptr);
 */
class PathComponentsIterator
{
public:
  /**
   * @brief A Path Components iterator constructor
   * @param PathComponents object to iterate over
   */
  explicit PathComponentsIterator(opennav_coverage_msgs::msg::PathComponents & msg)
  : path_components_(msg), idx_(0)
  {
    if (fabs(path_components_.swaths.size() - path_components_.turns.size()) > 1) {
      throw std::runtime_error("PathComponents size not valid for iteration!");
    } else if (!path_components_.contains_turns) {
      throw std::runtime_error("PathComponents cannot be iterated over without turns!");
    } else if (!path_components_.swaths_ordered) {
      throw std::runtime_error("PathComponents cannot be iterated over without ordered swaths!");
    }

    max_idx_ = path_components_.swaths.size();
  }

  /**
   * @brief For condition if still valid to continue iterating
   */
  bool isValid()
  {
    return idx_ < max_idx_;
  }

  /**
   * @brief For loop marching
   */
  void advance()
  {
    idx_++;
  }

  /**
   * @brief Get the data of the current iteration
   * @return returns a pair of pointers to the current swath and its next turn
   * If at the end, the turn is nullptr, so be sure to check it!
   */
  std::pair<opennav_coverage_msgs::msg::Swath *, nav_msgs::msg::Path *>
  getNext()
  {
    opennav_coverage_msgs::msg::Swath * swath =
      idx_ < path_components_.swaths.size() ? &path_components_.swaths[idx_] : nullptr;
    nav_msgs::msg::Path * turn =
      (idx_ + 1 < max_idx_ && idx_ < path_components_.turns.size()) ?
      &path_components_.turns[idx_] : nullptr;
    return std::make_pair(swath, turn);
  }

  opennav_coverage_msgs::msg::PathComponents & path_components_;
  unsigned int idx_;
  unsigned int max_idx_;
};

}  // namespace util

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__UTILS_HPP_
