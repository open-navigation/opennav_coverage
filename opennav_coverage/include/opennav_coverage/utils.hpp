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

#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <utility>

#include "fields2cover.h" // NOLINT
#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/types.hpp"
#include "nav2_util/node_utils.hpp"
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

  if (raw_path.states.size() == 0) {
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

  PathSectionType curr_state = path.states[0].type;
  if (curr_state == PathSectionType::SWATH) {
    curr_swath_start = path.states[0].point;
  } else if (curr_state == PathSectionType::TURN) {
    msg.turns.push_back(nav_msgs::msg::Path());
    msg.turns.back().header = header;
    curr_turn = &msg.turns.back();
  }

  for (unsigned int i = 0; i != path.states.size(); i++) {
    if (curr_state == path.states[i].type && path.states[i].type == PathSectionType::SWATH) {
      // Continuing swath so...
      // (1) no action required.
    } else if (curr_state == path.states[i].type && path.states[i].type == PathSectionType::TURN) {
      // Continuing a turn so...
      // (1) keep populating
      curr_turn->poses.push_back(toMsg(path.states[i]));
    } else if (curr_state != path.states[i].type && path.states[i].type == PathSectionType::TURN) {
      // Transitioning from a swath to a turn so...
      // (1) Complete the existing swath
      opennav_coverage_msgs::msg::Swath swath;
      swath.start = toMsg(curr_swath_start);
      swath.end = toMsg(path.states[i - 1].point);
      msg.swaths.push_back(swath);
      // (2) Start a new turn path
      msg.turns.push_back(nav_msgs::msg::Path());
      msg.turns.back().header = header;
      curr_turn = &msg.turns.back();
      curr_turn->poses.push_back(toMsg(path.states[i]));
    } else if (curr_state != path.states[i].type && path.states[i].type == PathSectionType::SWATH) {
      // Transitioning from a turn to a swath so...
      // (1) Update new swath starting point
      curr_swath_start = path.states[i].point;
    }

    curr_state = path.states[i].type;

    if (path.states[i].type != PathSectionType::SWATH &&
      path.states[i].type != PathSectionType::TURN)
    {
      throw std::runtime_error("Unknown type of path state detected, cannot obtain path!");
    }
  }

  if (curr_state == PathSectionType::SWATH) {
    opennav_coverage_msgs::msg::Swath swath;
    swath.start = toMsg(curr_swath_start);
    swath.end = toMsg(path.states.back().point);
    msg.swaths.push_back(swath);
  }

  return msg;
}

/**
 * @brief Converts full path to nav_msgs/path message for action client, visualization
 * and use in direct-sending to a controller to replace the planner server. Interpolates
 * the F2C path to make it dense for following semantics.
 * @param path Full path to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return nav_msgs/Path Path
 */
inline nav_msgs::msg::Path toNavPathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian,
  const float & pt_dist)
{
  using f2c::types::PathSectionType;
  nav_msgs::msg::Path msg;
  msg.header = header;

  if (raw_path.states.size() == 0) {
    return msg;
  }

  Path path = raw_path;
  if (!is_cartesian) {
    path = f2c::Transform::transformToPrevCRS(raw_path, field);
  } else {
    path.moveTo(field.getRefPoint());
  }

  for (unsigned int i = 0; i != path.states.size(); i++) {
    // Swaths come in pairs of start-end sequentially
    if (i > 0 && path.states[i].type == PathSectionType::SWATH &&
      path.states[i - 1].type == PathSectionType::SWATH)
    {
      const float & x0 = path.states[i - 1].point.getX();
      const float & y0 = path.states[i - 1].point.getY();
      const float & x1 = path.states[i].point.getX();
      const float & y1 = path.states[i].point.getY();

      const float dist = hypotf(x1 - x0, y1 - y0);
      const float ux = (x1 - x0) / dist;
      const float uy = (y1 - y0) / dist;
      float curr_dist = pt_dist;

      geometry_msgs::msg::PoseStamped pose;
      pose.pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(path.states[i].angle);
      pose.pose.position.x = x0;
      pose.pose.position.y = y0;
      pose.pose.position.z = path.states[i].point.getZ();

      while (curr_dist < dist) {
        pose.pose.position.x += pt_dist * ux;
        pose.pose.position.y += pt_dist * uy;
        msg.poses.push_back(pose);
        curr_dist += pt_dist;
      }
    } else {
      // Turns are already dense paths
      msg.poses.push_back(toMsg(path.states[i]));
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
    return idx_ <= max_idx_;
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
    if (idx_ < max_idx_ - 1) {
      return std::make_pair(&path_components_.swaths[idx_], &path_components_.turns[idx_]);
    } else {
      return std::make_pair(&path_components_.swaths[idx_], nullptr);
    }
  }

  opennav_coverage_msgs::msg::PathComponents & path_components_;
  unsigned int idx_;
  unsigned int max_idx_;
};

}  // namespace util

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__UTILS_HPP_
