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

#ifndef NAV2_COVERAGE__UTILS_HPP_
#define NAV2_COVERAGE__UTILS_HPP_

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "fields2cover.h" // NOLINT
#include "rclcpp/rclcpp.hpp"
#include "nav2_coverage/types.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace nav2_coverage
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
inline nav2_complete_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const Swaths & raw_swaths, const F2CField & field,
  const bool ordered, const std_msgs::msg::Header & header, const bool is_cartesian)
{
  nav2_complete_coverage_msgs::msg::PathComponents msg;
  msg.contains_turns = false;
  msg.swaths_ordered = ordered;
  msg.header = header;
  msg.swaths.resize(raw_swaths.size());

  Swaths swaths = raw_swaths;
  if (!is_cartesian) {
    swaths = f2c::Transform::transformToPrevCRS(raw_swaths, field);
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
inline nav2_complete_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian)
{
  using f2c::types::PathSectionType;
  nav2_complete_coverage_msgs::msg::PathComponents msg;
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
      nav2_complete_coverage_msgs::msg::Swath swath;
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
    nav2_complete_coverage_msgs::msg::Swath swath;
    swath.start = toMsg(curr_swath_start);
    swath.end = toMsg(path.states.back().point);
    msg.swaths.push_back(swath);
  }

  return msg;
}

/**
 * @brief Converts full path to nav_msgs/path message for action client, visualization
 * and use in direct-sending to a controller to replace the planner server
 * @param path Full path to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return nav_msgs/Path Path
 */
inline nav_msgs::msg::Path toNavPathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian)
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
  }

  msg.poses.resize(path.states.size());
  for (unsigned int i = 0; i != path.states.size(); i++) {
    msg.poses[i] = toMsg(path.states[i]);
  }

  return msg;
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

}  // namespace util

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__UTILS_HPP_
