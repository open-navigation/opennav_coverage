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
#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/robot_params.hpp"
#include "opennav_coverage/route_generator.hpp"
#include "opennav_coverage/swath_generator.hpp"
#include "opennav_coverage/path_generator.hpp"
#include "tf2/utils.hpp"
#include "fields2cover/utils/random.h"

// Luckily, F2C has very high test coverage so we only need to test what we touch

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_coverage
{

class PathShim : public PathGenerator
{
public:
  template<typename NodeT>
  explicit PathShim(const NodeT & node, RobotParams * robot_params)
  : PathGenerator(node, robot_params)
  {}

  TurningBasePtr createCurveShim(const PathType & type, const PathContinuityType & c_type)
  {
    return createCurve(type, c_type);
  }

  std::string toStringShim(const PathType & type, const PathContinuityType & c_type)
  {
    return toString(type, c_type);
  }

  PathType toTypeShim(const std::string & str)
  {
    return toType(str);
  }

  PathContinuityType toContinuityTypeShim(const std::string & str)
  {
    return toContinuityType(str);
  }
};

TEST(PathTests, TestpathUtils)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams params(node);
  auto generator = PathShim(node, &params);

  EXPECT_EQ(generator.toTypeShim("FAKE"), PathType::UNKNOWN);
  EXPECT_EQ(generator.toTypeShim("REEDS_SHEPP"), PathType::REEDS_SHEPP);
  EXPECT_EQ(generator.toTypeShim("reeds_shepp"), PathType::REEDS_SHEPP);
  EXPECT_EQ(generator.toTypeShim("DUBIN"), PathType::DUBIN);
  EXPECT_EQ(generator.toTypeShim("dubin"), PathType::DUBIN);

  EXPECT_EQ(generator.toContinuityTypeShim("FAKE"), PathContinuityType::UNKNOWN);
  EXPECT_EQ(generator.toContinuityTypeShim("CONTINUOUS"), PathContinuityType::CONTINUOUS);
  EXPECT_EQ(generator.toContinuityTypeShim("continuous"), PathContinuityType::CONTINUOUS);
  EXPECT_EQ(generator.toContinuityTypeShim("DISCONTINUOUS"), PathContinuityType::DISCONTINUOUS);
  EXPECT_EQ(generator.toContinuityTypeShim("discontinuous"), PathContinuityType::DISCONTINUOUS);

  EXPECT_GT(generator.toStringShim(PathType::UNKNOWN, PathContinuityType::UNKNOWN).size(), 20u);
  EXPECT_GT(
    generator.toStringShim(
      PathType::REEDS_SHEPP, PathContinuityType::CONTINUOUS).size(), 20u);
  EXPECT_GT(
    generator.toStringShim(
      PathType::DUBIN, PathContinuityType::DISCONTINUOUS).size(), 20u);

  EXPECT_TRUE(generator.createCurveShim(PathType::REEDS_SHEPP, PathContinuityType::CONTINUOUS));
  EXPECT_TRUE(generator.createCurveShim(PathType::REEDS_SHEPP, PathContinuityType::DISCONTINUOUS));
  EXPECT_TRUE(generator.createCurveShim(PathType::DUBIN, PathContinuityType::CONTINUOUS));
  EXPECT_TRUE(generator.createCurveShim(PathType::DUBIN, PathContinuityType::DISCONTINUOUS));
  EXPECT_FALSE(generator.createCurveShim(PathType::UNKNOWN, PathContinuityType::UNKNOWN));
  EXPECT_FALSE(generator.createCurveShim(PathType::UNKNOWN, PathContinuityType::DISCONTINUOUS));
  EXPECT_FALSE(generator.createCurveShim(PathType::UNKNOWN, PathContinuityType::CONTINUOUS));
  EXPECT_FALSE(generator.createCurveShim(PathType::REEDS_SHEPP, PathContinuityType::UNKNOWN));
  EXPECT_FALSE(generator.createCurveShim(PathType::DUBIN, PathContinuityType::UNKNOWN));

  generator.setPathMode("a mode");
  generator.setPathContinuityMode("another mode");
  generator.setTurnPointDistance(0.1);
}

TEST(PathTests, TestpathGeneration)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteGenerator route_gen(node);
  PathShim generator(node, &robot_params);

  // Generate some toy route
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);
  opennav_coverage_msgs::msg::RouteMode rt_settings;
  auto route = route_gen.generateRoute(cells, sbc, rt_settings);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::PathMode settings;
  auto path1 = generator.generatePath(route, settings);
  EXPECT_GT(path1.size(), 0u);
  EXPECT_TRUE(std::isfinite(path1.getTaskTime()));
  settings.mode = "REEDS_SHEPP";
  settings.continuity_mode = "CONTINUOUS";
  auto path2 = generator.generatePath(route, settings);
  EXPECT_GT(path2.size(), 0u);
}

TEST(PathTests, TestpathGenerationFromF2CRoute)
{
  // B1-T6: generatePath(F2CRoute) overload returns non-empty path with finite task time
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteGenerator route_gen(node);
  PathShim generator(node, &robot_params);

  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));

  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode rt_settings;
  rt_settings.mode = "TSP";
  F2CRoute tsp_route = route_gen.generateRoute(cells, sbc, rt_settings);
  ASSERT_FALSE(tsp_route.isEmpty());

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(tsp_route, path_settings);
  EXPECT_GT(path.size(), 0u);
  EXPECT_TRUE(std::isfinite(path.getTaskTime()));
}

TEST(PathTests, TestpathGenerationMultiCellConnections)
{
  // A multi-cell TSP route carries inter-cell connections; assemblePath must
  // stitch them without dropping swaths or the connection track.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteGenerator route_gen(node);
  PathShim generator(node, &robot_params);

  F2CCells cells;
  cells.addGeometry(F2CCell(F2CLinearRing({
      F2CPoint(0, 0), F2CPoint(100, 0), F2CPoint(100, 100),
      F2CPoint(0, 100), F2CPoint(0, 0)})));
  cells.addGeometry(F2CCell(F2CLinearRing({
      F2CPoint(100, 0), F2CPoint(200, 0), F2CPoint(200, 100),
      F2CPoint(100, 100), F2CPoint(100, 0)})));

  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode rt_settings;
  rt_settings.mode = "TSP";
  rt_settings.tsp_time_limit = 1;
  F2CRoute route = route_gen.generateRoute(cells, sbc, rt_settings);
  ASSERT_FALSE(route.isEmpty());
  ASSERT_GE(route.sizeConnections(), 1u);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  EXPECT_GT(path.size(), 0u);
  EXPECT_TRUE(std::isfinite(path.getTaskTime()));
}

// [AI generated] The tool offset tests and their helpers were generated by AI and reviewed by
// the author

// The largest distance between the end of a path state and the start of the next state
double maxGap(const Path & path)
{
  double gap = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    gap = std::max(gap, path.getState(i).atEnd().distance(path.getState(i + 1).point));
  }
  return gap;
}

// The points of the followed connections of a path, in order
std::vector<Point> followedPoints(const Path & path)
{
  std::vector<Point> points;
  for (const auto & state : path.getStates()) {
    if (state.type != f2c::types::PathSectionType::HL_SWATH) {
      continue;
    }
    for (const Point & point : {state.point, state.atEnd()}) {
      if (points.empty() || points.back().distance(point) > 1e-6) {
        points.push_back(point);
      }
    }
  }
  return points;
}

void expectPoints(const std::vector<Point> & points, const std::vector<Point> & expected)
{
  ASSERT_EQ(points.size(), expected.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_LT(points[i].distance(expected[i]), 1e-6) << i;
  }
}

Swath toolSwath(const Point & start, const Point & end)
{
  return Swath(LineString({start, end}), 2.5);
}

TEST(PathTests, TestpathToolOffset)
{
  // On routes of the route planner, the tool covers each swath of the route, in the order of
  // the route, for a tool in front of, behind and beside the base
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteGenerator route_gen(node);
  PathShim generator(node, &robot_params);

  F2CCells cells;
  cells.addGeometry(F2CCell(F2CLinearRing({
      F2CPoint(0, 0), F2CPoint(50, 0), F2CPoint(50, 30), F2CPoint(0, 30), F2CPoint(0, 0)})));
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);
  opennav_coverage_msgs::msg::PathMode dubins;

  for (const auto & mode : {"BOUSTROPHEDON", "TSP"}) {
    opennav_coverage_msgs::msg::RouteMode rt_settings;
    rt_settings.mode = mode;
    rt_settings.tsp_time_limit = 1;
    std::optional<F2CPoint> start;
    if (rt_settings.mode == "TSP") {
      start = F2CPoint(-5, -5);
    }
    const F2CRoute route = route_gen.generateRoute(cells, sbc, rt_settings, start);
    ASSERT_FALSE(route.isEmpty());
    std::vector<Swath> tool_swaths;
    for (const auto & swaths : route.getVectorSwaths()) {
      tool_swaths.insert(tool_swaths.end(), swaths.begin(), swaths.end());
    }
    const Path tool_path = generator.generatePath(route, dubins);

    for (const Point & d : {Point(1.4, 0), Point(-1, 0), Point(0, 0.8), Point(-1, 0.3)}) {
      const Path path = generator.generatePath(util::toBaseFrame(route, d), dubins);
      size_t k = 0;
      for (const auto & state : path.getStates()) {
        if (state.type != f2c::types::PathSectionType::SWATH) {
          continue;
        }
        ASSERT_LT(k, tool_swaths.size());
        const Point shift(
          d.getX() * std::cos(state.angle) - d.getY() * std::sin(state.angle),
          d.getX() * std::sin(state.angle) + d.getY() * std::cos(state.angle));
        EXPECT_LT((state.point + shift).distance(tool_swaths[k].startPoint()), 1e-6) << mode;
        EXPECT_LT((state.atEnd() + shift).distance(tool_swaths[k].endPoint()), 1e-6) << mode;
        ++k;
      }
      EXPECT_EQ(k, tool_swaths.size());
      // The path has no gap. A turn state ends some centimeters from the next state, also
      // with no offset
      EXPECT_LT(maxGap(path), 0.05) << mode;
      // The start pose is a position of the base
      if (start) {
        EXPECT_LT(path.getState(0).point.distance(*start), 1e-6);
        EXPECT_LT(path.atEnd().distance(*start), 1e-6);
      }
    }

    // No offset gives the path of the route
    const Path same = generator.generatePath(util::toBaseFrame(route, Point(0, 0)), dubins);
    ASSERT_EQ(same.size(), tool_path.size());
    for (size_t i = 0; i < same.size(); ++i) {
      EXPECT_LT(same.getState(i).point.distance(tool_path.getState(i).point), 1e-6) << i;
      EXPECT_NEAR(same.getState(i).len, tool_path.getState(i).len, 1e-6) << i;
    }
  }
}

TEST(PathTests, TestpathToolOffsetConnection)
{
  // Two reversing swaths and a connection that follows a border: the first and the last point
  // of the connection move with the swaths, and the points on the border stay
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);
  opennav_coverage_msgs::msg::PathMode dubins;

  F2CRoute route;
  route.addSwaths(Swaths({toolSwath(Point(0, 0), Point(20, 0))}));
  route.addConnection(F2CMultiPoint({Point(20, 0), Point(25, 0), Point(25, 8), Point(20, 8)}));
  route.addSwaths(Swaths({toolSwath(Point(20, 8), Point(0, 8))}));

  const auto followed = [&](const Point & tool_offset) {
      return followedPoints(generator.generatePath(util::toBaseFrame(route, tool_offset), dubins));
    };
  expectPoints(
    followedPoints(generator.generatePath(route, dubins)),
    {Point(20, 0), Point(25, 0), Point(25, 8), Point(20, 8)});
  // Tool in front: the base leaves the swath 2 m before its end and starts 2 m before the next
  expectPoints(followed(Point(2, 0)), {Point(18, 0), Point(25, 0), Point(25, 8), Point(22, 8)});
  // Tool behind
  expectPoints(followed(Point(-2, 0)), {Point(22, 0), Point(25, 0), Point(25, 8), Point(18, 8)});
  // Tool at the left: the base is at the right of each swath
  expectPoints(
    followed(Point(0, 0.5)), {Point(20, -0.5), Point(25, 0), Point(25, 8), Point(20, 8.5)});
  // Known limit: a tool 6 m behind puts the base past the border, and it turns back to it
  expectPoints(followed(Point(-6, 0)), {Point(26, 0), Point(25, 0), Point(25, 8), Point(14, 8)});
}

TEST(PathTests, TestpathToolOffsetDecision)
{
  // The choice between a planned turn and the connection uses the swath ends of the route. With
  // a tool offset these are the swath ends of the base, so the choice can change
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);
  opennav_coverage_msgs::msg::PathMode dubins;

  // Near the straight line between the tool swath ends: a planned turn. The base swath ends of
  // a rear tool are 2 m farther out, so the connection is followed
  F2CRoute near;
  near.addSwaths(Swaths({toolSwath(Point(20, 0), Point(0, 0))}));
  near.addConnection(F2CMultiPoint({Point(0, 0), Point(-1, 0), Point(-1, 7), Point(0, 7)}));
  near.addSwaths(Swaths({toolSwath(Point(0, 7), Point(20, 7))}));
  EXPECT_TRUE(followedPoints(generator.generatePath(near, dubins)).empty());
  expectPoints(
    followedPoints(generator.generatePath(util::toBaseFrame(near, Point(-2, 0)), dubins)),
    {Point(-2, 0), Point(-1, 0), Point(-1, 7), Point(2, 7)});

  // A detour from the straight line between the tool swath ends: followed. It is near the
  // straight line between the base swath ends of a rear tool: a planned turn
  F2CRoute detour;
  detour.addSwaths(Swaths({toolSwath(Point(20, 0), Point(0, 0))}));
  detour.addConnection(F2CMultiPoint({Point(0, 0), Point(-1.5, 2.5), Point(0, 5)}));
  detour.addSwaths(Swaths({toolSwath(Point(0, 5), Point(20, 5))}));
  expectPoints(
    followedPoints(generator.generatePath(detour, dubins)),
    {Point(0, 0), Point(-1.5, 2.5), Point(0, 5)});
  EXPECT_TRUE(
    followedPoints(generator.generatePath(util::toBaseFrame(detour, Point(-3, 0)), dubins))
    .empty());
}

}  // namespace opennav_coverage
