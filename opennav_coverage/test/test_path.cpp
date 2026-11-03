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

// [AI generated] SwathJoints, swathJoints(), expectToolOnSwaths() and TestpathToolOffset
// were generated by AI and reviewed by the author
struct SwathJoints
{
  double max_gap{0.0};
  int jumps{0};
};

SwathJoints swathJoints(const Path & path)
{
  SwathJoints joints;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const PathState & a = path.getState(i);
    const PathState & b = path.getState(i + 1);
    if ((a.type == f2c::types::PathSectionType::SWATH) ==
      (b.type == f2c::types::PathSectionType::SWATH))
    {
      continue;
    }
    joints.max_gap = std::max(joints.max_gap, a.atEnd().distance(b.point));
    if (std::abs(std::remainder(b.angle - a.angle, 2.0 * M_PI)) > M_PI / 2.0 + 1e-6) {
      joints.jumps++;
    }
  }
  return joints;
}

// For each offset, the tool covers the route swaths in order, and the joints are not worse
// than with no offset
void expectToolOnSwaths(
  PathGenerator & generator, const F2CRoute & route,
  const opennav_coverage_msgs::msg::PathMode & settings)
{
  std::vector<Swath> tool_swaths;
  for (const auto & swaths : route.getVectorSwaths()) {
    tool_swaths.insert(tool_swaths.end(), swaths.begin(), swaths.end());
  }
  const auto zero_joints = swathJoints(generator.generatePath(route, settings));

  for (const Point & d : {Point(1.4, -0.3), Point(-1, 0.3), Point(0, 0.8)}) {
    const Path path = generator.generatePath(util::toBaseFrame(route, d), settings);
    size_t k = 0;
    for (const auto & state : path.getStates()) {
      if (state.type != f2c::types::PathSectionType::SWATH) {
        continue;
      }
      ASSERT_LT(k, tool_swaths.size());
      const Point shift(
        d.getX() * std::cos(state.angle) - d.getY() * std::sin(state.angle),
        d.getX() * std::sin(state.angle) + d.getY() * std::cos(state.angle));
      EXPECT_LT((state.point + shift).distance(tool_swaths[k].startPoint()), 1e-6);
      EXPECT_LT((state.atEnd() + shift).distance(tool_swaths[k].endPoint()), 1e-6);
      ++k;
    }
    EXPECT_EQ(k, tool_swaths.size());
    const auto joints = swathJoints(path);
    EXPECT_LE(joints.max_gap, std::max(1e-3, zero_joints.max_gap));
    EXPECT_LE(joints.jumps, zero_joints.jumps);
  }
}

TEST(PathTests, TestpathToolOffset)
{
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
    expectToolOnSwaths(generator, route, dubins);
  }

  // U-shaped connection: the interior points must move too, else the base drives to the tool
  // corner (20, 8)
  F2CRoute transit;
  transit.addSwaths(Swaths({Swath(LineString({Point(0, 0), Point(20, 0)}), 2.5, 0)}));
  transit.addConnection(F2CMultiPoint({Point(20, 0), Point(20, 8), Point(0, 8), Point(0, 1)}));
  transit.addSwaths(Swaths({Swath(LineString({Point(0, 1), Point(20, 1)}), 2.5, 1)}));
  expectToolOnSwaths(generator, transit, dubins);
}

}  // namespace opennav_coverage
