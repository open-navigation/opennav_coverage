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

}  // namespace opennav_coverage
