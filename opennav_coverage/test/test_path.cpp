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
#include "tf2/utils.h"
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
  auto field = rand.generateRandField(5, 1e5);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  auto swaths = swath_gen.generateSwaths(field.field.getGeometry(0), sw_settings);
  opennav_coverage_msgs::msg::RouteMode rt_settings;
  auto route = route_gen.generateRoute(swaths, rt_settings);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::PathMode settings;
  auto path1 = generator.generatePath(route, settings);
  settings.mode = "REEDS_SHEPP";
  settings.continuity_mode = "CONTINUOUS";
  auto path2 = generator.generatePath(route, settings);
}

}  // namespace opennav_coverage
