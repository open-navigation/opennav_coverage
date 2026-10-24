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
#include <limits>

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

namespace
{

// Distance from `q` to the polyline through the path's states. Straight runs are
// a single state, so measuring to the states alone would miss the track between.
double distanceToPath(const Path & path, const F2CPoint & q)
{
  double best = std::numeric_limits<double>::max();
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1].point;
    const auto & b = path[i].point;
    const double dx = b.getX() - a.getX();
    const double dy = b.getY() - a.getY();
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12) {
      t = std::clamp(
        ((q.getX() - a.getX()) * dx + (q.getY() - a.getY()) * dy) / len2, 0.0, 1.0);
    }
    best = std::min(
      best, std::hypot(q.getX() - (a.getX() + t * dx), q.getY() - (a.getY() + t * dy)));
  }
  return best;
}

double maxHeadingStep(const Path & path)
{
  double worst = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const double delta = path[i].angle - path[i - 1].angle;
    worst = std::max(worst, std::fabs(std::atan2(std::sin(delta), std::cos(delta))));
  }
  return worst;
}

// Two opposing swaths with a gap between them, for connections to span.
void makeSwaths(double x_second, F2CSwaths & first, F2CSwaths & second)
{
  first.emplace_back(F2CSwath(F2CLineString({F2CPoint(0, 0), F2CPoint(0, 20)})));
  second.emplace_back(
    F2CSwath(F2CLineString({F2CPoint(x_second, 20), F2CPoint(x_second, 0)})));
}

}  // namespace

TEST(PathTests, TestConnectionCornersAreSmoothed)
{
  // Corners of a detouring connection must go through the turn planner: a raw
  // boundary vertex is a heading step no turning radius can hold.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths first, second;
  makeSwaths(30.0, first, second);

  // Connection detours over the top with two right-angle corners.
  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), first);
  route.addConnectedSwaths(
    F2CMultiPoint({F2CPoint(0, 20), F2CPoint(0, 40), F2CPoint(30, 40), F2CPoint(30, 20)}),
    second);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  EXPECT_LT(maxHeadingStep(path), 1.0);  // a square corner would leave a ~1.57 rad step

  // Smoothing must round the corners, not skip the detour: a path that cut
  // straight across would satisfy the heading bound above on its own.
  EXPECT_LT(distanceToPath(path, F2CPoint(15, 40)), 0.5);
  EXPECT_LT(distanceToPath(path, F2CPoint(0, 30)), 0.5);
  EXPECT_LT(distanceToPath(path, F2CPoint(30, 30)), 0.5);
}

TEST(PathTests, TestConnectionJogIsSmoothed)
{
  // A jog with two corners a couple of metres apart: too tight to take one at a
  // time, so it has to be rounded as a single S rather than driven square.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths first, second;
  makeSwaths(30.0, first, second);

  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), first);
  route.addConnectedSwaths(
    F2CMultiPoint({
      F2CPoint(0, 20), F2CPoint(0, 40), F2CPoint(14, 40), F2CPoint(16, 42),
      F2CPoint(30, 42), F2CPoint(30, 20)}),
    second);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  EXPECT_LT(maxHeadingStep(path), 1.0);
  EXPECT_LT(distanceToPath(path, F2CPoint(25, 42)), 0.5);
}

TEST(PathTests, TestNeighbourUturnIsPlannedAsATurn)
{
  // Neighbouring swaths doubling back on each other. The track wanders out past
  // the straight-hop tolerance, but this is still a u-turn, and a u-turn is not
  // a corner to round: driving its two vertices square would put a pair of right
  // angles where the headland maneuver belongs.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths first, second;
  first.emplace_back(F2CSwath(F2CLineString({F2CPoint(0, 0), F2CPoint(0, 20)})));
  second.emplace_back(F2CSwath(F2CLineString({F2CPoint(2.5, 20), F2CPoint(2.5, 0)})));

  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), first);
  route.addConnectedSwaths(
    F2CMultiPoint({F2CPoint(0, 20), F2CPoint(0, 21.5), F2CPoint(2.5, 21.5), F2CPoint(2.5, 20)}),
    second);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  // Squaring off the two vertices would leave ~1.57 rad steps at each.
  EXPECT_LT(maxHeadingStep(path), 1.0);
}

TEST(PathTests, TestCurvedConnectionIsNotFlattened)
{
  // Every point of this arc sits millimetres from the chord of its immediate
  // neighbours, so simplifying by local collinearity would drop them one by one
  // and collapse a 5m bulge onto its chord. The kept track must stay on the arc.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths first, second;
  makeSwaths(40.0, first, second);

  F2CMultiPoint arc;
  for (int k = 0; k <= 40; ++k) {
    const double x = static_cast<double>(k);
    arc.addPoint(F2CPoint(x, 20.0 + 5.0 * std::sin(M_PI * x / 40.0)));
  }

  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), first);
  route.addConnectedSwaths(arc, second);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  EXPECT_LT(distanceToPath(path, F2CPoint(20, 25)), 0.5);   // apex
  EXPECT_LT(distanceToPath(path, F2CPoint(10, 23.54)), 0.5);
  EXPECT_LT(distanceToPath(path, F2CPoint(30, 23.54)), 0.5);
}

TEST(PathTests, TestUnroundableCornerKeepsTheTrack)
{
  // With a turning circle far wider than the corridor, no maneuver fits through
  // the corner. The corner then stays sharp on purpose: cutting it would put the
  // vehicle off the planned track, which is worse than a step the controller
  // has to absorb.
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("min_turning_radius", 8.0),
      rclcpp::Parameter("operation_width", 2.5)});
  auto node = std::make_shared<rclcpp::Node>("test_node", options);
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths first, second;
  makeSwaths(30.0, first, second);

  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), first);
  route.addConnectedSwaths(
    F2CMultiPoint({F2CPoint(0, 20), F2CPoint(0, 40), F2CPoint(30, 40), F2CPoint(30, 20)}),
    second);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  // Corners themselves, not just their neighbourhood: nothing may be cut here.
  EXPECT_LT(distanceToPath(path, F2CPoint(0, 40)), 0.1);
  EXPECT_LT(distanceToPath(path, F2CPoint(30, 40)), 0.1);
}

TEST(PathTests, TestSwathHeadingRoundsTheJunction)
{
  // The junction with a swath is a corner: the connection need not leave along
  // the heading the swath holds. Without that heading it is not seen as one.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  // Swath runs east and ends where the connection sets off north.
  F2CSwaths first, second;
  first.emplace_back(F2CSwath(F2CLineString({F2CPoint(-20, 20), F2CPoint(0, 20)})));
  second.emplace_back(F2CSwath(F2CLineString({F2CPoint(30, 20), F2CPoint(30, 0)})));

  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), first);
  route.addConnectedSwaths(
    F2CMultiPoint({F2CPoint(0, 20), F2CPoint(0, 40), F2CPoint(30, 40), F2CPoint(30, 20)}),
    second);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  EXPECT_LT(maxHeadingStep(path), 1.0);  // driven straight off, a ~1.57 rad step

  // Starts on the swath's own end pose, not up the leg at the next vertex.
  const auto & turns = generator.getConnectionTurns();
  ASSERT_FALSE(turns.empty());
  EXPECT_NEAR(turns.front()[0].point.getX(), 0.0, 1e-3);
  EXPECT_NEAR(turns.front()[0].point.getY(), 20.0, 1e-3);
}

TEST(PathTests, TestOpenEndedConnectionFollowsTheTrack)
{
  // Route-end connections have a swath on one side only. The open end holds no
  // heading, so it is no corner, and nothing may be read off it.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths swaths;
  swaths.emplace_back(F2CSwath(F2CLineString({F2CPoint(0, 0), F2CPoint(0, 20)})));

  // A lead-in from off the field, and a lead-out back off it.
  F2CRoute route;
  route.addConnectedSwaths(
    F2CMultiPoint({F2CPoint(-20, -20), F2CPoint(0, -20), F2CPoint(0, 0)}), swaths);
  route.addConnection(
    F2CMultiPoint({F2CPoint(0, 20), F2CPoint(0, 40), F2CPoint(-20, 40)}));

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  EXPECT_LT(maxHeadingStep(path), 1.0);

  // One corner each; the open ends are driven through.
  EXPECT_EQ(generator.getConnectionTurns().size(), 2u);

  EXPECT_LT(distanceToPath(path, F2CPoint(-10, -20)), 0.5);
  EXPECT_LT(distanceToPath(path, F2CPoint(0, 30)), 0.5);
}

TEST(PathTests, TestShallowCornerKeepsAnApproach)
{
  // A shallow corner's fillet tangent collapses with it. The floor under that
  // is what leaves the planner a maneuver rather than two poses 6cm apart.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths swaths;
  swaths.emplace_back(F2CSwath(F2CLineString({F2CPoint(0, 0), F2CPoint(0, 20)})));

  // Lead-out bending 15deg at (0, 50), well over the ~3deg driven straight.
  const double bend = 15.0 * M_PI / 180.0;
  const F2CPoint corner(0, 50);
  const F2CPoint tip(30.0 * std::sin(bend), 50.0 + 30.0 * std::cos(bend));

  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), swaths);
  route.addConnection(F2CMultiPoint({F2CPoint(0, 20), corner, tip}));

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  // Rounded rather than left sharp, and the only corner on the route.
  const auto & turns = generator.getConnectionTurns();
  ASSERT_EQ(turns.size(), 1u);

  // 6cm of tangent here, nothing to turn in: it must start further back.
  EXPECT_GT(turns.front()[0].point.distance(corner), 0.15);

  EXPECT_LT(distanceToPath(path, F2CPoint(0, 35)), 0.5);  // leg in followed
  // The leg out is one state with no point at its end, so check its heading.
  EXPECT_NEAR(path[path.size() - 1].angle, M_PI / 2.0 - bend, 1e-3);
}

TEST(PathTests, TestTightCornersAreFoldedIntoOneTurn)
{
  // Corners too close to round one at a time: rounding the first leaves the
  // second no approach, so the pair has to be taken as one S.
  //
  // The radius sets how close is too close. At the default 0.4m the jog would
  // be small enough to pass as a straight hop and never reach the rounding.
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("min_turning_radius", 2.0),
      rclcpp::Parameter("operation_width", 2.5)});
  auto node = std::make_shared<rclcpp::Node>("test_node", options);
  RobotParams robot_params(node);
  PathShim generator(node, &robot_params);

  F2CSwaths first, second;
  first.emplace_back(F2CSwath(F2CLineString({F2CPoint(0, 0), F2CPoint(0, 20)})));
  second.emplace_back(F2CSwath(F2CLineString({F2CPoint(3, 63), F2CPoint(3, 83)})));

  // Corners at (0,40) and (3,43): 4.24m apart, inside the 4.8m a turn needs.
  F2CRoute route;
  route.addConnectedSwaths(F2CMultiPoint(), first);
  route.addConnectedSwaths(
    F2CMultiPoint({F2CPoint(0, 20), F2CPoint(0, 40), F2CPoint(3, 43), F2CPoint(3, 63)}),
    second);

  opennav_coverage_msgs::msg::PathMode path_settings;
  auto path = generator.generatePath(route, path_settings);
  ASSERT_GT(path.size(), 2u);

  // Taken one at a time they would each round here, and leave two.
  EXPECT_EQ(generator.getConnectionTurns().size(), 1u);

  // Their deflections cancel, so nothing bounds the fold by the corner it cuts.
  // It still has to make the jog: this is the midpoint of the leg between them.
  EXPECT_LT(distanceToPath(path, F2CPoint(1.5, 41.5)), 0.5);

  EXPECT_LT(maxHeadingStep(path), 0.2);  // square corners leave ~0.79 rad
}

}  // namespace opennav_coverage
