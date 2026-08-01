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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/robot_params.hpp"
#include "opennav_coverage/route_generator.hpp"
#include "opennav_coverage/swath_generator.hpp"
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

class RouteShim : public RouteGenerator
{
public:
  template<typename NodeT>
  explicit RouteShim(const NodeT & node)
  : RouteGenerator(node)
  {}

  RouteGeneratorPtr createGeneratorShim(const RouteType & type)
  {
    return createGenerator(type);
  }

  std::string toStringShim(const RouteType & type)
  {
    return toString(type);
  }

  RouteType toTypeShim(const std::string & str)
  {
    return toType(str);
  }
};

// Deterministic square field so B.9 start-point assertions aren't subject to random geometry
static F2CCells makeSquareCells(double s)
{
  F2CLinearRing ring;
  ring.addPoint(0.0, 0.0);
  ring.addPoint(s, 0.0);
  ring.addPoint(s, s);
  ring.addPoint(0.0, s);
  ring.addPoint(0.0, 0.0);
  F2CCells cells;
  cells.addGeometry(F2CCell(ring));
  return cells;
}

// Two s x s cells sharing the x=s edge, to exercise the multi-cell TSP stitch.
static F2CCells makeTwoAdjacentCells(double s)
{
  F2CCells cells;
  cells.addGeometry(F2CCell(F2CLinearRing({
      F2CPoint(0.0, 0.0), F2CPoint(s, 0.0), F2CPoint(s, s),
      F2CPoint(0.0, s), F2CPoint(0.0, 0.0)})));
  cells.addGeometry(F2CCell(F2CLinearRing({
      F2CPoint(s, 0.0), F2CPoint(2 * s, 0.0), F2CPoint(2 * s, s),
      F2CPoint(s, s), F2CPoint(s, 0.0)})));
  return cells;
}

TEST(RouteTests, TestrouteUtils)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = RouteShim(node);

  EXPECT_EQ(generator.toTypeShim("FAKE"), RouteType::UNKNOWN);
  EXPECT_EQ(generator.toTypeShim("BOUSTROPHEDON"), RouteType::BOUSTROPHEDON);
  EXPECT_EQ(generator.toTypeShim("boustrophedon"), RouteType::BOUSTROPHEDON);
  EXPECT_EQ(generator.toTypeShim("SNAKE"), RouteType::SNAKE);
  EXPECT_EQ(generator.toTypeShim("snake"), RouteType::SNAKE);
  EXPECT_EQ(generator.toTypeShim("SPIRAL"), RouteType::SPIRAL);
  EXPECT_EQ(generator.toTypeShim("spiral"), RouteType::SPIRAL);
  EXPECT_EQ(generator.toTypeShim("CUSTOM"), RouteType::CUSTOM);
  EXPECT_EQ(generator.toTypeShim("custom"), RouteType::CUSTOM);

  EXPECT_EQ(generator.toStringShim(RouteType::UNKNOWN), std::string("Unknown"));
  EXPECT_EQ(generator.toStringShim(RouteType::BOUSTROPHEDON), std::string("Boustrophedon"));
  EXPECT_EQ(generator.toStringShim(RouteType::SNAKE), std::string("Snake"));
  EXPECT_EQ(generator.toStringShim(RouteType::SPIRAL), std::string("Spiral"));
  EXPECT_EQ(generator.toStringShim(RouteType::CUSTOM), std::string("Custom"));
  // B1-T11: TSP toType/toString
  EXPECT_EQ(generator.toTypeShim("TSP"), RouteType::TSP);
  EXPECT_EQ(generator.toTypeShim("tsp"), RouteType::TSP);
  EXPECT_EQ(generator.toStringShim(RouteType::TSP), std::string("TSP"));

  EXPECT_TRUE(generator.createGeneratorShim(RouteType::BOUSTROPHEDON));
  EXPECT_TRUE(generator.createGeneratorShim(RouteType::SNAKE));
  EXPECT_TRUE(generator.createGeneratorShim(RouteType::SPIRAL));
  EXPECT_TRUE(generator.createGeneratorShim(RouteType::CUSTOM));
  EXPECT_FALSE(generator.createGeneratorShim(RouteType::UNKNOWN));
  // TSP is now a valid RouteMethod (TspRouteMethod adapting RoutePlannerBase)
  EXPECT_TRUE(generator.createGeneratorShim(RouteType::TSP));

  generator.setMode("a mode");
  generator.setSpiralN(10);
  generator.setCustomOrder(std::vector<long int>{});  // NOLINT
}

TEST(RouteTests, TestrouteGeneration)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  RouteShim generator(node);

  // Generate some toy field
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::RouteMode settings;
  auto route1 = generator.generateRoute(cells, sbc, settings);
  settings.mode = "BOUSTROPHEDON";
  auto route2 = generator.generateRoute(cells, sbc, settings);
  settings.mode = "SPIRAL";
  auto route3 = generator.generateRoute(cells, sbc, settings);

  // Throws since custom order is set to emptry set
  settings.mode = "CUSTOM";
  EXPECT_THROW(generator.generateRoute(cells, sbc, settings), std::length_error);
}

TEST(RouteTests, TestTSPGeneration)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  // Generate a two-cell field for TSP (B1-T1: non-empty route, B1-T2: coverage, B1-T3: connections)
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));

  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "TSP";
  settings.tsp_redirect_swaths = true;
  settings.tsp_time_limit = 1;
  settings.tsp_search_for_optimum = false;
  settings.tsp_d_tol = 1e-4;

  // B1-T1: route is non-empty
  F2CRoute route = generator.generateRoute(cells, sbc, settings);
  EXPECT_FALSE(route.isEmpty());
  EXPECT_GE(route.sizeVectorSwaths(), 1u);
  EXPECT_GT(route.length(), 0.0);

  // B1-T2: swath count is preserved
  size_t total_in = sbc.sizeTotal();
  size_t total_out = 0;
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    total_out += route.getVectorSwaths()[i].size();
  }
  EXPECT_EQ(total_in, total_out);

  // B1-T3: connection count is consistent (>= 0, asLineString non-empty)
  EXPECT_GE(route.sizeConnections(), 0u);
  EXPECT_GT(route.asLineString().size(), 0u);
}

TEST(RouteTests, TestTSPSingleCell)
{
  // B1-T4: single cell input should not crash
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));

  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "TSP";

  F2CRoute route = generator.generateRoute(cells, sbc, settings);
  EXPECT_FALSE(route.isEmpty());
}

TEST(RouteTests, TestSwathOrderWrappedAsRoute)
{
  // Orderer modes return their sorted swaths wrapped in a single-group F2CRoute;
  // check the wrap keeps one group and preserves the swath count.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));

  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "BOUSTROPHEDON";
  F2CRoute route = generator.generateRoute(cells, sbc, settings);

  EXPECT_FALSE(route.isEmpty());
  EXPECT_EQ(route.sizeVectorSwaths(), 1u);

  size_t total_out = 0;
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    total_out += route.getVectorSwaths()[i].size();
  }
  EXPECT_EQ(sbc.sizeTotal(), total_out);
}

TEST(RouteTests, TestTSPStartPointHonored)
{
  // On the global route path, the route starts and returns at the given point
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeSquareCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "TSP";

  F2CPoint start(0.0, 0.0);
  F2CRoute route = generator.generateRoute(cells, sbc, settings, start);
  EXPECT_FALSE(route.isEmpty());
  EXPECT_NEAR(route.startPoint().getX(), start.getX(), 1e-2);
  EXPECT_NEAR(route.startPoint().getY(), start.getY(), 1e-2);
  EXPECT_NEAR(route.endPoint().getX(), start.getX(), 1e-2);
  EXPECT_NEAR(route.endPoint().getY(), start.getY(), 1e-2);
}

TEST(RouteTests, TestTSPNoStartPointRegression)
{
  // No start point still produces a valid route
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeSquareCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "TSP";
  F2CRoute route = generator.generateRoute(cells, sbc, settings);
  EXPECT_FALSE(route.isEmpty());
}

TEST(RouteTests, TestNonTSPStartPointIgnored)
{
  // Orderer modes ignore the start point, producing an identical route
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeSquareCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "BOUSTROPHEDON";
  F2CRoute without = generator.generateRoute(cells, sbc, settings);
  F2CRoute with = generator.generateRoute(cells, sbc, settings, F2CPoint(0.0, 0.0));

  EXPECT_EQ(without.asLineString().size(), with.asLineString().size());
  EXPECT_NEAR(without.length(), with.length(), 1e-6);
}

TEST(RouteTests, TestTSPMultiCellStitch)
{
  // Two cells route per-cell and stitch: every cell stays contiguous, swaths are
  // preserved, and at least one inter-cell connection (bridge) is produced.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeTwoAdjacentCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);
  ASSERT_GE(sbc.size(), 2u);  // one swath group per cell

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "TSP";
  settings.tsp_time_limit = 1;
  F2CRoute route = generator.generateRoute(cells, sbc, settings);

  EXPECT_FALSE(route.isEmpty());
  EXPECT_GE(route.sizeVectorSwaths(), 2u);
  EXPECT_GE(route.sizeConnections(), 1u);

  size_t total_out = 0;
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    total_out += route.getVectorSwaths()[i].size();
  }
  EXPECT_EQ(sbc.sizeTotal(), total_out);
}

TEST(RouteTests, TestTSPMultiCellStartPoint)
{
  // On the multi-cell path the exact start point isn't honored (the nearest cell
  // is chosen and warned), but the route stays valid.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeTwoAdjacentCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "TSP";
  settings.tsp_time_limit = 1;
  F2CRoute route = generator.generateRoute(cells, sbc, settings, F2CPoint(0.0, 0.0));
  EXPECT_FALSE(route.isEmpty());
}

TEST(RouteTests, TestSwathOrderMultiCellNotInterleaved)
{
  // Regression: non-TSP multi-cell must finish one cell before starting the
  // other. Membership is geometric (midpoint side of x=s), not swath id,
  // since ids are per-cell and collide across cells.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  const double s = 100.0;
  F2CCells cells = makeTwoAdjacentCells(s);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);
  ASSERT_GE(sbc.size(), 2u);  // one swath group per cell

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "BOUSTROPHEDON";
  F2CRoute route = generator.generateRoute(cells, sbc, settings);

  EXPECT_FALSE(route.isEmpty());
  EXPECT_GE(route.sizeVectorSwaths(), 2u);

  std::vector<int> sides;
  size_t total_out = 0;
  for (size_t g = 0; g < route.sizeVectorSwaths(); ++g) {
    for (const auto & swath : route.getVectorSwaths()[g]) {
      const double mid_x = (swath.startPoint().getX() + swath.endPoint().getX()) / 2.0;
      sides.push_back(mid_x < s ? 0 : 1);
      ++total_out;
    }
  }
  EXPECT_EQ(sbc.sizeTotal(), total_out);

  // Both cells must actually appear, or the no-crossing-back check below is vacuous.
  ASSERT_TRUE(
    std::any_of(sides.begin(), sides.end(), [](int v) {return v == 0;}) &&
    std::any_of(sides.begin(), sides.end(), [](int v) {return v == 1;}));

  // Once the route crosses to the other cell, it must never cross back.
  size_t switch_idx = sides.size();
  for (size_t i = 1; i < sides.size(); ++i) {
    if (sides[i] != sides[0]) {
      switch_idx = i;
      break;
    }
  }
  for (size_t i = switch_idx; i < sides.size(); ++i) {
    EXPECT_EQ(sides[i], sides[switch_idx]) << "cells interleaved at swath index " << i;
  }
}

TEST(RouteTests, TestSwathOrderMultiCellBridged)
{
  // Multi-cell non-TSP must bridge cells via the stitch layer, not leave a gap.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeTwoAdjacentCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "BOUSTROPHEDON";
  F2CRoute route = generator.generateRoute(cells, sbc, settings);

  bool has_bridge = false;
  for (const auto & conn : route.getConnections()) {
    if (conn.size() > 0) {
      has_bridge = true;
      break;
    }
  }
  EXPECT_TRUE(has_bridge);
}

TEST(RouteTests, TestSwathOrderMultiCellModes)
{
  // SNAKE and SPIRAL must also plan multi-cell without throwing, like BOUSTROPHEDON.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeTwoAdjacentCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "SNAKE";
  F2CRoute snake_route = generator.generateRoute(cells, sbc, settings);
  EXPECT_FALSE(snake_route.isEmpty());

  settings.mode = "SPIRAL";
  settings.spiral_n = 2;
  F2CRoute spiral_route = generator.generateRoute(cells, sbc, settings);
  EXPECT_FALSE(spiral_route.isEmpty());
}

TEST(RouteTests, TestCustomOrderMultiCellRejected)
{
  // CUSTOM's order vector can't be split across cells; multi-cell must throw.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeTwoAdjacentCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "CUSTOM";
  EXPECT_THROW(generator.generateRoute(cells, sbc, settings), CoverageException);
}

TEST(RouteTests, TestSwathOrderSingleCellUnchanged)
{
  // Guards the single-cell fast path: still one group with the full swath count.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot_params(node);
  SwathGenerator swath_gen(node, &robot_params);
  RouteShim generator(node);

  F2CCells cells = makeSquareCells(100.0);
  opennav_coverage_msgs::msg::SwathMode sw_settings;
  F2CSwathsByCells sbc = swath_gen.generateSwathsByCells(cells, sw_settings);

  opennav_coverage_msgs::msg::RouteMode settings;
  settings.mode = "BOUSTROPHEDON";
  F2CRoute route = generator.generateRoute(cells, sbc, settings);

  EXPECT_EQ(route.sizeVectorSwaths(), 1u);
  EXPECT_EQ(route.getVectorSwaths()[0].size(), sbc.sizeTotal());
}

}  // namespace opennav_coverage
