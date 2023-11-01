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

  EXPECT_TRUE(generator.createGeneratorShim(RouteType::BOUSTROPHEDON));
  EXPECT_TRUE(generator.createGeneratorShim(RouteType::SNAKE));
  EXPECT_TRUE(generator.createGeneratorShim(RouteType::SPIRAL));
  EXPECT_TRUE(generator.createGeneratorShim(RouteType::CUSTOM));
  EXPECT_FALSE(generator.createGeneratorShim(RouteType::UNKNOWN));

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
  auto field = rand.generateRandField(5, 1e5);
  auto swaths = swath_gen.generateSwaths(field.field.getGeometry(0), sw_settings);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::RouteMode settings;
  auto route1 = generator.generateRoute(swaths, settings);
  settings.mode = "BOUSTROPHEDON";
  auto route2 = generator.generateRoute(swaths, settings);
  settings.mode = "SPIRAL";
  auto route3 = generator.generateRoute(swaths, settings);

  // Throws since custom order is set to emptry set
  settings.mode = "CUSTOM";
  EXPECT_THROW(generator.generateRoute(swaths, settings), std::length_error);
}

}  // namespace opennav_coverage
