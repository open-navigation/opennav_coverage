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

class SwathShim : public SwathGenerator
{
public:
  template<typename NodeT>
  explicit SwathShim(const NodeT & node, RobotParams * robot_params)
  : SwathGenerator(node, robot_params)
  {}

  SwathObjectivePtr createObjectiveShim(const SwathType & type)
  {
    return createObjective(type);
  }

  std::string toStringShim(const SwathType & type, const SwathAngleType & angle_type)
  {
    return toString(type, angle_type);
  }

  SwathType toTypeShim(const std::string & str)
  {
    return toType(str);
  }


  SwathAngleType toAngleTypeShim(const std::string & str)
  {
    return toAngleType(str);
  }
};

TEST(SwathTests, TestswathUtils)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot(node);
  auto generator = SwathShim(node, &robot);

  EXPECT_EQ(generator.toAngleTypeShim("FAKE"), SwathAngleType::UNKNOWN);
  EXPECT_EQ(generator.toAngleTypeShim("SET_ANGLE"), SwathAngleType::SET_ANGLE);
  EXPECT_EQ(generator.toAngleTypeShim("set_angle"), SwathAngleType::SET_ANGLE);
  EXPECT_EQ(generator.toAngleTypeShim("BRUTE_FORCE"), SwathAngleType::BRUTE_FORCE);
  EXPECT_EQ(generator.toAngleTypeShim("brute_force"), SwathAngleType::BRUTE_FORCE);

  EXPECT_EQ(generator.toTypeShim("FAKE"), SwathType::UNKNOWN);
  EXPECT_EQ(generator.toTypeShim("LENGTH"), SwathType::LENGTH);
  EXPECT_EQ(generator.toTypeShim("length"), SwathType::LENGTH);
  EXPECT_EQ(generator.toTypeShim("NUMBER"), SwathType::NUMBER);
  EXPECT_EQ(generator.toTypeShim("number"), SwathType::NUMBER);
  EXPECT_EQ(generator.toTypeShim("COVERAGE"), SwathType::COVERAGE);
  EXPECT_EQ(generator.toTypeShim("coverage"), SwathType::COVERAGE);


  EXPECT_EQ(generator.toStringShim(SwathType::NUMBER, SwathAngleType::SET_ANGLE).size(), 37u);
  EXPECT_EQ(generator.toStringShim(SwathType::COVERAGE, SwathAngleType::BRUTE_FORCE).size(), 41u);
  EXPECT_GT(generator.toStringShim(SwathType::UNKNOWN, SwathAngleType::UNKNOWN).size(), 20u);
  EXPECT_GT(generator.toStringShim(SwathType::UNKNOWN, SwathAngleType::BRUTE_FORCE).size(), 20u);

  EXPECT_TRUE(generator.createObjectiveShim(SwathType::LENGTH));
  EXPECT_TRUE(generator.createObjectiveShim(SwathType::NUMBER));
  EXPECT_TRUE(generator.createObjectiveShim(SwathType::COVERAGE));
  EXPECT_FALSE(generator.createObjectiveShim(SwathType::UNKNOWN));

  generator.setSwathAngleMode("NUMBER");
  generator.setSwathMode("SET_ANGLE");
  generator.setSwathAngle(0.0);
  generator.setOVerlap(false);
  generator.setStepAngle(false);
}

TEST(SwathTests, TestswathGeneration)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot(node);
  auto generator = SwathShim(node, &robot);

  // Generate some toy field
  f2c::Random rand;
  auto field = rand.generateRandField(5, 1e5);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::SwathMode settings;
  auto swaths1 = generator.generateSwaths(field.field.getGeometry(0), settings);
  settings.mode = "BRUTE_FORCE";
  settings.objective = "LENGTH";
  auto swaths2 = generator.generateSwaths(field.field.getGeometry(0), settings);
  settings.mode = "SET_ANGLE";
  settings.objective = "NUMBER";
  auto swaths3 = generator.generateSwaths(field.field.getGeometry(0), settings);
}

}  // namespace opennav_coverage
