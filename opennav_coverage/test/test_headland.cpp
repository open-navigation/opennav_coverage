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
#include "opennav_coverage/headland_generator.hpp"
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

class HeadlandShim : public HeadlandGenerator
{
public:
  template<typename NodeT>
  explicit HeadlandShim(const NodeT & node)
  : HeadlandGenerator(node)
  {}

  HeadlandGeneratorPtr createGeneratorShim(const HeadlandType & type)
  {
    return createGenerator(type);
  }

  std::string toStringShim(const HeadlandType & type)
  {
    return toString(type);
  }

  HeadlandType toTypeShim(const std::string & str)
  {
    return toType(str);
  }
};

TEST(HeadlandTests, TestheadlandUtils)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  EXPECT_EQ(generator.toStringShim(HeadlandType::UNKNOWN), std::string("Unknown"));
  EXPECT_EQ(generator.toStringShim(HeadlandType::CONSTANT), std::string("Constant"));

  EXPECT_EQ(generator.toTypeShim("FAKE"), HeadlandType::UNKNOWN);
  EXPECT_EQ(generator.toTypeShim("constant"), HeadlandType::CONSTANT);
  EXPECT_EQ(generator.toTypeShim("CONSTANT"), HeadlandType::CONSTANT);

  EXPECT_TRUE(generator.createGeneratorShim(HeadlandType::CONSTANT));
  EXPECT_FALSE(generator.createGeneratorShim(HeadlandType::UNKNOWN));

  generator.setMode("constant");
}

TEST(HeadlandTests, TestheadlandGeneration)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  // Generate some toy field
  f2c::Random rand;
  auto field = rand.generateRandField(5, 1e5);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::HeadlandMode settings;
  auto field_out = generator.generateHeadlands(field.field.getGeometry(0), settings);
  settings.mode = "CONSTANT";
  auto field_out2 = generator.generateHeadlands(field.field.getGeometry(0), settings);
}

}  // namespace opennav_coverage
