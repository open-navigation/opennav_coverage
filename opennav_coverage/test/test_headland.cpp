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
  auto field = rand.generateRandField(1e5, 5);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::HeadlandMode settings;
  auto field_out = generator.generateHeadlands(field.getField().getGeometry(0), settings);
  settings.mode = "CONSTANT";
  auto field_out2 = generator.generateHeadlands(field.getField().getGeometry(0), settings);
}

TEST(HeadlandTests, TestheadlandBetweenCells)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  // Two 50x100 cells sharing the x=50 border, inside a 100x100 outer boundary.
  F2CCell left(F2CLinearRing({
      F2CPoint(0, 0), F2CPoint(50, 0), F2CPoint(50, 100), F2CPoint(0, 100), F2CPoint(0, 0)}));
  F2CCell right(F2CLinearRing({
      F2CPoint(50, 0), F2CPoint(100, 0), F2CPoint(100, 100),
      F2CPoint(50, 100), F2CPoint(50, 0)}));
  F2CCells cells;
  cells.addGeometry(left);
  cells.addGeometry(right);

  F2CCells result = generator.generateHeadlandsBetweenCells(cells, 2.0);

  EXPECT_EQ(result.size(), 2u);
  // Only x=50 is carved (49x100 each): total > full-border-buffer case (8832)
  // but < the original (10000).
  EXPECT_GT(result.area(), 9000.0);
  EXPECT_LT(result.area(), 9999.0);

  // Width larger than the cells collapses everything -> throws.
  EXPECT_THROW(generator.generateHeadlandsBetweenCells(cells, 120.0), CoverageException);

  // Cells facing each other across a void share no border, so nothing may be
  // carved (route headland only ever goes between truly adjacent cells).
  F2CCell far_right(F2CLinearRing({
      F2CPoint(60, 0), F2CPoint(110, 0), F2CPoint(110, 100),
      F2CPoint(60, 100), F2CPoint(60, 0)}));
  F2CCells apart;
  apart.addGeometry(left);
  apart.addGeometry(far_right);
  F2CCells untouched = generator.generateHeadlandsBetweenCells(apart, 2.0);
  EXPECT_NEAR(untouched.area(), apart.area(), 1e-3);
}

TEST(HeadlandTests, TestheadlandSwathRings)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  F2CCell cell(F2CLinearRing({
      F2CPoint(0, 0), F2CPoint(100, 0), F2CPoint(100, 100),
      F2CPoint(0, 100), F2CPoint(0, 0)}));

  opennav_coverage_msgs::msg::HeadlandMode settings;  // default width 2.0 m
  // 2.0 / 0.7 -> 3 concentric passes, ordered outer to inner
  auto rings = generator.generateHeadlandSwaths(cell, 0.7, settings);

  EXPECT_EQ(rings.size(), 3u);
  double prev_area = cell.area();
  for (const auto & ring : rings) {
    ASSERT_GT(ring.size(), 0u);
    EXPECT_LT(ring.getGeometry(0).area(), prev_area);
    prev_area = ring.getGeometry(0).area();
  }

  // Invalid operation width must throw rather than divide by zero
  EXPECT_THROW(generator.generateHeadlandSwaths(cell, 0.0, settings), CoverageException);
}

}  // namespace opennav_coverage
