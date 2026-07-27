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
  auto field = rand.generateRandField(1e5, 5);

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::HeadlandMode settings;
  auto field_out = generator.generateHeadlands(field.getField().getGeometry(0), settings);
  settings.mode = "CONSTANT";
  auto field_out2 = generator.generateHeadlands(field.getField().getGeometry(0), settings);
}

TEST(HeadlandTests, TestheadlandGenerationMultiCell)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  Field cell = field.getField().getGeometry(0);
  double area_in = cell.area();

  // 2-cell F2CCells
  F2CCells cells;
  cells.addGeometry(cell);
  cells.addGeometry(cell);

  opennav_coverage_msgs::msg::HeadlandMode settings;
  F2CCells result = generator.generateHeadlands(cells, settings);

  EXPECT_EQ(result.size(), 2u);
  // Each cell should have shrunk (headland removed)
  EXPECT_LT(result.getGeometry(0).area(), area_in);
  EXPECT_LT(result.getGeometry(1).area(), area_in);
}

TEST(HeadlandTests, TestheadlandMultiCellCollapseSkipped)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  // Big cell (100x100) survives the default 2 m inward buffer; the 3 m-wide strip
  // collapses to an empty geometry. The collapsed cell must be skipped rather than
  // dereferenced (which would throw "Geometry does not contain point 0").
  F2CCell big(F2CLinearRing({
      F2CPoint(0, 0), F2CPoint(100, 0), F2CPoint(100, 100),
      F2CPoint(0, 100), F2CPoint(0, 0)}));
  F2CCell thin(F2CLinearRing({
      F2CPoint(200, 0), F2CPoint(203, 0), F2CPoint(203, 100),
      F2CPoint(200, 100), F2CPoint(200, 0)}));
  F2CCells cells;
  cells.addGeometry(big);
  cells.addGeometry(thin);

  opennav_coverage_msgs::msg::HeadlandMode settings;  // default width 2.0 m
  F2CCells result = generator.generateHeadlands(cells, settings);

  EXPECT_EQ(result.size(), 1u);
  EXPECT_GT(result.getGeometry(0).area(), 0.0);
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
  // Only x=50 is carved (~48x100 each): total > full-border-buffer case (8832)
  // but < the original (10000).
  EXPECT_GT(result.area(), 9000.0);
  EXPECT_LT(result.area(), 9999.0);

  // Width larger than the cells collapses everything -> throws.
  EXPECT_THROW(generator.generateHeadlandsBetweenCells(cells, 60.0), CoverageException);

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

TEST(HeadlandTests, TestheadlandMultiCellAllCollapseThrows)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  // Every sub-cell is a 3 m-wide strip that collapses under the 2 m buffer, so the
  // result is empty and a CoverageException is thrown instead of returning nothing.
  F2CCell thin_a(F2CLinearRing({
      F2CPoint(0, 0), F2CPoint(3, 0), F2CPoint(3, 100), F2CPoint(0, 100), F2CPoint(0, 0)}));
  F2CCell thin_b(F2CLinearRing({
      F2CPoint(50, 0), F2CPoint(53, 0), F2CPoint(53, 100), F2CPoint(50, 100), F2CPoint(50, 0)}));
  F2CCells cells;
  cells.addGeometry(thin_a);
  cells.addGeometry(thin_b);

  opennav_coverage_msgs::msg::HeadlandMode settings;  // default width 2.0 m
  EXPECT_THROW(generator.generateHeadlands(cells, settings), CoverageException);
}

}  // namespace opennav_coverage
