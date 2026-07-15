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
#include "opennav_coverage/decomp_generator.hpp"
#include "opennav_coverage_msgs/msg/decomp_mode.hpp"
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

using opennav_coverage_msgs::msg::DecompMode;

class DecompShim : public DecompGenerator
{
public:
  template<typename NodeT>
  explicit DecompShim(const NodeT & node)
  : DecompGenerator(node)
  {}

  std::string toStringShim(const DecompType & type)
  {
    return toString(type);
  }

  DecompType toTypeShim(const std::string & str)
  {
    return toType(str);
  }
};

TEST(DecompTests, TestDecompUtils)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);

  // toType: unknown -> UNKNOWN (not a throw)
  EXPECT_EQ(gen.toTypeShim("FAKE"), DecompType::UNKNOWN);
  EXPECT_EQ(gen.toTypeShim("UNKNOWN"), DecompType::UNKNOWN);

  // toType: case-insensitivity
  EXPECT_EQ(gen.toTypeShim("NONE"), DecompType::NONE);
  EXPECT_EQ(gen.toTypeShim("none"), DecompType::NONE);
  EXPECT_EQ(gen.toTypeShim("TRAPEZOIDAL"), DecompType::TRAPEZOIDAL);
  EXPECT_EQ(gen.toTypeShim("trapezoidal"), DecompType::TRAPEZOIDAL);
  EXPECT_EQ(gen.toTypeShim("BOUSTROPHEDON"), DecompType::BOUSTROPHEDON);
  EXPECT_EQ(gen.toTypeShim("boustrophedon"), DecompType::BOUSTROPHEDON);

  // toString
  EXPECT_EQ(gen.toStringShim(DecompType::NONE), std::string("NONE"));
  EXPECT_EQ(gen.toStringShim(DecompType::TRAPEZOIDAL), std::string("TRAPEZOIDAL"));
  EXPECT_EQ(gen.toStringShim(DecompType::BOUSTROPHEDON), std::string("BOUSTROPHEDON"));
  EXPECT_GT(gen.toStringShim(DecompType::UNKNOWN).size(), 0u);

  // setters should not throw
  gen.setMode("TRAPEZOIDAL");
  gen.setMode("none");
  gen.setSplitAngle(0.5 * M_PI);
}

TEST(DecompTests, TestDecompNone)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));

  DecompMode mode;
  mode.mode = "NONE";
  F2CCells result = gen.decompose(cells, mode);
  EXPECT_EQ(result.size(), 1u);
}

TEST(DecompTests, TestDecompNonConvex)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);

  // Build an L-shaped non-convex polygon by hand
  F2CLinearRing ring;
  ring.addPoint(0, 0);
  ring.addPoint(10, 0);
  ring.addPoint(10, 5);
  ring.addPoint(5, 5);
  ring.addPoint(5, 10);
  ring.addPoint(0, 10);
  ring.addPoint(0, 0);  // closed ring
  F2CCell cell;
  cell.addRing(ring);
  F2CCells cells;
  cells.addGeometry(cell);

  DecompMode mode;
  mode.mode = "TRAPEZOIDAL";
  F2CCells result_trap = gen.decompose(cells, mode);
  EXPECT_GT(result_trap.size(), 1u);  // L-field must split

  // BOUSTROPHEDON extends TrapezoidalDecomp with a merge step, so its exact cell
  // count is F2C-internal (may merge back to one). Only verify it produces valid,
  // non-empty output without throwing.
  mode.mode = "BOUSTROPHEDON";
  F2CCells result_bou = gen.decompose(cells, mode);
  EXPECT_GE(result_bou.size(), 1u);
}

TEST(DecompTests, TestDecompDefaultFallback)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));

  DecompMode mode;  // default mode = "UNKNOWN" -> default_type_ = NONE
  F2CCells result = gen.decompose(cells, mode);
  EXPECT_EQ(result.size(), cells.size());  // behaves like NONE, unchanged
}

}  // namespace opennav_coverage
