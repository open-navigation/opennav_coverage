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
#include "opennav_row_coverage/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opennav_row_coverage/row_swath_generator.hpp"
#include "opennav_coverage_msgs/msg/row_swath_mode.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_row_coverage
{

class SwathTestFixture : public ::testing::Test
{
public:
  void SetUp()
  {
    node_ = std::make_unique<rclcpp::Node>("test_node");
    generator_ = std::make_unique<RowSwathGenerator>(node_);

    const std::string file_path =
      ament_index_cpp::get_package_share_directory("opennav_coverage") +
      "/cartesian_test_field.xml";
    rows_ = opennav_row_coverage::util::parseRows(file_path);
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RowSwathGenerator> generator_;
  F2CField field_;
  Rows rows_;
};

TEST_F(SwathTestFixture, TestCenter)
{
  opennav_coverage_msgs::msg::RowSwathMode settings;
  settings.mode = "CENTER";

  auto swaths = generator_->generateSwaths(rows_, settings);

  ASSERT_EQ(swaths.size(), 6);
  EXPECT_EQ(swaths[0].getId(), 1);
  EXPECT_EQ(swaths[0].getPath().getX(0), 6.475);
  EXPECT_EQ(swaths[0].getPath().getY(0), 5.0);
  EXPECT_EQ(swaths[0].getPath().getX(1), 6.13);
  EXPECT_EQ(swaths[0].getPath().getY(1), 15.0);
}

TEST_F(SwathTestFixture, TestCenterWithSkipping)
{
  opennav_coverage_msgs::msg::RowSwathMode settings;
  settings.mode = "CENTER";
  settings.skip_ids = {1, 10};

  auto swaths = generator_->generateSwaths(rows_, settings);
  ASSERT_EQ(swaths.size(), 5);
  EXPECT_EQ(swaths[0].getId(), 2);
  EXPECT_NEAR(swaths[0].getPath().getX(0), 8.04, 1e-3);
  EXPECT_EQ(swaths[0].getPath().getY(0), 5.0);
  EXPECT_NEAR(swaths[0].getPath().getX(1), 8.03, 1e-3);
  EXPECT_EQ(swaths[0].getPath().getY(1), 15.0);
}

TEST_F(SwathTestFixture, TestOffset)
{
  opennav_coverage_msgs::msg::RowSwathMode settings;
  settings.mode = "OFFSET";
  settings.offset = 6.0;
  settings.skip_ids = {1, 10};

  auto swaths = generator_->generateSwaths(rows_, settings);

  ASSERT_EQ(swaths.size(), 5);
  EXPECT_EQ(swaths[0].getId(), 2);
  EXPECT_NEAR(swaths[0].getPath().getX(0), 13.699, 1e-3);
  EXPECT_EQ(swaths[0].getPath().getY(0), 5.0);
  EXPECT_NEAR(swaths[0].getPath().getX(1), 12.76, 1e-3);
  EXPECT_EQ(swaths[0].getPath().getY(1), 15.0);
}

TEST_F(SwathTestFixture, TestRowsAreSwaths)
{
  opennav_coverage_msgs::msg::RowSwathMode settings;
  settings.mode = "ROWSARESWATHS";
  settings.skip_ids = {1, 2};

  auto swaths = generator_->generateSwaths(rows_, settings);

  EXPECT_EQ(swaths.size(), 5);

  EXPECT_NEAR(swaths[0].getPath().getX(0), rows_[2].first.getX(0), 1e-3);
  EXPECT_NEAR(swaths[0].getPath().getY(0), rows_[2].first.getY(0), 1e-3);

  EXPECT_NEAR(swaths[0].getPath().getX(1), rows_[2].first.getX(1), 1e-3);
  EXPECT_NEAR(swaths[0].getPath().getY(1), rows_[2].first.getY(1), 1e-3);
  EXPECT_NEAR(swaths[0].getWidth(), 1.0699, 1e-3);

  EXPECT_EQ(swaths[0].getId(), 3);
}

}  // namespace opennav_row_coverage
