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

#include <utility>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_row_coverage/utils.hpp"
#include "tf2/utils.h"
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

TEST(UtilsTests, TestRowParserSimple)
{
  const std::string file_path =
    ament_index_cpp::get_package_share_directory("opennav_coverage") +
    "/cartesian_test_field.xml";
  auto rows = opennav_row_coverage::util::parseRows(file_path);
  ASSERT_EQ(rows.size(), 7);
}

TEST(UtilsTests, TestRowParserComplex)
{
  const std::string file_path =
    ament_index_cpp::get_package_share_directory("opennav_coverage") +
    "/irregular_test_field.xml";
  auto rows = opennav_row_coverage::util::parseRows(file_path);

  ASSERT_EQ(rows.size(), 10);

  EXPECT_EQ(rows[2].first.getX(0), 4.26122335903712);
  EXPECT_EQ(rows[2].first.getY(0), 51.7859542010475);
  EXPECT_EQ(rows[2].second, 3);

  EXPECT_EQ(rows[4].first.getX(1), 4.25994305603656);
  EXPECT_EQ(rows[4].first.getY(1), 51.7899570320785);
  EXPECT_EQ(rows[4].second, 5);

  EXPECT_EQ(rows[8].first.getX(1), 4.25751883150527);
  EXPECT_EQ(rows[8].first.getY(1), 51.7903779047924);
  EXPECT_EQ(rows[8].second, 9);

  EXPECT_EQ(rows[9].first.getX(0), 4.25644928223945);
  EXPECT_EQ(rows[9].first.getY(0), 51.7894578087076);
  EXPECT_EQ(rows[9].second, 10);
}

}  // namespace opennav_row_coverage
