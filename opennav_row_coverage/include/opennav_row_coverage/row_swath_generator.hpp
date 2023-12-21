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

#ifndef OPENNAV_ROW_COVERAGE__ROW_SWATH_GENERATOR_HPP_
#define OPENNAV_ROW_COVERAGE__ROW_SWATH_GENERATOR_HPP_


#include <vector>
#include <string>
#include <algorithm>

#include "opennav_row_coverage/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_coverage/utils.hpp"
#include "opennav_row_coverage/utils.hpp"

namespace opennav_row_coverage
{

using namespace opennav_coverage;  // NOLINT

/**
  * @class An object to contain the relevent implementations of the swath generation policies
  * to be called within the swath row generator object with the appropriate parameters
  */
class SwathFactory
{
public:
  /**
   * @brief Computes swaths in the center of the marked rows
   * @param rows Rows to generate swaths from
   * @param ids_to_skip The ids to skip
   */
  Swaths generateCenterSwaths(const Rows & rows, const SkipIDs & ids_to_skip);

  /**
   * @brief Computes swaths offset of the marked rows by a set amount
   * @param rows Rows to generate swaths from
   * @param offset Offset to use w.r.t. row
   * @return Swaths The generated swaths
   */
  Swaths generateOffsetSwaths(const Rows & rows, const SkipIDs & ids_to_skip, float offset);

  /**
   * @brief Create Swaths directly from the rows
   * @param rows Rows to generate swaths from
   * @param ids The ids to skip
   * @return Swaths The generated swaths
   */
  Swaths generateRowsAreSwaths(const Rows & rows, const SkipIDs & ids_to_skip);

private:
  /**
   * @brief Determine if a id should be skipped
   * @param ids_to_skip The ids to skip
   * @param id The id to check
   * @return true If the id is in the ids_to_skip list
   */
  bool skipId(const SkipIDs & ids_to_skip, int id);

  /**
   * @brief Calculate the width between two rows
   * @param row1 The first row
   * @param row2 The second row
   * @return double The width
   */
  double calculateWidth(const LineString & row1, const LineString & row2);
};

/**
  * @class An object to compute swaths given a set of pre-computed or defined rows
  * based on a policy about what the rows represent (e.g. if rows are crop, it should select
  * if we should use the center of the rows, use an offset, if the rows themselves
  * are the objective, etc)
  */
class RowSwathGenerator
{
public:
  /**
   * @brief Constructor for swath generator
   * @param node A node to get the swath type from
   */
  template<typename NodeT>
  explicit RowSwathGenerator(const NodeT & node)
  {
    logger_ = node->get_logger();
    nav2_util::declare_parameter_if_not_declared(
      node, "default_swath_type", rclcpp::ParameterValue("CENTER"));
    std::string type_str = node->get_parameter("default_swath_type").as_string();
    default_type_ = toType(type_str);

    nav2_util::declare_parameter_if_not_declared(
      node, "default_offset", rclcpp::ParameterValue(0.5));
    default_offset_ = static_cast<float>(node->get_parameter("default_offset").as_double());
  }

  /**
   * @brief Main method to generate the swaths
   * @param rows The rows of obstacles in the field
   * @param settings Action requested settings to use
   * @return Swaths The generated swaths
   */
  Swaths generateSwaths(
    const Rows & rows,
    const opennav_coverage_msgs::msg::RowSwathMode & settings);

  /**
   * @brief Sets the mode manually of the swath mode for dynamic parameters
   * @param mode String for mode to use
   */
  void setMode(const std::string & new_mode);

  /**
   * @brief Sets the offset manually for dynamic parameters
   * @param offset Offset to use
   */
  void setOffset(const float offset) {default_offset_ = offset;}

private:
  /**
   * @brief Convert the swath type to a string
   * @param swath_type The type of swath to generate
   * @return std::string The swath type as a string
   */
  std::string toString(const RowSwathType & swath_type);

  /**
   * @brief Convert a string to a swath type
   * @param str The string to convert
   * @return RowSwathType The swath type
   */
  RowSwathType toType(const std::string & str);

  /**
   * @brief Manually correct annotated row orientations for consistent operations
   * @param rows The rows of obstacles in the field
   * @return Rows with orientations made consistent
  */
  Rows adjustRowOrientations(const Rows & rows);

  SwathFactory swath_factory_;
  RowSwathType default_type_;
  float default_offset_;

  rclcpp::Logger logger_{rclcpp::get_logger("SwathGenerator")};
};

}  // namespace opennav_row_coverage

#endif  // OPENNAV_ROW_COVERAGE__ROW_SWATH_GENERATOR_HPP_
