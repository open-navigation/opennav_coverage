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

#ifndef OPENNAV_ROW_COVERAGE__TYPES_HPP_
#define OPENNAV_ROW_COVERAGE__TYPES_HPP_

#include <vector>
#include <utility>

#include "opennav_coverage/types.hpp"

namespace opennav_row_coverage
{

typedef std::pair<opennav_coverage::LineString, int> Row;
typedef std::vector<Row> Rows;
typedef std::vector<int> SkipIDs;

enum class RowSwathType
{
  UNKNOWN = 0,
  CENTER = 1,
  OFFSET = 2,
  ROWSARESWATHS = 3
};

}  // namespace opennav_row_coverage

#endif  // OPENNAV_ROW_COVERAGE__TYPES_HPP_
