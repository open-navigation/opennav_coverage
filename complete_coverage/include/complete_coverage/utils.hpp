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

#ifndef COMPLETE_COVERAGE__UTILS_HPP_
#define COMPLETE_COVERAGE__UTILS_HPP_

#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace complete_coverage
{

inline void toUpper(const std::string & string)
{
  std::transform(string.begin(), string.end(), string.begin(), ::toupper);
}

}  // namespace complete_coverage

#endif  // COMPLETE_COVERAGE__UTILS_HPP_
