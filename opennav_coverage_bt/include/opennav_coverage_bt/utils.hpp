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

#ifndef OPENNAV_COVERAGE_BT__UTILS_HPP_
#define OPENNAV_COVERAGE_BT__UTILS_HPP_

#include <charconv>
#include "behaviortree_cpp/behavior_tree.h"

namespace BT
{

/**
  * @brief Specialization for unsigned short (uint16)
  * @return unsigned short from BT port
  */
template<>
inline unsigned short convertFromString(const StringView str)  // NOLINT
{
  unsigned short result = 0;  // NOLINT
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc()) {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to unsigned short"));
  }
  return result;
}

}  // namespace BT

#endif  // OPENNAV_COVERAGE_BT__UTILS_HPP_
