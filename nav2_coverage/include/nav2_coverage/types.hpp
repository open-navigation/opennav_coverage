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

#ifndef NAV2_COVERAGE__TYPES_HPP_
#define NAV2_COVERAGE__TYPES_HPP_

#include <vector>
#include <string>
#include <memory>

namespace nav2_coverage
{

typedef F2CCells Fields;
typedef F2CCell Field;
typedef F2CSwaths Swaths;
typedef F2CPath Path;
typedef F2CRobot Robot;

typedef std::shared_ptr<f2c::hg::HeadlandGeneratorBase> HeadlandGeneratorPtr;
typedef std::shared_ptr<f2c::obj::SGObjective> SwathObjectivePtr;
typedef std::shared_ptr<f2c::pp::TurningBase> TurningBasePtr;
typedef std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> RouteGeneratorPtr;

/**
 * @enum Headland computations types
 */
enum class HeadlandType
{
  UNKNOWN = 0,
  CONSTANT = 1
};

/**
 * @enum Swath computations types
 */
enum class SwathType
{
  UNKNOWN = 0,
  LENGTH = 1,
  NUMBER = 2,
  COVERAGE = 3
};

/**
 * @enum Swath angle types
 */
enum class SwathAngleType
{
  UNKNOWN = 0,
  SET_ANGLE = 1,
  BRUTE_FORCE = 2
};

/**
 * @enum Swath computations types
 */
enum class RouteType
{
  UNKNOWN = 0,
  BOUSTROPHEDON = 1,
  SNAKE = 2,
  SPIRAL = 3,
  CUSTOM = 4
};

/**
 * @enum Path computations types
 */
enum class PathType
{
  UNKNOWN = 0,
  REEDS_SHEPP = 1,
  DUBIN = 2,
};

/**
 * @enum Path continuity types
 */
enum class PathContinuityType
{
  UNKNOWN = 0,
  CONTINUOUS = 1,
  DISCONTINUOUS = 2
};

class CoverageException : public std::runtime_error
{
public:
  explicit CoverageException(const std::string & description)
  : std::runtime_error(description) {}
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__TYPES_HPP_
