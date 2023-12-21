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

#ifndef OPENNAV_COVERAGE__TYPES_HPP_
#define OPENNAV_COVERAGE__TYPES_HPP_

#include <vector>
#include <string>
#include <memory>

#include "fields2cover.h" // NOLINT

#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"

namespace opennav_coverage
{

typedef F2CCells Fields;
typedef F2CCell Field;
typedef F2CSwaths Swaths;
typedef F2CSwath Swath;
typedef F2CPath Path;
typedef F2CRobot Robot;
typedef F2CLinearRing Polygon;
typedef F2CPoint Point;
typedef f2c::types::PathState PathState;
typedef f2c::types::LineString LineString;
typedef F2CLineString LineString;

typedef std::shared_ptr<f2c::hg::HeadlandGeneratorBase> HeadlandGeneratorPtr;
typedef std::shared_ptr<f2c::obj::SGObjective> SwathObjectivePtr;
typedef std::shared_ptr<f2c::pp::TurningBase> TurningBasePtr;
typedef std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> RouteGeneratorPtr;

typedef opennav_coverage_msgs::action::ComputeCoveragePath ComputeCoveragePath;

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

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__TYPES_HPP_
