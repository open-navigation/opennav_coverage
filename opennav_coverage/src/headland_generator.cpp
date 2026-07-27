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

#include <vector>
#include <string>

#include "opennav_coverage/headland_generator.hpp"

namespace opennav_coverage
{

HeadlandGeneratorPtr HeadlandGenerator::resolveGenerator(
  const opennav_coverage_msgs::msg::HeadlandMode & settings, double & width)
{
  HeadlandType action_type = toType(settings.mode);
  HeadlandGeneratorPtr generator{nullptr};

  // If not set by action, use default mode
  if (action_type == HeadlandType::UNKNOWN) {
    action_type = default_type_;
    generator = default_generator_;
    width = default_headland_width_;
  } else {
    generator = createGenerator(action_type);
    width = settings.width;
  }

  if (!generator) {
    throw CoverageException("No valid headlands mode set! Options: CONSTANT.");
  }

  RCLCPP_DEBUG(
    logger_, "Generating Headland with generator: %s", toString(action_type).c_str());
  return generator;
}

Field HeadlandGenerator::generateHeadlands(
  const Field & field, const opennav_coverage_msgs::msg::HeadlandMode & settings)
{
  double width = 0.0;
  HeadlandGeneratorPtr generator = resolveGenerator(settings, width);
  return generator->generateHeadlands(Fields(field), width).getGeometry(0);
}

F2CCells HeadlandGenerator::generateHeadlands(
  const F2CCells & cells, const opennav_coverage_msgs::msg::HeadlandMode & settings)
{
  double width = 0.0;
  HeadlandGeneratorPtr generator = resolveGenerator(settings, width);

  // Apply the headland to each sub-cell independently: decomposed cells share
  // borders, so buffering the whole F2CCells at once would only shrink the outer
  // boundary and drop the inter-cell headlands.
  F2CCells result;
  for (size_t i = 0; i < cells.size(); ++i) {
    // Skip sub-cells that the inward buffer collapses to empty; keep every polygon
    // a buffer may split one cell into.
    F2CCells cell_headland = generator->generateHeadlands(Fields(cells.getGeometry(i)), width);
    for (size_t j = 0; j < cell_headland.size(); ++j) {
      result.addGeometry(cell_headland.getGeometry(j));
    }
  }

  if (result.size() == 0) {
    throw CoverageException(
            "Headland width is too large for the decomposed field: every sub-cell "
            "collapsed. Reduce the headland width or disable decomposition.");
  }
  return result;
}

F2CCells HeadlandGenerator::generateHeadlandsBetweenCells(
  const F2CCells & cells, double route_width)
{
  if (route_width <= 0.0) {
    throw CoverageException("route_headland_width must be > 0 to carve inter-cell corridors.");
  }

  // kMinCellArea drops slivers left by carving: F2C's unchecked geometry casts
  // corrupt memory on degenerate (near-zero-area) polygons.
  const double kSharedTol = 1e-3;
  const double kMinCellArea = 1e-3;

  F2CCells result;
  for (size_t i = 0; i < cells.size(); ++i) {
    F2CCells swath_cell;
    swath_cell.addGeometry(cells.getGeometry(i));
    const Polygon ring_i = cells.getCellBorder(i);

    for (size_t k = 0; k < cells.size(); ++k) {
      if (k == i) {
        continue;
      }
      const Polygon ring_k = cells.getCellBorder(k);
      // Carve only the edges of cell_i that lie on cell_k's border: corridors
      // exist strictly between adjacent cells, never on headland or void edges.
      for (size_t e = 0; e + 1 < ring_i.size(); ++e) {
        const Point a = ring_i.getGeometry(e);
        const Point b = ring_i.getGeometry(e + 1);
        const Point mid = (a + b) * 0.5;
        if (ring_k.closestPointTo(a).distance(a) < kSharedTol &&
          ring_k.closestPointTo(b).distance(b) < kSharedTol &&
          ring_k.closestPointTo(mid).distance(mid) < kSharedTol)
        {
          swath_cell = swath_cell.difference(
            Field::buffer(LineString(a, b), route_width));
        }
      }
    }

    for (size_t j = 0; j < swath_cell.size(); ++j) {
      if (swath_cell.getGeometry(j).area() > kMinCellArea) {
        result.addGeometry(swath_cell.getGeometry(j));
      }
    }
  }

  if (result.size() == 0) {
    throw CoverageException(
            "route_headland_width is too large: every sub-cell collapsed. Reduce it.");
  }
  return result;
}

void HeadlandGenerator::setMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
  default_generator_ = createGenerator(default_type_);
}

HeadlandGeneratorPtr HeadlandGenerator::createGenerator(const HeadlandType & type)
{
  switch (type) {
    case HeadlandType::CONSTANT:
      return std::move(std::make_shared<f2c::hg::ConstHL>());
    default:
      RCLCPP_WARN(logger_, "Unknown headland type set!");
      return HeadlandGeneratorPtr{nullptr};
  }
}

std::string HeadlandGenerator::toString(const HeadlandType & type)
{
  switch (type) {
    case HeadlandType::CONSTANT:
      return "Constant";
    default:
      return "Unknown";
  }
}

HeadlandType HeadlandGenerator::toType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "CONSTANT") {
    return HeadlandType::CONSTANT;
  } else {
    return HeadlandType::UNKNOWN;
  }
}

}  // namespace opennav_coverage
