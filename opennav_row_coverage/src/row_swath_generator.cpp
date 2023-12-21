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

#include "opennav_row_coverage/row_swath_generator.hpp"
#include "opennav_row_coverage/types.hpp"

namespace opennav_row_coverage
{

Swaths SwathFactory::generateCenterSwaths(const Rows & rows, const SkipIDs & ids_to_skip)
{
  if (rows.size() < 2) {
    throw std::invalid_argument("Rows must be greater than 1");
  }

  Swaths swaths;
  for (unsigned int i = 1; i < rows.size(); ++i) {
    auto & prev_row = rows[i - 1].first;
    auto & cur_row = rows[i].first;

    if (prev_row.size() != cur_row.size() ) {
      throw std::invalid_argument("All rows must be the same size");
    }

    int id = rows[i - 1].second;
    if (skipId(ids_to_skip, id)) {
      continue;
    }

    LineString line_string;
    std::vector<double> widths;
    for (unsigned int j = 0; j < prev_row.size(); ++j) {
      Point prev_point;
      prev_row.getGeometry(j, prev_point);

      Point cur_point;
      cur_row.getGeometry(j, cur_point);

      Point vector = cur_point - prev_point;
      double width = std::sqrt(vector.getX() * vector.getX() + vector.getY() * vector.getY());
      widths.push_back(width);

      Point new_point = (prev_point + cur_point) * 0.5;
      line_string.addPoint(new_point);
    }
    swaths.emplace_back(
      line_string, *std::min_element(
        widths.begin(),
        widths.end()), id);
  }
  return swaths;
}

Swaths SwathFactory::generateRowsAreSwaths(const Rows & rows, const SkipIDs & ids_to_skip)
{
  if (rows.size() < 2) {
    throw std::invalid_argument("Rows must be greater than 1");
  }

  Swaths swaths;
  for (unsigned int i = 0; i < rows.size(); ++i) {
    int id = rows[i].second;
    if (skipId(ids_to_skip, id)) {
      continue;
    }

    double width = 0.0;
    if (i == rows.size() - 1) {
      width = calculateWidth(rows[i].first, rows[i - 1].first);
    } else {
      width = calculateWidth(rows[i + 1].first, rows[i].first);
    }

    auto & row = rows[i].first;
    LineString line_string;
    for (unsigned int j = 0; j < row.size(); ++j) {
      Point point;
      row.getGeometry(j, point);

      line_string.addPoint(point);
    }
    swaths.emplace_back(
      line_string, width, id);
  }
  return swaths;
}

Swaths SwathFactory::generateOffsetSwaths(
  const Rows & rows, const SkipIDs & ids_to_skip,
  float offset)
{
  if (rows.size() < 2) {
    throw std::invalid_argument("Rows must be greater than 1");
  }

  Swaths swaths;
  for (unsigned int i = 1; i < rows.size(); ++i) {
    auto & prev_row = rows[i - 1].first;
    auto & cur_row = rows[i].first;

    if (prev_row.size() != cur_row.size() ) {
      throw std::invalid_argument("All rows must be the same size");
    }

    int id = rows[i - 1].second;
    if (skipId(ids_to_skip, id)) {
      continue;
    }

    LineString line_string;
    std::vector<double> widths;
    for (unsigned int j = 0; j < prev_row.size(); ++j) {
      Point prev_point;
      prev_row.getGeometry(j, prev_point);

      Point cur_point;
      cur_row.getGeometry(j, cur_point);

      Point vector = cur_point - prev_point;
      double width = std::sqrt(vector.getX() * vector.getX() + vector.getY() * vector.getY());
      widths.push_back(width);

      Point unit_vector = vector * (1 / width);

      Point new_point = prev_point + unit_vector * offset;
      line_string.addPoint(new_point);
    }
    swaths.emplace_back(
      line_string, *std::min_element(
        widths.begin(),
        widths.end()), id);
  }
  return swaths;
}

bool SwathFactory::skipId(const SkipIDs & ids_to_skip, int id)
{
  auto it = std::find(ids_to_skip.begin(), ids_to_skip.end(), id);
  return it != ids_to_skip.end();
}

double SwathFactory::calculateWidth(const LineString & row1, const LineString & row2)
{
  std::vector<double> widths;
  for (unsigned int i = 0; i < row1.size(); ++i) {
    Point point1;
    row1.getGeometry(i, point1);

    Point point2;
    row2.getGeometry(i, point2);

    Point vector = point2 - point1;
    double width = std::sqrt(vector.getX() * vector.getX() + vector.getY() * vector.getY());
    widths.push_back(width);
  }
  return *std::min_element(widths.begin(), widths.end());
}

Rows RowSwathGenerator::adjustRowOrientations(const Rows & input_rows)
{
  if (input_rows.size() < 2u) {
    return input_rows;
  }

  Rows rows = input_rows;

  auto unitVectorize = [&](const Row & row) {
      Point v = row.first.EndPoint() - row.first.StartPoint();
      double normalize = 1.0 / std::sqrt(
        v.getX() * v.getX() + v.getY() * v.getY());
      return v * normalize;
    };

  auto dot = [](const Point & A, const Point & B) {
      return A.getX() * B.getX() + A.getY() * B.getY();
    };

  const auto u_init_swath = unitVectorize(rows[0]);
  for (unsigned int i = 0; i != rows.size(); i++) {
    if (dot(u_init_swath, unitVectorize(rows[i])) < 0.0) {
      rows[i].first.reversePoints();
    }
  }
  return rows;
}

Swaths RowSwathGenerator::generateSwaths(
  const Rows & rows,
  const opennav_coverage_msgs::msg::RowSwathMode & settings)
{
  RowSwathType action_type = toType(settings.mode);
  float offset = 0.0f;
  SkipIDs skip_ids;

  // If not set by action, use default mode
  if (action_type == RowSwathType::UNKNOWN) {
    action_type = default_type_;
    offset = default_offset_;
  } else {
    offset = settings.offset;
  }
  skip_ids = settings.skip_ids;

  RCLCPP_DEBUG(
    logger_, "Generating Swaths with: %s", toString(action_type).c_str());

  // Manually correct annotated row orientations for consistent operations
  Rows adjusted_rows = adjustRowOrientations(rows);

  Swaths swaths;
  switch (action_type) {
    case RowSwathType::CENTER:
      return swath_factory_.generateCenterSwaths(adjusted_rows, skip_ids);
    case RowSwathType::OFFSET:
      return swath_factory_.generateOffsetSwaths(adjusted_rows, skip_ids, offset);
    case RowSwathType::ROWSARESWATHS:
      return swath_factory_.generateRowsAreSwaths(adjusted_rows, skip_ids);
    default:
      throw std::invalid_argument("RowSwathType was not set");
  }
}

void RowSwathGenerator::setMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
}

std::string RowSwathGenerator::toString(const RowSwathType & type)
{
  switch (type) {
    case RowSwathType::CENTER:
      return "CENTER";
    case RowSwathType::OFFSET:
      return "OFFSET";
    case RowSwathType::ROWSARESWATHS:
      return "ROWSARESWATHS";
    default:
      return "UNKNOWN";
  }
}

RowSwathType RowSwathGenerator::toType(const std::string & str)
{
  std::string mode_str = str;
  opennav_coverage::util::toUpper(mode_str);
  if (mode_str == "CENTER") {
    return RowSwathType::CENTER;
  } else if (mode_str == "OFFSET") {
    return RowSwathType::OFFSET;
  } else if (mode_str == "ROWSARESWATHS") {
    return RowSwathType::ROWSARESWATHS;
  } else {
    return RowSwathType::UNKNOWN;
  }
}

}  // namespace opennav_row_coverage
