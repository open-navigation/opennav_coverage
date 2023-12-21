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

#ifndef OPENNAV_ROW_COVERAGE__UTILS_HPP_
#define OPENNAV_ROW_COVERAGE__UTILS_HPP_

#include <vector>
#include <string>
#include <memory>
#include <utility>

#include "tinyxml2.h" // NOLINT
#include "opennav_coverage/types.hpp"
#include "opennav_row_coverage/types.hpp"

namespace opennav_row_coverage
{

namespace util
{

using namespace opennav_coverage;  // NOLINT

/**
 * @brief Transform rows to a new coordinate system and a new reference.
 * Note: The field must be transformed to UTM before this operation.
 * @param rows The rows to transform
 * @param field The field holds the reference point and coordinate systems
 * @return rows transformed by the transformation of the field's coordinates
 */
inline Rows transformRowsWithRef(
  const Rows & rows,
  const F2CField & field)
{
  auto coords = f2c::Transform::generateCoordTransf(field.getPrevCRS(), field.getCRS());
  Rows trans_rows;
  for (const auto & row : rows) {
    LineString new_row = row.first.clone();
    for (auto & point : new_row) {
      point->transform(coords.get());
      point = point - field.getRefPoint();
    }
    trans_rows.emplace_back(new_row, row.second);
  }
  return trans_rows;
}

/**
 * @brief Removes the field's reference point from the rows so that
 * All F2C operations continue to reference the axial point
 * @param rows The rows to transform
 * @param field The field holds the reference point and coordinate systems
 * @return rows transformed by the transformation of the field's coordinates
 */
inline Rows removeRowsRefPoint(
  const Rows & rows,
  const F2CField & field)
{
  Rows trans_rows;
  for (const auto & row : rows) {
    LineString new_row = row.first.clone();
    for (auto & point : new_row) {
      point = point - field.getRefPoint();
    }
    trans_rows.emplace_back(new_row, row.second);
  }
  return trans_rows;
}

/**
 * @brief Parse the rows from file format
 * @param file_path The path of the file to parse
 * @param reorder Whether to reorder by ID
 * @throw std::invalid_argument If the file is invalid
 * @return Rows A vector of line strings corresponding to the row
 */
inline Rows parseRows(const std::string & file_path, const bool reorder = false)
{
  tinyxml2::XMLDocument doc;
  doc.LoadFile(file_path.c_str());
  auto * p_parcel = doc.RootElement();

  if (p_parcel == nullptr) {
    throw std::invalid_argument("File not found");
  }

  Rows rows;
  for (tinyxml2::XMLElement * rowElement = p_parcel->FirstChildElement("Row"); rowElement;
    rowElement = rowElement->NextSiblingElement("Row"))
  {
    const char * id = rowElement->Attribute("id");
    if (!id) {
      id = "0";
    }

    tinyxml2::XMLElement * geometry = rowElement->FirstChildElement("geometry");
    if (!geometry) {
      throw std::invalid_argument("Failed to get geometry");
    }

    tinyxml2::XMLElement * line_string = geometry->FirstChildElement("gml:LineString");
    if (!line_string) {
      throw std::invalid_argument("Failed to get line string");
    }

    const char * srs_name = line_string->Attribute("srsName");
    if (!srs_name) {
      throw std::invalid_argument("Failed to get srsName");
    }

    tinyxml2::XMLElement * coords = line_string->FirstChildElement("gml:coordinates");
    if (!coords) {
      throw std::invalid_argument("Failed to get coords");
    }

    const char * coord_text = coords->GetText();
    if (!coord_text) {
      throw std::invalid_argument("Failed to get coord text");
    }
    std::string p_coords = std::string(coord_text);

    auto findAndReplaceAll =
      [](std::string & data, std::string toSearch, std::string replaceStr) {
        size_t pos = data.find(toSearch);
        while (pos != std::string::npos) {
          data.replace(pos, toSearch.size(), replaceStr);
          pos = data.find(toSearch, pos + replaceStr.size());
        }
        return;
      };

    findAndReplaceAll(p_coords, ",", ";");
    findAndReplaceAll(p_coords, " ", ", ");
    findAndReplaceAll(p_coords, ";", " ");
    p_coords = "LINESTRING (" + p_coords + ")";

    OGRGeometry * new_geom = nullptr;
    auto result = OGRGeometryFactory::createFromWkt(p_coords.c_str(), nullptr, &new_geom);

    if (result == OGRERR_NONE) {
      rows.emplace_back(new_geom, std::stoi(id));
    } else {
      throw std::invalid_argument("Failed to create linestring from wkt");
    }
    OGRGeometryFactory::destroyGeometry(new_geom);
  }

  if (reorder) {
    std::sort(
      rows.begin(), rows.end(),
      [](auto & left, auto & right) {
        return left.second < right.second;
      });
  }

  return rows;
}

}  // namespace util

}  // namespace opennav_row_coverage

#endif  // OPENNAV_ROW_COVERAGE__UTILS_HPP_
