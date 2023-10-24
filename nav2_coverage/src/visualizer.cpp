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
#include <algorithm>

#include "nav2_coverage/visualizer.hpp"

namespace nav2_coverage
{

void Visualizer::deactivate()
{
  nav_plan_pub_.reset();
  headlands_pub_.reset();
  planning_field_pub_.reset();
  swaths_pub_.reset();
}

void Visualizer::visualize(
  const Field & total_field, const Field & no_headland_field,
  const std::shared_ptr<ComputeCoveragePath::Result> & result,
  const std_msgs::msg::Header & header)
{
  // Visualize coverage path
  if (nav_plan_pub_->get_subscription_count() > 0) {
    nav_plan_pub_->publish(std::move(std::make_unique<nav_msgs::msg::Path>(result->nav_path)));
  }

  // Visualize field boundary
  if (headlands_pub_->get_subscription_count() > 0) {
    auto field_polygon = std::make_unique<geometry_msgs::msg::PolygonStamped>();
    field_polygon->header = header;
    Polygon boundary = total_field.getGeometry(0);  // Only outer-most polygon boundary
    for (unsigned int i = 0; i != boundary.size(); i++) {
      field_polygon->polygon.points.push_back(util::toMsg(boundary.getGeometry(i)));
    }
    headlands_pub_->publish(std::move(field_polygon));
  }

  // Visualize field for planning (after headland removed)
  if (planning_field_pub_->get_subscription_count() > 0) {
    auto headlandless_polygon = std::make_unique<geometry_msgs::msg::PolygonStamped>();
    headlandless_polygon->header = header;
    Polygon planning_field = no_headland_field.getGeometry(0);  // Only outer-most polygon boundary
    for (unsigned int i = 0; i != planning_field.size(); i++) {
      headlandless_polygon->polygon.points.push_back(util::toMsg(planning_field.getGeometry(i)));
    }
    planning_field_pub_->publish(std::move(headlandless_polygon));
  }

  // Visualize swaths alone
  if (swaths_pub_->get_subscription_count() > 0) {
    auto output_swaths = std::make_unique<visualization_msgs::msg::Marker>();
    output_swaths->header = header;
    output_swaths->action = visualization_msgs::msg::Marker::ADD;
    output_swaths->type = visualization_msgs::msg::Marker::LINE_LIST;
    output_swaths->pose.orientation.w = 1.0;
    output_swaths->scale.x = 0.3;
    output_swaths->scale.y = 0.3;
    output_swaths->scale.z = 0.3;
    output_swaths->color.b = 1.0;
    output_swaths->color.a = 1.0;

    for (unsigned int i = 0; i != result->coverage_path.swaths.size(); i++) {
      auto & swath = result->coverage_path.swaths[i];
      output_swaths->points.push_back(util::pointToPoint32(swath.start));
      output_swaths->points.push_back(util::pointToPoint32(swath.end));
    }

    swaths_pub_->publish(std::move(output_swaths));
  }
}

}  // namespace nav2_coverage
