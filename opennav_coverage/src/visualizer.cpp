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

#include "opennav_coverage/visualizer.hpp"

namespace opennav_coverage
{

const std::string GLOBAL_FRAME = "map";  // NOLINT

void Visualizer::deactivate()
{
  nav_plan_pub_.reset();
  headlands_pub_.reset();
  planning_field_pub_.reset();
  swaths_pub_.reset();
}

void Visualizer::visualize(
  const Field & total_field, const Field & no_headland_field,
  const Point & ref_pt, const nav_msgs::msg::Path & nav_path,
  const Swaths swaths, const std_msgs::msg::Header & header)
{
  // F2C strips out reference point of all data, so we need to readd it
  // so that our visualizations mirror the true transformed output

  // Visualize coverage path
  if (nav_plan_pub_->get_subscription_count() > 0 && nav_path.poses.size() > 0) {
    auto utm_path = std::make_unique<nav_msgs::msg::Path>(nav_path);
    utm_path->header.frame_id = GLOBAL_FRAME;
    for (unsigned int i = 0; i != utm_path->poses.size(); i++) {
      utm_path->poses[i].pose.position.x += ref_pt.getX();
      utm_path->poses[i].pose.position.y += ref_pt.getY();
    }
    nav_plan_pub_->publish(std::move(utm_path));
  }

  // Visualize field boundary
  if (headlands_pub_->get_subscription_count() > 0) {
    auto field_polygon = std::make_unique<geometry_msgs::msg::PolygonStamped>();
    field_polygon->header.stamp = header.stamp;
    field_polygon->header.frame_id = GLOBAL_FRAME;
    Polygon boundary = total_field.getGeometry(0);  // Only outer-most polygon boundary
    for (unsigned int i = 0; i != boundary.size(); i++) {
      field_polygon->polygon.points.push_back(util::toMsg(boundary.getGeometry(i) + ref_pt));
    }
    headlands_pub_->publish(std::move(field_polygon));
  }

  // Visualize field for planning (after headland removed)
  if (planning_field_pub_->get_subscription_count() > 0) {
    auto headlandless_polygon = std::make_unique<geometry_msgs::msg::PolygonStamped>();
    headlandless_polygon->header.stamp = header.stamp;
    headlandless_polygon->header.frame_id = GLOBAL_FRAME;
    if (no_headland_field.size() > 0) {
      Polygon planning_field = no_headland_field.getGeometry(0);  // Only outer polygon boundary
      for (unsigned int i = 0; i != planning_field.size(); i++) {
        headlandless_polygon->polygon.points.push_back(
          util::toMsg(planning_field.getGeometry(i) + ref_pt));
      }
      planning_field_pub_->publish(std::move(headlandless_polygon));
    }
  }

  // Visualize swaths alone
  if (swaths_pub_->get_subscription_count() > 0) {
    auto output_swaths = std::make_unique<visualization_msgs::msg::Marker>();
    output_swaths->header.stamp = header.stamp;
    output_swaths->header.frame_id = GLOBAL_FRAME;
    output_swaths->action = visualization_msgs::msg::Marker::ADD;
    output_swaths->type = visualization_msgs::msg::Marker::LINE_LIST;
    output_swaths->pose.orientation.w = 1.0;
    output_swaths->scale.x = 0.3;
    output_swaths->scale.y = 0.3;
    output_swaths->scale.z = 0.3;
    output_swaths->color.b = 1.0;
    output_swaths->color.a = 1.0;

    for (unsigned int i = 0; i != swaths.size(); i++) {
      auto & swath = swaths[i];
      output_swaths->points.push_back(
        util::pointToPoint32(util::toMsg(swath.startPoint() + ref_pt)));
      output_swaths->points.push_back(
        util::pointToPoint32(util::toMsg(swath.endPoint() + ref_pt)));
    }

    swaths_pub_->publish(std::move(output_swaths));
  }
}

}  // namespace opennav_coverage
