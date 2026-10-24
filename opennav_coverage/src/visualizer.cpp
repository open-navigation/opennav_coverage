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
  headland_swaths_pub_.reset();
  connection_turns_pub_.reset();
}

void Visualizer::visualize(
  const Field & total_field, const Field & no_headland_field,
  const Point & ref_pt, const nav_msgs::msg::Path & nav_path,
  const Swaths swaths, const std_msgs::msg::Header & header,
  const Path & headland_path,
  const std::vector<Path> & connection_turns)
{
  // F2C strips out reference point of all data, so we need to readd it
  // so that our visualizations mirror the true transformed output

  // Visualize coverage path
  if (nav_plan_pub_->get_subscription_count() > 0 && nav_path.poses.size() > 0) {
    auto utm_path = std::make_unique<nav_msgs::msg::Path>(nav_path);
    utm_path->header.frame_id = GLOBAL_FRAME;
    for (unsigned int i = 0; i != utm_path->poses.size(); i++) {
      utm_path->poses[i].header.frame_id = GLOBAL_FRAME;
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

  // Headland perimeter loop, if driven, in a distinct color so it's not confused
  // with the (identically-shaped) planning_field boundary.
  if (headland_swaths_pub_->get_subscription_count() > 0) {
    auto output_hl = std::make_unique<visualization_msgs::msg::Marker>();
    output_hl->header.stamp = header.stamp;
    output_hl->header.frame_id = GLOBAL_FRAME;

    if (headland_path.size() == 0) {
      output_hl->action = visualization_msgs::msg::Marker::DELETEALL;
    } else {
      output_hl->action = visualization_msgs::msg::Marker::ADD;
      output_hl->type = visualization_msgs::msg::Marker::LINE_STRIP;
      output_hl->pose.orientation.w = 1.0;
      output_hl->scale.x = 0.4;
      output_hl->color.r = 1.0;
      output_hl->color.a = 1.0;

      for (const auto & s : headland_path) {
        output_hl->points.push_back(util::pointToPoint32(util::toMsg(s.point + ref_pt)));
      }
      output_hl->points.push_back(
        util::pointToPoint32(util::toMsg(headland_path.back().atEnd() + ref_pt)));
    }

    headland_swaths_pub_->publish(std::move(output_hl));
  }

  // The turns planned between swath groups, drawn on their own so a rounded
  // corner can be told apart from one left square. Each carries the label the
  // planner logs for it.
  if (connection_turns_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

    visualization_msgs::msg::Marker clear;
    clear.header.stamp = header.stamp;
    clear.header.frame_id = GLOBAL_FRAME;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    msg->markers.push_back(clear);

    if (!connection_turns.empty()) {
      visualization_msgs::msg::Marker lines;
      lines.header.stamp = header.stamp;
      lines.header.frame_id = GLOBAL_FRAME;
      lines.ns = "turns";
      lines.action = visualization_msgs::msg::Marker::ADD;
      // LINE_LIST, not LINE_STRIP: the turns are disjoint and a strip would
      // draw a line across the field between each pair of them.
      lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      lines.pose.orientation.w = 1.0;
      lines.scale.x = 0.05;
      lines.color.r = 0.7;
      lines.color.b = 1.0;
      lines.color.a = 1.0;

      for (size_t t = 0; t != connection_turns.size(); t++) {
        const auto & turn = connection_turns[t];
        if (turn.size() == 0) {
          continue;
        }
        for (size_t i = 0; i + 1 < turn.size(); i++) {
          lines.points.push_back(util::pointToPoint32(util::toMsg(turn[i].point + ref_pt)));
          lines.points.push_back(util::pointToPoint32(util::toMsg(turn[i + 1].point + ref_pt)));
        }
        lines.points.push_back(util::pointToPoint32(util::toMsg(turn.back().point + ref_pt)));
        lines.points.push_back(util::pointToPoint32(util::toMsg(turn.back().atEnd() + ref_pt)));

        visualization_msgs::msg::Marker label;
        label.header.stamp = header.stamp;
        label.header.frame_id = GLOBAL_FRAME;
        label.ns = "turn_labels";
        label.id = static_cast<int>(t);
        label.action = visualization_msgs::msg::Marker::ADD;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.pose.orientation.w = 1.0;
        label.pose.position =
          util::pointToPoint32(util::toMsg(turn[turn.size() / 2].point + ref_pt));
        label.scale.z = 0.4;
        label.color.r = 1.0;
        label.color.g = 1.0;
        label.color.b = 1.0;
        label.color.a = 1.0;
        label.text = util::turnLabel(t);
        msg->markers.push_back(label);
      }
      msg->markers.push_back(lines);
    }

    connection_turns_pub_->publish(std::move(msg));
  }
}

}  // namespace opennav_coverage
