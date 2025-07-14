// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include "nav2_smac_planner/collision_checker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      oriented_footprint.push_back(new_pt);
    }

    oriented_footprints_.push_back(oriented_footprint);
  }

  unoriented_footprint_ = footprint;
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  if (footprint_is_radius_) {
    // For radius-based collision checking
    CollisionResult result = checkCenterPointCollision(x, y, traverse_unknown);
    center_cost_ = static_cast<float>(result.cost);
    return result.in_collision;
  } else {
    // For footprint-based collision checking
    CollisionResult result = checkFootprintCollision(x, y, angle_bin, traverse_unknown);
    center_cost_ = static_cast<float>(result.cost);
    return result.in_collision;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return center_cost_ >= INSCRIBED_COST;
}

bool GridCollisionChecker::inCollision(
  const std::vector<geometry_msgs::msg::PoseStamped>& path,
  const bool& traverse_unknown)
{
  if (path.empty()) {
    return false;
  }

  // For radius mode, use simple center point checking
  if (footprint_is_radius_) {
    for (const auto& pose : path) {
      unsigned int mx, my;
      if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        return true; // Outside map bounds = collision
      }

      CollisionResult result = checkCenterPointCollision(
        static_cast<float>(mx), static_cast<float>(my), traverse_unknown);
      
      if (result.in_collision) {
        return true;
      }
    }
    return false;
  }

  // For footprint mode, use swept convex hull approximation
  if (path.size() == 1) {
    // Single pose - use regular collision checking
    const auto& pose = path[0];
    unsigned int mx, my;
    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      return true; // Outside map bounds = collision
    }

    double yaw = tf2::getYaw(pose.pose.orientation);
    while (yaw < 0.0) yaw += 2.0 * M_PI;
    while (yaw >= 2.0 * M_PI) yaw -= 2.0 * M_PI;
    
    double bin_size = 2.0 * M_PI / static_cast<double>(angles_.size());
    unsigned int angle_bin = static_cast<unsigned int>(yaw / bin_size) % angles_.size();
    
    return inCollision(static_cast<float>(mx), static_cast<float>(my), 
                      static_cast<float>(angle_bin), traverse_unknown);
  }

  // Multi-pose path - use swept convex hull approximation between consecutive poses
  // Process path segment by segment
  for (size_t i = 0; i < path.size() - 1; ++i) {
    const auto& pose1 = path[i];
    const auto& pose2 = path[i + 1];

    // Get angle bins for both poses
    double yaw1 = tf2::getYaw(pose1.pose.orientation);
    double yaw2 = tf2::getYaw(pose2.pose.orientation);
    
    // Normalize angles
    while (yaw1 < 0.0) yaw1 += 2.0 * M_PI;
    while (yaw1 >= 2.0 * M_PI) yaw1 -= 2.0 * M_PI;
    while (yaw2 < 0.0) yaw2 += 2.0 * M_PI;
    while (yaw2 >= 2.0 * M_PI) yaw2 -= 2.0 * M_PI;
    
    double bin_size = 2.0 * M_PI / static_cast<double>(angles_.size());
    unsigned int angle_bin1 = static_cast<unsigned int>(yaw1 / bin_size) % angles_.size();
    unsigned int angle_bin2 = static_cast<unsigned int>(yaw2 / bin_size) % angles_.size();

    if (angle_bin1 >= oriented_footprints_.size() || angle_bin2 >= oriented_footprints_.size()) {
      continue; // Skip invalid angle bins
    }

    // Create swept footprint for this segment by combining both poses
    nav2_costmap_2d::Footprint segment_footprint;
    
    // Add footprint points from first pose
    const auto& oriented_footprint1 = oriented_footprints_[angle_bin1];
    for (const auto& point : oriented_footprint1) {
      geometry_msgs::msg::Point world_point;
      world_point.x = pose1.pose.position.x + point.x;
      world_point.y = pose1.pose.position.y + point.y;
      segment_footprint.push_back(world_point);
    }
    
    // Add footprint points from second pose
    const auto& oriented_footprint2 = oriented_footprints_[angle_bin2];
    for (const auto& point : oriented_footprint2) {
      geometry_msgs::msg::Point world_point;
      world_point.x = pose2.pose.position.x + point.x;
      world_point.y = pose2.pose.position.y + point.y;
      segment_footprint.push_back(world_point);
    }

    // Compute convex hull of this segment
    std::vector<geometry_msgs::msg::Point> segment_hull = computeConvexHull(segment_footprint);
    
    if (segment_hull.size() >= 3) {
      // Use parent class footprintCost for collision checking
      double segment_cost = footprintCost(segment_hull);
      
      if (segment_cost == static_cast<double>(UNKNOWN_COST) && !traverse_unknown) {
        return true; // Unknown area and not allowed to traverse
      }
      
      if (segment_cost >= static_cast<double>(OCCUPIED_COST)) {
        return true; // Collision detected
      }
    }
  }

  // If we got here, no collision was detected
  return false;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

GridCollisionChecker::CollisionResult GridCollisionChecker::checkCenterPointCollision(
  const float& x, const float& y, const bool& traverse_unknown) const
{
  CollisionResult result;
  result.needs_full_check = false;
  
  // Use parent class pointCost method
  double cost = pointCost(static_cast<int>(x + 0.5f), static_cast<int>(y + 0.5f));
  result.cost = cost;
  
  // Check for immediate collision conditions
  if (cost == static_cast<double>(UNKNOWN_COST) && !traverse_unknown) {
    result.in_collision = true;
    return result;
  }
  if (cost >= static_cast<double>(OCCUPIED_COST)) {
    result.in_collision = true;
    return result;
  }
  
  if (footprint_is_radius_) {
    // For radius mode, inscribed cost means collision
    result.in_collision = (cost >= static_cast<double>(INSCRIBED_COST));
  } else {
    // For footprint mode, check immediate center point collisions
    if (cost >= static_cast<double>(OCCUPIED_COST)) {
      result.in_collision = true;
      return result;
    }
    
    // Check if we need full footprint checking
    if (cost >= static_cast<double>(possible_collision_cost_) && possible_collision_cost_ > 0.0f) {
      result.needs_full_check = true;
    }
    
    result.in_collision = false;
  }
  
  return result;
}

GridCollisionChecker::CollisionResult GridCollisionChecker::checkFootprintCollision(
  const float& x, const float& y, const float& angle_bin, 
  const bool& traverse_unknown)
{
  CollisionResult result;
  result.needs_full_check = false;
  
  // First check center point for early optimization
  CollisionResult center_result = checkCenterPointCollision(x, y, traverse_unknown);
  
  // If center point indicates we can skip full footprint check
  if (!center_result.needs_full_check) {
    return center_result;
  }
  
  // Perform full footprint checking using parent class method
  double wx, wy;
  costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
  
  if (static_cast<size_t>(angle_bin) >= oriented_footprints_.size()) {
    // Fallback to center point cost
    result.cost = center_result.cost;
    result.in_collision = center_result.in_collision;
    return result;
  }
  
  // Use parent class footprintCost method
  const nav2_costmap_2d::Footprint& oriented_footprint = oriented_footprints_[static_cast<size_t>(angle_bin)];
  nav2_costmap_2d::Footprint current_footprint;
  current_footprint.reserve(oriented_footprint.size());
  
  for (const auto& point : oriented_footprint) {
    geometry_msgs::msg::Point new_pt;
    new_pt.x = wx + point.x;
    new_pt.y = wy + point.y;
    current_footprint.push_back(new_pt);
  }
  
  // Check full area covered by footprint using parent class method
  double footprint_cost = footprintCost(current_footprint);
  result.cost = footprint_cost;
  
  if (footprint_cost == static_cast<double>(UNKNOWN_COST) && traverse_unknown) {
    result.in_collision = false;
    return result;
  }
  
  // if occupied or unknown and not to traverse unknown space
  result.in_collision = (footprint_cost >= static_cast<double>(OCCUPIED_COST));
  return result;
}

std::vector<geometry_msgs::msg::Point> GridCollisionChecker::computeConvexHull(
  const std::vector<geometry_msgs::msg::Point>& points) const
{
  if (points.size() < 3) {
    return points;
  }

  // Convert to OpenCV points
  std::vector<cv::Point2f> cv_points;
  cv_points.reserve(points.size());
  for (const auto& point : points) {
    cv_points.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }

  // Compute convex hull using OpenCV
  std::vector<cv::Point2f> hull;
  cv::convexHull(cv_points, hull);

  // Convert back to geometry_msgs
  std::vector<geometry_msgs::msg::Point> result;
  result.reserve(hull.size());
  for (const auto& cv_point : hull) {
    geometry_msgs::msg::Point point;
    point.x = static_cast<double>(cv_point.x);
    point.y = static_cast<double>(cv_point.y);
    point.z = 0.0;
    result.push_back(point);
  }

  return result;
}

}  // namespace nav2_smac_planner
