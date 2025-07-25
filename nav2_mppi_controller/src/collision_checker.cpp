// Copyright (c) 2025, Angsa Robotics GmbH
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

#include "nav2_mppi_controller/collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include <algorithm>
#include <cmath>

namespace nav2_mppi_controller
{

MPPICollisionChecker::MPPICollisionChecker(
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

void MPPICollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const std::string & inflation_layer_name)
{
  // Calculate the circumscribed cost automatically
  possible_collision_cost_ = findCircumscribedCost(inflation_layer_name);
  
  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller#potential-fields"
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

bool MPPICollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & yaw,
  const bool & traverse_unknown)
{
  // Convert yaw to angle bin internally
  unsigned int angle_bin = getAngleBin(yaw);
  
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(costmap_->getCost(
    static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      return false;
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (center_cost_ == UNKNOWN_COST && !traverse_unknown) {
      return true;
    }

    if (center_cost_ == INSCRIBED_COST || center_cost_ == OCCUPIED_COST) {
      return true;
    }

    // Use full area checking instead of edge-only checking
    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
    geometry_msgs::msg::Point new_pt;
    const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    // Check full area covered by footprint
    float footprint_cost = static_cast<float>(footprintCost(current_footprint));

    if (footprint_cost == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost >= OCCUPIED_COST;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= INSCRIBED_COST;
  }
}

bool MPPICollisionChecker::inCollision(
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

float MPPICollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool MPPICollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

unsigned int MPPICollisionChecker::getAngleBin(float yaw) const
{
  // Normalize yaw to [0, 2*PI)
  while (yaw < 0.0f) {
    yaw += 2.0f * M_PI;
  }
  while (yaw >= 2.0f * M_PI) {
    yaw -= 2.0f * M_PI;
  }
  
  // Calculate bin size
  const float bin_size = 2.0f * M_PI / static_cast<float>(angles_.size());
  
  // Calculate bin index with rounding
  unsigned int bin = static_cast<unsigned int>(yaw / bin_size + 0.5f);
  
  // Handle edge case where rounding puts us at the last bin
  if (bin >= angles_.size()) {
    bin = 0;  // Wrap around to first bin
  }
  
  return bin;
}

CollisionResult MPPICollisionChecker::inCollision(
  const std::vector<float> & x,
  const std::vector<float> & y,
  const std::vector<float> & yaw,
  const bool & traverse_unknown)
{
  CollisionResult result;
  result.in_collision = false;

  // Check if all vectors have the same size
  if (x.size() != y.size() || x.size() != yaw.size()) {
    result.in_collision = true;
    return result;
  }

  // Initialize result vectors
  result.center_cost.resize(x.size(), 0.0f);
  result.footprint_cost.resize(x.size(), 0.0f);
  result.collision_type.resize(x.size(), CollisionType::NONE);

  // Step 1: Check all poses for bounds and get center costs
  std::vector<bool> needs_footprint_check(x.size(), false);
  
  for (size_t i = 0; i < x.size(); ++i) {
    // Check to make sure cell is inside the map
    if (outsideRange(costmap_->getSizeInCellsX(), x[i]) ||
        outsideRange(costmap_->getSizeInCellsY(), y[i]))
    {
      result.in_collision = true;
      return result;
    }

    // Get center cost for this pose
    float current_center_cost = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x[i] + 0.5f), static_cast<unsigned int>(y[i] + 0.5f)));
    
    result.center_cost[i] = current_center_cost;

    if (current_center_cost == UNKNOWN_COST && !traverse_unknown) {
      result.in_collision = true;
      result.collision_type[i] = CollisionType::POINT_COST;
      return result;
    }

    if (current_center_cost >= INSCRIBED_COST) {
      result.in_collision = true;
      result.collision_type[i] = CollisionType::POINT_COST;
      return result;
    }

    // For footprint-based collision checking, mark poses that need further checking
    if (!footprint_is_radius_) {
      // Skip if center cost is below collision threshold
      if (current_center_cost >= possible_collision_cost_ || possible_collision_cost_ <= 0.0f) {
        needs_footprint_check[i] = true;
      } else {
        // If center cost is below threshold, use center cost as footprint cost
        result.footprint_cost[i] = current_center_cost;
      }
    } else {
      // For radius-based checking, we still need to calculate footprint cost
      // even though we use center cost for collision detection
      result.footprint_cost[i] = current_center_cost;
    }
  }

  // Step 2: If using footprint, check footprint costs for all poses
  if (!footprint_is_radius_) {
    std::vector<bool> needs_swept_area_check(x.size(), false);
    // Cache computed footprints to avoid recomputation in Step 4
    std::vector<nav2_costmap_2d::Footprint> cached_footprints(x.size());
    
    for (size_t i = 0; i < x.size(); ++i) {
      // Skip poses that don't need footprint checking
      if (!needs_footprint_check[i]) {
        continue;
      }

      // Get angle bin from yaw
      unsigned int angle_bin_idx = getAngleBin(yaw[i]);

      // Prepare footprint for collision checking
      double wx, wy;
      costmap_->mapToWorld(static_cast<double>(x[i]), static_cast<double>(y[i]), wx, wy);
      geometry_msgs::msg::Point new_pt;
      const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin_idx];
      nav2_costmap_2d::Footprint current_footprint;
      current_footprint.reserve(oriented_footprint.size());
      
      for (unsigned int j = 0; j < oriented_footprint.size(); ++j) {
        new_pt.x = wx + oriented_footprint[j].x;
        new_pt.y = wy + oriented_footprint[j].y;
        current_footprint.push_back(new_pt);
      }

      // Cache the computed footprint for potential reuse in Step 4
      cached_footprints[i] = current_footprint;

      // Check footprint perimeter
      float footprint_cost = static_cast<float>(footprintCost(current_footprint, false));
      
      // Store footprint cost in result
      result.footprint_cost[i] = footprint_cost;

      if (footprint_cost == UNKNOWN_COST && !traverse_unknown) {
        result.in_collision = true;
        result.collision_type[i] = CollisionType::FOOTPRINT_COST;
        return result;
      }

      if (footprint_cost >= OCCUPIED_COST) {
        result.in_collision = true;
        result.collision_type[i] = CollisionType::FOOTPRINT_COST;
        return result;
      }

      // Mark for swept area checking if footprint cost is INSCRIBED_COST
      if (footprint_cost == INSCRIBED_COST) {
        needs_swept_area_check[i] = true;
      }
    }

    // Step 3: Check swept area for consecutive poses with footprint cost INSCRIBED_COST
    // Find consecutive sequences of poses that need swept area checking
    std::vector<std::vector<size_t>> consecutive_sequences;
    std::vector<size_t> current_sequence;
    
    for (size_t i = 0; i < x.size(); ++i) {
      if (needs_swept_area_check[i]) {
        current_sequence.push_back(i);
        // Limit sequence to maximum of 5 poses
        if (current_sequence.size() >= 5) {
          consecutive_sequences.push_back(current_sequence);
          current_sequence.clear();
        }
      } else {
        if (!current_sequence.empty()) {
          // Only add sequences with more than one pose
          if (current_sequence.size() > 1) {
            consecutive_sequences.push_back(current_sequence);
          }
          current_sequence.clear();
        }
      }
    }
    
    // Don't forget the last sequence if it ends at the last pose
    if (!current_sequence.empty()) {
      // Only add sequences with more than one pose
      if (current_sequence.size() > 1) {
        consecutive_sequences.push_back(current_sequence);
      }
    }
    
    // Step 4: Check swept area using convex hull for each consecutive sequence
    for (const auto& sequence : consecutive_sequences) {
      // Collect all footprint points from consecutive poses using cached footprints
      std::vector<geometry_msgs::msg::Point> all_points;
      
      for (size_t seq_idx : sequence) {
        // Use cached footprint instead of recomputing
        const nav2_costmap_2d::Footprint & current_footprint = cached_footprints[seq_idx];
        
        for (const auto& footprint_pt : current_footprint) {
          all_points.push_back(footprint_pt);
        }
      }
      
      // Create convex hull from all collected points
      nav2_costmap_2d::Footprint convex_hull = createConvexHull(all_points);
      
      // Check swept area cost using full area checking
      float swept_area_cost = static_cast<float>(footprintCost(convex_hull, true));
      
      if (swept_area_cost == UNKNOWN_COST && !traverse_unknown) {
        result.in_collision = true;
        // Mark all poses in this sequence as swept area collision
        for (size_t seq_idx : sequence) {
          result.collision_type[seq_idx] = CollisionType::SWEPT_AREA_COST;
        }
        return result;
      }
      
      if (swept_area_cost >= OCCUPIED_COST) {
        result.in_collision = true;
        // Mark all poses in this sequence as swept area collision
        for (size_t seq_idx : sequence) {
          result.collision_type[seq_idx] = CollisionType::SWEPT_AREA_COST;
        }
        return result;
      }
    }
    
  }

  return result;
}

nav2_costmap_2d::Footprint MPPICollisionChecker::createConvexHull(
  const std::vector<geometry_msgs::msg::Point>& points)
{
  if (points.size() <= 1) {
    return points;
  }

  // Create a copy of points for sorting
  std::vector<geometry_msgs::msg::Point> sorted_points = points;
  
  // Sort points lexicographically (first by x, then by y)
  std::sort(sorted_points.begin(), sorted_points.end(), 
    [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });

  // Remove duplicate points
  sorted_points.erase(
    std::unique(sorted_points.begin(), sorted_points.end(),
      [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        return std::abs(a.x - b.x) < 1e-9 && std::abs(a.y - b.y) < 1e-9;
      }),
    sorted_points.end());

  if (sorted_points.size() <= 1) {
    return sorted_points;
  }

  // Andrew's monotone chain algorithm
  std::vector<geometry_msgs::msg::Point> hull;
  
  // Helper function to compute cross product
  auto cross = [](const geometry_msgs::msg::Point& O, 
                  const geometry_msgs::msg::Point& A, 
                  const geometry_msgs::msg::Point& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
  };

  // Build lower hull
  for (size_t i = 0; i < sorted_points.size(); ++i) {
    while (hull.size() >= 2 && cross(hull[hull.size()-2], hull[hull.size()-1], sorted_points[i]) <= 0) {
      hull.pop_back();
    }
    hull.push_back(sorted_points[i]);
  }

  // Build upper hull
  size_t t = hull.size() + 1;
  for (int i = sorted_points.size() - 2; i >= 0; --i) {
    while (hull.size() >= t && cross(hull[hull.size()-2], hull[hull.size()-1], sorted_points[i]) <= 0) {
      hull.pop_back();
    }
    hull.push_back(sorted_points[i]);
  }

  // Remove last point because it's the same as the first one
  if (hull.size() > 1) {
    hull.pop_back();
  }

  return hull;
}

float MPPICollisionChecker::findCircumscribedCost(const std::string & inflation_layer_name)
{
  if (!costmap_ros_) {
    RCLCPP_ERROR(logger_, "Costmap ROS is not available for circumscribed cost calculation");
    return -1.0f;
  }

  double result = -1.0;
  const double circum_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  const auto inflation_layer = nav2_costmap_2d::InflationLayer::getInflationLayer(
    costmap_ros_,
    inflation_layer_name);
  
  if (inflation_layer != nullptr) {
    const double resolution = costmap_ros_->getCostmap()->getResolution();
    double inflation_radius = inflation_layer->getInflationRadius();
    
    if (inflation_radius < circum_radius) {
      RCLCPP_ERROR(
        logger_,
        "The inflation radius (%f) is smaller than the circumscribed radius (%f) "
        "If this is an SE2-collision checking plugin, it cannot use costmap potential "
        "field to speed up collision checking by only checking the full footprint "
        "when robot is within possibly-inscribed radius of an obstacle. This may "
        "significantly slow down planning times!",
        inflation_radius, circum_radius);
      result = 0.0;
    } else {
      result = inflation_layer->computeCost(circum_radius / resolution);
    }
  } else {
    RCLCPP_WARN(
      logger_,
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

}  // namespace nav2_mppi_controller
