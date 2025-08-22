// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <cmath>
#include <vector>
#include <unordered_set>
#include <utility>
#include <algorithm>
#include <limits>
#include <boost/functional/hash.hpp>
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace mppi::critics
{

void CostCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(enforce_path_inversion_, "enforce_path_inversion", false);

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.81f);
  getParam(critical_cost_, "critical_cost", 300.0f);
  getParam(near_collision_cost_, "near_collision_cost", 253);
  getParam(collision_cost_, "collision_cost", 1000000.0f);
  getParam(near_goal_distance_, "near_goal_distance", 0.5f);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));
  getParam(swept_area_sample_density_, "swept_area_sample_density", 1.0f);  // Lower default since using all points

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;

  // Normalize weight when parameter is changed dynamically as well
  auto weightDynamicCb = [&](
    const rclcpp::Parameter & weight, rcl_interfaces::msg::SetParametersResult & /*result*/) {
      weight_ = weight.as_double() / 254.0f;
    };
  parameters_handler_->addParamCallback(name_ + ".cost_weight", weightDynamicCb);

  collision_checker_ = std::make_unique<nav2_mppi_controller::MPPICollisionChecker>(
    costmap_ros_, parent_.lock());

  if (costmap_ros_->getUseRadius() == consider_footprint_) {
    RCLCPP_WARN(
    logger_,
    "Inconsistent configuration in collision checking. Please verify the robot's shape settings "
    "in both the costmap and the cost critic.");
    if (costmap_ros_->getUseRadius()) {
      throw nav2_core::ControllerException(
      "Considering footprint in collision checking but no robot footprint provided in the "
      "costmap.");
    }
  }

  if(near_collision_cost_ > 253) {
    RCLCPP_WARN(logger_, "Near collision cost is set higher than INSCRIBED_INFLATED_OBSTACLE");
  }

  RCLCPP_INFO(
    logger_,
    "InflationCostCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost. "
    "Using all trajectory points with swept area sample density: %f",
    power_, critical_cost_, weight_, consider_footprint_ ?
    "footprint" : "circular", swept_area_sample_density_);
}

void CostCritic::getFootprintCellsAtPose(float world_x, float world_y, float yaw,
                                         std::vector<std::pair<unsigned int, unsigned int>>& cells) const
{
  cells.clear(); // Reuse the provided vector
  
  if (consider_footprint_) {
    // Get robot footprint and transform to world coordinates
    auto footprint = costmap_ros_->getRobotFootprint();
    
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);
    
    // Pre-allocate transformed footprint vector
    std::vector<std::pair<float, float>> transformed_footprint;
    transformed_footprint.reserve(footprint.size());
    
    // Vectorized approach for bounding box calculation
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    
    // Transform footprint vertices using vectorized operations
    for (const auto& point : footprint) {
      const float transformed_x = world_x + (point.x * cos_yaw - point.y * sin_yaw);
      const float transformed_y = world_y + (point.x * sin_yaw + point.y * cos_yaw);
      
      transformed_footprint.emplace_back(transformed_x, transformed_y);
      
      // Update bounding box efficiently
      min_x = std::min(min_x, transformed_x);
      max_x = std::max(max_x, transformed_x);
      min_y = std::min(min_y, transformed_y);
      max_y = std::max(max_y, transformed_y);
    }
    
    // Convert bounding box to map coordinates
    unsigned int min_mx, min_my, max_mx, max_my;
    if (worldToMapFloat(min_x, min_y, min_mx, min_my) && 
        worldToMapFloat(max_x, max_y, max_mx, max_my)) {
      
      // Pre-compute constants for coordinate conversion
      const float origin_x_offset = origin_x_ - 0.5f * resolution_;
      const float origin_y_offset = origin_y_ - 0.5f * resolution_;
      
      // Optimized nested loop with bounds checking
      for (unsigned int mx = min_mx; mx <= max_mx && mx < size_x_; mx++) {
        const float cell_world_x = origin_x_offset + mx * resolution_;
        
        for (unsigned int my = min_my; my <= max_my && my < size_y_; my++) {
          const float cell_world_y = origin_y_offset + my * resolution_;
          
          // Optimized point-in-polygon test (ray casting)
          bool inside = false;
          for (size_t i = 0, j = transformed_footprint.size() - 1; i < transformed_footprint.size(); j = i++) {
            const auto& p1 = transformed_footprint[i];
            const auto& p2 = transformed_footprint[j];
            
            if (((p1.second > cell_world_y) != (p2.second > cell_world_y)) &&
                (cell_world_x < (p2.first - p1.first) * (cell_world_y - p1.second) / (p2.second - p1.second) + p1.first)) {
              inside = !inside;
            }
          }
          
          if (inside) {
            cells.emplace_back(mx, my);
          }
        }
      }
    }
  } else {
    // Optimized circular footprint calculation
    const float radius = costmap_ros_->getRobotRadius();
    const int cell_radius = static_cast<int>(std::ceil(radius / resolution_));
    const int cell_radius_squared = cell_radius * cell_radius;
    
    unsigned int center_x, center_y;
    if (worldToMapFloat(world_x, world_y, center_x, center_y)) {
      // Pre-allocate space based on circle area estimate
      const int estimated_cells = static_cast<int>(M_PI * cell_radius_squared) + 1;
      cells.reserve(estimated_cells);
      
      // Optimized circle rasterization
      for (int dx = -cell_radius; dx <= cell_radius; dx++) {
        const int dx_squared = dx * dx;
        for (int dy = -cell_radius; dy <= cell_radius; dy++) {
          if (dx_squared + dy * dy <= cell_radius_squared) {
            const unsigned int mx = center_x + dx;
            const unsigned int my = center_y + dy;
            if (mx < size_x_ && my < size_y_) {
              cells.emplace_back(mx, my);
            }
          }
        }
      }
    }
  }
}

std::unordered_set<std::pair<unsigned int, unsigned int>, 
                   boost::hash<std::pair<unsigned int, unsigned int>>>
CostCritic::getSweptAreaCells(const std::vector<float>& x_coords,
                              const std::vector<float>& y_coords, 
                              const std::vector<float>& yaw_angles) const
{
  std::unordered_set<std::pair<unsigned int, unsigned int>, 
                     boost::hash<std::pair<unsigned int, unsigned int>>> swept_cells;
  
  if (x_coords.size() < 2) {
    return swept_cells;
  }
  
  // Pre-compute sampling resolution and constants
  const float sample_resolution = resolution_ / swept_area_sample_density_;
  const float inv_sample_resolution = 1.0f / sample_resolution;
  
  // Pre-allocate vectors for footprint cells to reduce repeated allocations
  std::vector<std::pair<unsigned int, unsigned int>> temp_footprint_cells;
  temp_footprint_cells.reserve(64); // Typical footprint size estimate
  
  // Use Eigen for vectorized segment processing where possible
  const size_t n_segments = x_coords.size() - 1;
  
  for (size_t i = 0; i < n_segments; i++) {
    // Convert from map coordinates to world coordinates
    const float start_x = (x_coords[i] + 0.5f) * resolution_ + origin_x_;
    const float start_y = (y_coords[i] + 0.5f) * resolution_ + origin_y_;
    const float start_yaw = yaw_angles[i];
    const float end_x = (x_coords[i + 1] + 0.5f) * resolution_ + origin_x_;
    const float end_y = (y_coords[i + 1] + 0.5f) * resolution_ + origin_y_;
    const float end_yaw = yaw_angles[i + 1];

    // Vectorized segment parameter calculation
    const float dx = end_x - start_x;
    const float dy = end_y - start_y;
    float dyaw = end_yaw - start_yaw;
    
    // Normalize yaw difference efficiently
    dyaw = std::fmod(dyaw + 3.0f * M_PI, 2.0f * M_PI) - M_PI;
    
    const float segment_length = std::sqrt(dx * dx + dy * dy);
    
    if (segment_length < 1e-6) {
      // Handle stationary case with pre-allocated vector
      temp_footprint_cells.clear();
      getFootprintCellsAtPose(start_x, start_y, start_yaw, temp_footprint_cells);
      swept_cells.insert(temp_footprint_cells.begin(), temp_footprint_cells.end());
      continue;
    }

    // Compute number of samples more efficiently
    const int num_samples = std::max(1, static_cast<int>(segment_length * inv_sample_resolution));
    const float inv_num_samples = 1.0f / static_cast<float>(num_samples);
    
    // Vectorized sampling with reduced function calls
    for (int s = 0; s <= num_samples; s++) {
      const float t = static_cast<float>(s) * inv_num_samples;
      const float sample_x = start_x + t * dx;
      const float sample_y = start_y + t * dy;
      const float sample_yaw = start_yaw + t * dyaw;
      
      // Use optimized footprint calculation
      temp_footprint_cells.clear();
      getFootprintCellsAtPose(sample_x, sample_y, sample_yaw, temp_footprint_cells);
      swept_cells.insert(temp_footprint_cells.begin(), temp_footprint_cells.end());
    }
  }
  
  return swept_cells;
}

void CostCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  geometry_msgs::msg::Pose goal = utils::getCriticGoal(data, enforce_path_inversion_);

  // Setup cost information for various parts of the critic
  is_tracking_unknown_ = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  auto * costmap = collision_checker_->getCostmap();
  origin_x_ = static_cast<float>(costmap->getOriginX());
  origin_y_ = static_cast<float>(costmap->getOriginY());
  resolution_ = static_cast<float>(costmap->getResolution());
  size_x_ = costmap->getSizeInCellsX();
  size_y_ = costmap->getSizeInCellsY();

  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, goal)) {
    near_goal = true;
  }

  Eigen::ArrayXf repulsive_cost(data.costs.rows());
  repulsive_cost.setZero();
  bool all_trajectories_collide = true;


  int traj_cols = data.trajectories.x.cols();
  int traj_rows = data.trajectories.x.rows();

  // Direct reference to trajectory data
  const auto& traj_x = data.trajectories.x;
  const auto& traj_y = data.trajectories.y;
  const auto& traj_yaw = data.trajectories.yaws;

  collision_checker_->setFootprint(
    costmap_ros_->getRobotFootprint(),
    costmap_ros_->getUseRadius(),
    inflation_layer_name_);

  // Pre-allocate vectors to avoid repeated allocations
  std::vector<float> x_coords, y_coords, yaw_angles;
  x_coords.reserve(traj_cols);
  y_coords.reserve(traj_cols);
  yaw_angles.reserve(traj_cols);
  
  // Pre-compute coordinate conversion constants for vectorization
  const float inv_resolution = 1.0f / resolution_;
  
  auto * swept_costmap = collision_checker_->getCostmap();
  const unsigned char* costmap_data = swept_costmap->getCharMap();
  
  // Use swept area approach with all trajectory points
  for (int i = 0; i < traj_rows; ++i) {
    bool trajectory_collide = false;
    float & traj_cost = repulsive_cost(i);

    // Clear and reuse vectors instead of reallocating
    x_coords.clear();
    y_coords.clear();
    yaw_angles.clear();

    // Vectorized coordinate conversion using Eigen
    Eigen::ArrayXf world_x = traj_x.row(i);
    Eigen::ArrayXf world_y = traj_y.row(i);
    Eigen::ArrayXf world_yaw = traj_yaw.row(i);
    
    // Convert to map coordinates using vectorized operations
    Eigen::ArrayXf map_x = (world_x - origin_x_) * inv_resolution;
    Eigen::ArrayXf map_y = (world_y - origin_y_) * inv_resolution;
    
    // Check bounds vectorized
    Eigen::Array<bool, Eigen::Dynamic, 1> in_bounds = 
      (map_x >= 0.0f) && (map_x < static_cast<float>(size_x_)) && 
      (map_y >= 0.0f) && (map_y < static_cast<float>(size_y_));
    
    // Early termination if any point is out of bounds
    if (!in_bounds.all()) {
      traj_cost = collision_cost_;
      trajectory_collide = true;
      all_trajectories_collide &= trajectory_collide;
      continue;
    }

    // Convert to integer coordinates and store in vectors
    for (int j = 0; j < traj_cols; j++) {
      x_coords.push_back(map_x(j));
      y_coords.push_back(map_y(j));
      yaw_angles.push_back(world_yaw(j));
    }

    // Get all cells swept by footprint along trajectory
    auto swept_cells = getSweptAreaCells(x_coords, y_coords, yaw_angles);
    
    if (swept_cells.empty()) {
      continue;
    }

    // Vectorized cost checking using direct memory access
    float total_cost = 0.0f;
    int high_cost_count = 0;
    int cell_count = 0;
    
    for (const auto& cell : swept_cells) {
      unsigned int mx = cell.first;
      unsigned int my = cell.second;
      
      // Direct memory access instead of getCost() function call
      unsigned char cell_cost = costmap_data[my * size_x_ + mx];
      
      if (cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        traj_cost = collision_cost_;
        trajectory_collide = true;
        break;
      } else if (cell_cost >= static_cast<unsigned char>(near_collision_cost_)) {
        high_cost_count++;
      } else if (!near_goal) {
        total_cost += static_cast<float>(cell_cost);
      }
      
      // Early termination for unknown cells if tracking unknown
      if (is_tracking_unknown_ && cell_cost == nav2_costmap_2d::NO_INFORMATION) {
        traj_cost = collision_cost_;
        trajectory_collide = true;
        break;
      }
      
      cell_count++;
    }

    if (!trajectory_collide && cell_count > 0) {
      // Calculate average cost with critical cost penalty
      float inv_cell_count = 1.0f / static_cast<float>(cell_count);
      traj_cost = total_cost * inv_cell_count + static_cast<float>(high_cost_count) * critical_cost_ * inv_cell_count;
    }

    all_trajectories_collide &= trajectory_collide;
  }

  if (power_ > 1u) {
    data.costs += (repulsive_cost *
      (weight_ / static_cast<float>(traj_cols))).pow(power_);
  } else {
    data.costs += repulsive_cost * (weight_ / static_cast<float>(traj_cols));
  }

  data.fail_flag = all_trajectories_collide;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CostCritic,
  mppi::critics::CriticFunction)
