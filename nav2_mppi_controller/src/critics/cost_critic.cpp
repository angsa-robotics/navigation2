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
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

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
  getParam(trajectory_point_step_, "trajectory_point_step", 2);
  getParam(angle_quantization_bins_, "angle_quantization_bins", 72);
  angle_bin_size_ = 2.0 * M_PI / angle_quantization_bins_;

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;

  // Normalize weight when parameter is changed dynamically as well
  auto weightDynamicCb = [&](
    const rclcpp::Parameter & weight, rcl_interfaces::msg::SetParametersResult & /*result*/) {
      weight_ = weight.as_double() / 254.0f;
    };
  parameters_handler_->addParamCallback(name_ + ".cost_weight", weightDynamicCb);

  collision_checker_ = std::make_unique<nav2_smac_planner::GridCollisionChecker>(
    costmap_ros_, angle_quantization_bins_, parent_.lock());

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
    "Critic will collision check based on %s cost.",
    power_, critical_cost_, weight_, consider_footprint_ ?
    "footprint" : "circular");
}

float CostCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  const auto inflation_layer = nav2_costmap_2d::InflationLayer::getInflationLayer(
    costmap,
    inflation_layer_name_);
  if (inflation_layer != nullptr) {
    const double resolution = costmap->getCostmap()->getResolution();
    double inflation_radius = inflation_layer->getInflationRadius();
    if (inflation_radius < circum_radius) {
      RCLCPP_ERROR(
        rclcpp::get_logger("computeCircumscribedCost"),
        "The inflation radius (%f) is smaller than the circumscribed radius (%f) "
        "If this is an SE2-collision checking plugin, it cannot use costmap potential "
        "field to speed up collision checking by only checking the full footprint "
        "when robot is within possibly-inscribed radius of an obstacle. This may "
        "significantly slow down planning times!",
        inflation_radius, circum_radius);
      result = 0.0;
      return result;
    }
    result = inflation_layer->computeCost(circum_radius / resolution);
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

  int strided_traj_cols = floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
  int strided_traj_rows = data.trajectories.x.rows();
  int outer_stride = strided_traj_rows * trajectory_point_step_;

  const auto traj_x = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_y = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_yaw = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));

  collision_checker_->setFootprint(
    costmap_ros_->getRobotFootprint(),
    costmap_ros_->getUseRadius(),
    findCircumscribedCost(costmap_ros_));

  // Batch collision checking using new vectorized function
  for (int i = 0; i < strided_traj_rows; ++i) {
    bool trajectory_collide = false;
    float & traj_cost = repulsive_cost(i);

    // Prepare vectors for batch collision checking
    std::vector<float> x_coords, y_coords, angle_bins;
    x_coords.reserve(strided_traj_cols);
    y_coords.reserve(strided_traj_cols);
    angle_bins.reserve(strided_traj_cols);

    // Convert world coordinates to map coordinates and prepare angle bins
    for (int j = 0; j < strided_traj_cols; j++) {
      float Tx = traj_x(i, j);
      float Ty = traj_y(i, j);
      unsigned int x_i = 0u, y_i = 0u;

      if (!worldToMapFloat(Tx, Ty, x_i, y_i)) {
        // If any point is outside the map, mark trajectory as colliding
        traj_cost = collision_cost_;
        trajectory_collide = true;
        break;
      }

      // Get the corresponding angle bin for the trajectory point
      double orientation_bin = std::round(traj_yaw(i, j) / angle_bin_size_);
      while (orientation_bin < 0.0) {
        orientation_bin += static_cast<double>(angle_quantization_bins_);
      }
      // This is needed to handle precision issues
      if (orientation_bin >= static_cast<double>(angle_quantization_bins_)) {
        orientation_bin -= static_cast<double>(angle_quantization_bins_);
      }

      x_coords.push_back(static_cast<float>(x_i));
      y_coords.push_back(static_cast<float>(y_i));
      angle_bins.push_back(static_cast<float>(orientation_bin));
    }

    // Skip batch collision checking if trajectory already marked as colliding
    if (trajectory_collide) {
      continue;
    }

    // Perform batch collision checking
    auto collision_result = collision_checker_->inCollision(
      x_coords, y_coords, angle_bins, is_tracking_unknown_);

    if (collision_result.in_collision) {
      traj_cost = collision_cost_;
      trajectory_collide = true;
    } else {
      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // and score based on the footprint cost for more accurate trajectory evaluation
      for (size_t k = 0; k < collision_result.footprint_cost.size(); ++k) {
        float cost = collision_result.footprint_cost[k];
        if (cost >= static_cast<float>(near_collision_cost_)) {
          traj_cost += critical_cost_;
        } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
          traj_cost += cost;
        }
      }
    }

    all_trajectories_collide &= trajectory_collide;
  }

  if (power_ > 1u) {
    data.costs += (repulsive_cost *
      (weight_ / static_cast<float>(strided_traj_cols))).pow(power_);
  } else {
    data.costs += repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols));
  }

  data.fail_flag = all_trajectories_collide;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CostCritic,
  mppi::critics::CriticFunction)
