// Copyright (c) 2025 Angsa Robotics
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

#ifndef NAV2_COLLISION_MONITOR__OCCUPANCY_GRID_HPP_
#define NAV2_COLLISION_MONITOR__OCCUPANCY_GRID_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_collision_monitor/source.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_collision_monitor
{

class OccupancyGridSource : public Source
{
public:
  OccupancyGridSource(
    const nav2::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);
  ~OccupancyGridSource();

  void configure();
  bool getData(const rclcpp::Time & curr_time, std::vector<Point> & data) override;
  void getParameters(std::string & source_topic);

private:
  void dataCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr data_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr data_sub_;
  int occupied_threshold_;
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__OCCUPANCY_GRID_HPP_
