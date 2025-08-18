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

#include "nav2_collision_monitor/occupancy_grid.hpp"
#include <functional>
#include <cmath>
#include "tf2/transform_datatypes.h"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_collision_monitor
{

OccupancyGridSource::OccupancyGridSource(
  const nav2::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating OccupancyGridSource", source_name_.c_str());
}

OccupancyGridSource::~OccupancyGridSource()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying OccupancyGridSource", source_name_.c_str());
  data_sub_.reset();
}

void OccupancyGridSource::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  std::string source_topic;
  getParameters(source_topic);
  data_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    source_topic,
    std::bind(&OccupancyGridSource::dataCallback, this, std::placeholders::_1));
}

bool OccupancyGridSource::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data)
{
  if (data_ == nullptr) {
    return false;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return false;
  }
  tf2::Transform tf_transform;
  if (!getTransform(curr_time, data_->header, tf_transform)) {
    return false;
  }
  // Extract occupied cells and transform to base frame
  const auto & grid = *data_;
  for (unsigned int y = 0; y < grid.info.height; ++y) {
    for (unsigned int x = 0; x < grid.info.width; ++x) {
      int idx = y * grid.info.width + x;
      if (grid.data[idx] >= occupied_threshold_) {
        double wx = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
        double wy = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;
        tf2::Vector3 p_v3_s(wx, wy, 0.0);
        tf2::Vector3 p_v3_b = tf_transform * p_v3_s;
        data.push_back({p_v3_b.x(), p_v3_b.y()});
      }
    }
  }
  return true;
}

void OccupancyGridSource::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  getCommonParameters(source_topic);
  nav2::declare_parameter_if_not_declared(
    node, source_name_ + ".occupied_threshold", rclcpp::ParameterValue(100));
  occupied_threshold_ = node->get_parameter(source_name_ + ".occupied_threshold").as_int();
}

void OccupancyGridSource::dataCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
