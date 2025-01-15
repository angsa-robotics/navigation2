// Copyright (c) 2024 Angsa Robotics
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

#include <string>
#include <memory>
#include <limits>

#include "nav2_behavior_tree/plugins/action/remove_in_collision_goals_action.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_behavior_tree
{

RemoveInCollisionGoals::RemoveInCollisionGoals(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::GetCosts>(service_node_name, conf,
    "/global_costmap/get_cost_global_costmap")
{}


void RemoveInCollisionGoals::on_tick()
{
  getInput("use_footprint", use_footprint_);
  getInput("cost_threshold", cost_threshold_);
  getInput("input_goals", input_goals_);
  getInput("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);

  if (input_goals_.empty()) {
    setOutput("output_goals", input_goals_);
    should_send_request_ = false;
    return;
  }
  request_ = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  request_->use_footprint = use_footprint_;

  for (const auto & goal : input_goals_) {
    // create a copy of the goal and set the timestamp to the current time (Angsa Robotics Hack)
    geometry_msgs::msg::PoseStamped goal_copy = goal;
    goal_copy.header.stamp = node_->now(); 
    request_->poses.push_back(goal_copy);
  }
}

BT::NodeStatus RemoveInCollisionGoals::on_completion(
  std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
{
  Goals valid_goal_poses;
  Goals skipped_goals;
  for (size_t i = 0; i < response->costs.size(); ++i) {
    if ((response->costs[i] == 255 && !consider_unknown_as_obstacle_) || response->costs[i] < cost_threshold_) {
      valid_goal_poses.push_back(input_goals_[i]);
    } else {
      skipped_goals.push_back(input_goals_[i]);
    }
  }
  // Inform if all goals have been removed
  if (valid_goal_poses.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "All goals are in collision and have been removed from the list");
  }

  Goals existing_skipped_poses;
  [[maybe_unused]] auto res = config().blackboard->get("skipped_poses", existing_skipped_poses);
  // Append the skipped goals to the skipped poses if they are not already there
  for (const auto & skipped_goal : skipped_goals) {
    if (std::find(existing_skipped_poses.begin(), existing_skipped_poses.end(), skipped_goal) == existing_skipped_poses.end()) {
      existing_skipped_poses.push_back(skipped_goal);
    }
  }
  config().blackboard->set("skipped_poses", existing_skipped_poses);

  setOutput("output_goals", valid_goal_poses);
  return BT::NodeStatus::SUCCESS;
}

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemoveInCollisionGoals>("RemoveInCollisionGoals");
}
