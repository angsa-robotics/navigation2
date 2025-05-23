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
  getInput("nb_goals_to_consider", nb_goals_to_consider_);
  getInput("update_status", update_status_);

  if (input_goals_.empty()) {
    setOutput("output_goals", input_goals_);
    should_send_request_ = false;
    return;
  }
  request_ = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  request_->use_footprint = use_footprint_;

  for (size_t i = 0; i < static_cast<size_t>(nb_goals_to_consider_) && i < input_goals_.size(); ++i) {
    // create a copy of the goal and set the timestamp to the current time (Angsa Robotics Hack)
    nav2_msgs::msg::Waypoint goal_copy = input_goals_[i];
    goal_copy.pose.header.stamp = node_->now(); 
    request_->poses.push_back(goal_copy.pose);
  }
}

BT::NodeStatus RemoveInCollisionGoals::on_completion(
  std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
{
  Goals goals_feedback;
  [[maybe_unused]] auto res = config().blackboard->get("goals_feedback", goals_feedback);
  for (int i = static_cast<int>(response->costs.size()) - 1; i >= 0; --i) {
    if ((response->costs[i] != 255 || consider_unknown_as_obstacle_) && response->costs[i] >= cost_threshold_) {
      if (update_status_) {
        goals_feedback[input_goals_[i].index].status = nav2_msgs::msg::Waypoint::SKIPPED;
      }
      // if it's not valid then we erase it from input_goals_ and set the status to SKIPPED
      input_goals_.erase(input_goals_.begin() + i);
    }
  }
  
  config().blackboard->set("goals_feedback", goals_feedback);
  setOutput("output_goals", input_goals_);
  return BT::NodeStatus::SUCCESS;
}

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemoveInCollisionGoals>("RemoveInCollisionGoals");
}
