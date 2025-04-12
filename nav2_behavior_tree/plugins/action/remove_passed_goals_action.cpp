// Copyright (c) 2021 Samsung Research America
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

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5)
{}

void RemovePassedGoals::initialize()
{
  getInput("radius", viapoint_achieved_radius_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
}

inline BT::NodeStatus RemovePassedGoals::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  getInput("nb_goals_to_consider", nb_goals_to_consider_);
  nb_goals_to_consider_ = std::min(nb_goals_to_consider_, static_cast<int>(goal_poses.size()));

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal_poses[0].pose.header.frame_id, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  Goals goals_feedback;
  [[maybe_unused]] auto res = config().blackboard->get("goals_feedback", goals_feedback);

  bool goal_reached = false;
  int reached_goal_index = -1;

  // Iterate over the first `nb_goals_to_consider` goals
  for (int i = 0; i < nb_goals_to_consider_; ++i) {
    double dist_to_goal = euclidean_distance(goal_poses[i].pose.pose, current_pose.pose);
    if (dist_to_goal <= viapoint_achieved_radius_) {
      reached_goal_index = i;
      goal_reached = true;
      break;
    }
  }

  if (goal_reached) {
    // Mark reached goal as VISITED and all previous ones as SKIPPED
    for (int i = 0; i <= reached_goal_index; ++i) {
      goals_feedback[goal_poses[i].index].status = 
          (i == reached_goal_index) ? nav2_msgs::msg::Waypoint::VISITED
                                    : nav2_msgs::msg::Waypoint::SKIPPED;
    }
    // If the reached goal is NOT the last one, erase all VISITED and SKIPPED goals
    if (reached_goal_index < static_cast<int>(goal_poses.size()) - 1) {
      goal_poses.erase(goal_poses.begin(), goal_poses.begin() + reached_goal_index + 1);
    }
  }
  config().blackboard->set("goals_feedback", goals_feedback);
  setOutput("output_goals", goal_poses);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
