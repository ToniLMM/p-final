// Copyright 2021 Intelligent Robotics Lab
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
#include <iostream>

#include "thief/GetWaypoint2.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace thief
{


using namespace std::chrono_literals;

GetWaypoint2::GetWaypoint2(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

// void
// GetWaypoint2::halt()
// {
// }

BT::NodeStatus
GetWaypoint2::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  auto elapsed = node_->now() - start_time_;

  // Si no ha pasado 3s, seguimos ejecutando
  if (elapsed < 3s) {
    RCLCPP_INFO(node_->get_logger(), "GETTING WAYPOINT..."); 
    return BT::NodeStatus::RUNNING;
  }
  else{
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace thief

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<thief::GetWaypoint2>("GetWaypoint2");
}
