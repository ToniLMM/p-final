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

#ifndef THIEF__TURN_HPP_
#define THIEF__TURN_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "yolo_msgs/msg/detection_array.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace thief
{

class Turn : public BT::ActionNodeBase
{
public:
  explicit Turn(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  void detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);

  void playSound(uint8_t sound_type);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_subscription_;
  rclcpp::Clock clock_;

  std::vector<yolo_msgs::msg::Detection> latest_detections_;
  kobuki_ros_interfaces::msg::Sound sonido1_;
  kobuki_ros_interfaces::msg::Sound sonido2_;
  kobuki_ros_interfaces::msg::Sound sonido3_;
  kobuki_ros_interfaces::msg::Sound sonido4_;
  kobuki_ros_interfaces::msg::Sound sonido5_;
};

}  // namespace thief

#endif  // THIEF__TURN_HPP_
