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

#include "thief/Turn.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace thief
{

// Este nodo devuelve SUCCESS si detecta una persona

using namespace std::chrono_literals;

Turn::Turn(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);
  sound_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("commands/sound", 10);

  detections_subscription_ = node_->create_subscription<yolo_msgs::msg::DetectionArray>(
    "/yolo/detections", 10, std::bind(&Turn::detectionsCallback, this, std::placeholders::_1));

  sonido1_.value = 5;
  sonido2_.value = 6;
  sonido3_.value = 3;
  sonido4_.value = 4;
  sonido5_.value = 5;
}

void Turn::detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
  // Guardamos sólo las detecciones, para consultarlas en tick()
  latest_detections_ = msg->detections;
}

void
Turn::halt()
{
}

// void
// Turn::playSound(uint8_t sound_type)
// {
//   kobuki_msgs::msg::Sound sound_msg;
//   sound_msg.value = sound_type;
//   sound_pub_->publish(sound_msg);
// }

BT::NodeStatus
Turn::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
    latest_detections_.clear();
  }

  auto elapsed = node_->now() - start_time_;

  // Si no ha pasado 8s, seguimos girando
  if (elapsed < 8s) {
    geometry_msgs::msg::Twist vel_msgs;
    vel_msgs.angular.z = 0.5;
    vel_pub_->publish(vel_msgs);
    RCLCPP_INFO(node_->get_logger(), "TURNING...");

    // Procesamiento de las detecciones de yolo
    for (auto & det : latest_detections_) {
      // ID de person = 0
      if (det.class_id == 0) {
        vel_msgs.angular.z = 0.0;
        vel_pub_->publish(vel_msgs);
        RCLCPP_INFO(node_->get_logger(), "PERSON DETECTED!");
        sound_->publish(sonido1_);
          
        return BT::NodeStatus::SUCCESS;
      }
      // ID de pelota = 32
      if (det.class_id == 32) {
        // vel_msgs.angular.z = 0.0;
        // vel_pub_->publish(vel_msgs);
        RCLCPP_INFO(node_->get_logger(), "BALL DETECTED!");
        sound_->publish(sonido2_);
      }
      // ID de mochila = 26
      if (det.class_id == 26) {
        RCLCPP_INFO(node_->get_logger(), "BAG DETECTED!");
        sound_->publish(sonido3_);
      }
      // ID de ordenador = 63
      if (det.class_id == 63) {
        RCLCPP_INFO(node_->get_logger(), "LAPTOP DETECTED!");
        sound_->publish(sonido4_);
      }
      // ID de botella = 39
      if (det.class_id == 39 ) {
        RCLCPP_INFO(node_->get_logger(), "BOTTLE DETECTED!");
        sound_->publish(sonido5_);
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  else{
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace thief

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<thief::Turn>("Turn");
}
