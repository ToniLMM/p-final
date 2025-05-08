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

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

class Turn : public rclcpp::Node
{
public:
  Turn();

private:
  // Callback de suscripción a YOLO
  void detectionsCallback(
    const yolo_msgs::msg::DetectionArray::SharedPtr msg);

  // Timer para ejecutar el giro y la lógica
  void onTimer();

  // Para publicar velocidad de giro
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // Suscriptor a /yolo/detections
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub_;

  // Timer de ROS
  rclcpp::TimerBase::SharedPtr timer_;

  // Momento en que arrancó el giro
  rclcpp::Time start_time_;

  // Últimas detecciones recibidas
  std::vector<yolo_msgs::msg::Detection> latest_detections_;
};

#endif  // THIEF__TURN_HPP_
