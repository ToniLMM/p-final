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

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

using namespace std::chrono_literals;

class Turn : public rclcpp::Node
{
public:
  Turn()
  : Node("turn_node")
  {
    // Publisher de velocidad angular
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    // Suscripción a detecciones YOLO
    detections_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
      "/yolo/detections", 10,
      std::bind(&Turn::detectionsCallback,
                this, std::placeholders::_1));

    // Timer a 10Hz para ejecutar onTimer()
    timer_ = this->create_wall_timer(
      100ms, std::bind(&Turn::onTimer, this));

    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "Turn arrancado: girando hasta 3s o detección.");
  }

private:
  void detectionsCallback(
    const yolo_msgs::msg::DetectionArray::SharedPtr msg)
  {
    latest_detections_ = msg->detections;

    // Imprimimos todo el array al llegar el mensaje:
    // RCLCPP_INFO(get_logger(), "=== N° de detecciones: %zu ===", latest_detections_.size());
    // for (size_t i = 0; i < latest_detections_.size(); ++i) {
    //   const auto & det = latest_detections_[i];
    //   RCLCPP_INFO(
    //     get_logger(),
    //     "Det[%zu] -> class_id: %d]",
    //     i,
    //     det.class_id);
    //}
  }

  void onTimer()
  {
    // 1) Compruebo si ya he visto una persona
    for (auto & det : latest_detections_) {
      if (det.class_id == 0) {
        RCLCPP_INFO(get_logger(), "¡Persona detectada! Deteniendo giro.");
        stopAndExit();
        return;
      }
      if (det.class_id == 32) {
        RCLCPP_INFO(get_logger(), "Pelota encontrada, emito sonido");
        // Aquí puedes llamar a tu publisher de sonido o ejecutar la lógica que quieras
      }
      // ID de mochila = 24
      if (det.class_id == 24) {
        RCLCPP_INFO(get_logger(), "Mochila encontrada, emito sonido");
        // Aquí puedes llamar a tu publisher de sonido o ejecutar la lógica que quieras
      }
      // ID de ordenador = 63
      if (det.class_id == 63) {
        RCLCPP_INFO(get_logger(), "Ordenador encontrada, emito sonido");
        // Aquí puedes llamar a tu publisher de sonido o ejecutar la lógica que quieras
      }
      // ID de botella = 39
      if (det.class_id == 39 ) {
        RCLCPP_INFO(get_logger(), "Botella encontrada, emito sonido");
        // Aquí puedes llamar a tu publisher de sonido o ejecutar la lógica que quieras
      }
    }

    // 2) Si llevo menos de 3s, sigo girando
    auto elapsed = now() - start_time_;
    if (elapsed < 8s) {
      geometry_msgs::msg::Twist vel;
      RCLCPP_INFO(get_logger(), "GIRANDO ...");
      vel.angular.z = 0.5;
      vel_pub_->publish(vel);
      return;
    }

    // 3) Si ya pasaron 3s sin detección, paro y salgo
    RCLCPP_INFO(get_logger(),
                "3 segundos sin detectar persona. Detengo giro.");
    stopAndExit();
  }

  void stopAndExit()
  {
    // Publico cero velocidad para parar el robot
    geometry_msgs::msg::Twist vel;
    vel.angular.z = 0.0;
    vel_pub_->publish(vel);

    rclcpp::shutdown();
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr
    detections_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::vector<yolo_msgs::msg::Detection> latest_detections_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Turn>();
  rclcpp::spin(node);
  return 0;
}
