// Copyright (c) 2021, ROAS Inc.
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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "roas_base/roas_gpio.hpp"
#include "clober_msgs/srv/set_led.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoasGPIO>("clober_gpio_node");

  rclcpp::Service<clober_msgs::srv::SetLed>::SharedPtr srv_led_ = node->create_service<clober_msgs::srv::SetLed>(
      "/led/control",
      [&node](const std::shared_ptr<clober_msgs::srv::SetLed::Request> req,
              std::shared_ptr<clober_msgs::srv::SetLed::Response> resp) -> void {
        node->setLed(req->color, req->mode);
        resp->success = true;
      });

  node->init();

  if (node->connect())
  {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    thread t([&executor]() -> void { executor->spin(); });

    while (rclcpp::ok())
      node->read();
  }

  rclcpp::shutdown();
  return 0;
}