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

#ifndef CLOBER_HARDWARE__CLOBER_HAREWARE_HPP_
#define CLOBER_HARDWARE__CLOBER_HAREWARE_HPP_

#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "roas_base/roas_controller.hpp"
#include "roas_base/message_type.hpp"
#include "std_msgs/msg/bool.hpp"
#include "clober_msgs/msg/feedback.hpp"
#include "clober_msgs/srv/set_led.hpp"
#include "clober_hardware/visibility_control.h"

namespace clober_hardware
{
class CloberHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CloberHardware)

  CLOBER_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

  CLOBER_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CLOBER_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CLOBER_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  CLOBER_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  CLOBER_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  CLOBER_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

  /**
   * \brief Publish feedback data
   */
  void publishFeedback();

  /**
   * \brief Publish estop state
   */
  void publishEstopState();

  /// Robot hardware interface node
  std::shared_ptr<rclcpp::Node> node_;

  /// Communication system with motor controller
  std::shared_ptr<RoasController> controller_;

  /// Thread
  // std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> node_executor_;

private:
  /// Hardware interface
  std::vector<double> hw_commands_, hw_positions_, hw_velocities_;

  /// rosservice
  rclcpp::Client<clober_msgs::srv::SetLed>::SharedPtr srv_led_;

  /// rostopic
  rclcpp::Publisher<clober_msgs::msg::Feedback>::SharedPtr pub_feedback_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_;
  std::shared_ptr<realtime_tools::RealtimePublisher<clober_msgs::msg::Feedback>> rp_feedback_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> rp_estop_;

  /// Serial parameters
  std::string port_;
  int baud_;

  /// Robot name
  std::string robot_;

  /// Robot parameters
  double wheel_radius_, max_speed_, max_rpm_;

  /// Command
  Command cmd_;

  /// Feedback related
  Feedback feedback_;

  /// Estop state
  std_msgs::msg::Bool estop_state_;

  /// Previous state of Estop
  bool previous_state_;
};
}  // namespace clober_hardware

#endif  // CLOBER_HARDWARE__CLOBER_HAREWARE_HPP_