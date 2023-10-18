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

#ifndef FORMER_TELEOP__TELEOP_JOY_HPP_
#define FORMER_TELEOP__TELEOP_JOY_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define SPEED_UP 0
#define SPEED_DOWN 1

using namespace std;

class FormerTeleop : public rclcpp::Node
{
public:
  FormerTeleop(const string& node);

  virtual ~FormerTeleop() = default;

  /**
   * \brief Joystick callback
   * \param msg Joystick message
   */
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr& msg);

  /**
   * \brief Increase or decrease the maximum speed according to the button being pressed
   * \param msg Joystick message
   */
  void changeSpeed(const sensor_msgs::msg::Joy::SharedPtr& msg);

  /**
   * \brief Publish the velocity command according to joystick axis values
   * \param msg Joystick message
   */
  void publishCmdVel(const sensor_msgs::msg::Joy::SharedPtr& msg);

  /**
   * \brief Watchdog tracks the joystick message
   */
  void watchdog();

private:
  /// ROS2 parameters
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;

  /// Time stamp
  std::chrono::steady_clock::time_point last_time_;

  /// Joystick axes
  int axis_linear_x_, axis_linear_y_, axis_angular_;

  /// Joystick buttons
  int speed_up_, speed_down_, deadman_;

  /// Maximum values
  double max_linear_x_, max_linear_y_, max_angular_, cur_scale_;

  /// Button state
  bool pressedButton_[2];

  /// Watchdog related
  bool watchdog_;
};

#endif  // FORMER_TELEOP__TELEOP_JOY_HPP_