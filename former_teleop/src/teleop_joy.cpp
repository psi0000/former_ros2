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

#include "former_teleop/teleop_joy.hpp"

FormerTeleop::FormerTeleop(const std::string& node)
  : Node(node)
  , axis_linear_x_(1)
  , axis_linear_y_(0)
  , axis_angular_(3)
  , speed_up_(2)
  , speed_down_(0)
  , deadman_(4)
  , max_linear_x_(1.5)
  , max_linear_y_(1.5)
  , max_angular_(2.0)
  , cur_scale_(0.4)
  , watchdog_(false)
{
  this->declare_parameter("axis_linear_x", rclcpp::ParameterValue(1));
  this->declare_parameter("axis_linear_y", rclcpp::ParameterValue(0));
  this->declare_parameter("axis_angular", rclcpp::ParameterValue(3));
  this->declare_parameter("speed_up", rclcpp::ParameterValue(2));
  this->declare_parameter("speed_down", rclcpp::ParameterValue(0));
  this->declare_parameter("deadman", rclcpp::ParameterValue(4));
  this->declare_parameter("max_linear_x", rclcpp::ParameterValue(1.5));
  this->declare_parameter("max_linear_y_", rclcpp::ParameterValue(1.5));
  this->declare_parameter("max_angular", rclcpp::ParameterValue(2.0));

  this->get_parameter("axis_linear_x", axis_linear_x_);
  this->get_parameter("axis_linear_y", axis_linear_y_);
  this->get_parameter("axis_angular", axis_angular_);
  this->get_parameter("speed_up", speed_up_);
  this->get_parameter("speed_down", speed_down_);
  this->get_parameter("deadman", deadman_);
  this->get_parameter("max_linear_x", max_linear_x_);
  this->get_parameter("max_linear_y_", max_linear_y_);
  this->get_parameter("max_angular", max_angular_);

  sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(10), [=](const sensor_msgs::msg::Joy::SharedPtr msg) { joyCallback(msg); });
  pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  for (int i = 0; i < 2; i++)
    pressedButton_[i] = false;

  last_time_ = std::chrono::steady_clock::now();
}

void FormerTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr& msg)
{
  if (msg->buttons[deadman_])
  {
    last_time_ = std::chrono::steady_clock::now();

    changeSpeed(msg);
    publishCmdVel(msg);
  }
}

void FormerTeleop::changeSpeed(const sensor_msgs::msg::Joy::SharedPtr& msg)
{
  if (msg->buttons[speed_up_])
  {
    if (!pressedButton_[SPEED_UP])
    {
      pressedButton_[SPEED_UP] = true;
      cur_scale_ += 0.2;
      if (cur_scale_ >= 1.0)
        cur_scale_ = 1.0;
    }
  }
  else if (msg->buttons[speed_down_])
  {
    if (!pressedButton_[SPEED_DOWN])
    {
      pressedButton_[SPEED_DOWN] = true;
      cur_scale_ -= 0.2;
      if (cur_scale_ <= 0.0)
        cur_scale_ = 0.0;
    }
  }
  else
  {
    pressedButton_[SPEED_UP] = false;
    pressedButton_[SPEED_DOWN] = false;
  }
}

void FormerTeleop::publishCmdVel(const sensor_msgs::msg::Joy::SharedPtr& msg)
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  vel.linear.x = cur_scale_ * max_linear_x_ * msg->axes[axis_linear_x_];
  vel.angular.z = cur_scale_ * max_angular_ * msg->axes[axis_angular_];

  pub_vel_->publish(vel);
}

void FormerTeleop::watchdog()
{
  std::chrono::steady_clock::time_point cur_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = cur_time - last_time_;
  const double elapsed_time = elapsed.count();

  if (elapsed_time < 0.5)
    watchdog_ = false;
  else if (elapsed_time > 0.5 && watchdog_ == false)
  {
    watchdog_ = true;
    geometry_msgs::msg::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    pub_vel_->publish(vel);
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormerTeleop>("teleop_node");
  rclcpp::WallRate rate(30.0);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->watchdog();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}