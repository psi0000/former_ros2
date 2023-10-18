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

#include "former_hardware/former_hardware.hpp"

namespace former_hardware
{
hardware_interface::return_type FormerHardware::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  port_ = info_.hardware_parameters["port"];
  baud_ = stoi(info_.hardware_parameters["baud"]);
  robot_ = info_.hardware_parameters["robot"];
  wheel_radius_ = stod(info_.hardware_parameters["wheel_radius"]);
  max_speed_ = stod(info_.hardware_parameters["max_speed"]);
  max_rpm_ = stod(info_.hardware_parameters["max_rpm"]);

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("FormerHardware"), "Joint '%s' has %d command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("FormerHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("FormerHardware"), "Joint '%s' has %d state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("FormerHardware"),
                   "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("FormerHardware"),
                   "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> FormerHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FormerHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type FormerHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("RoasHardware"), "Starting ...please wait...");

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (isnan(hw_commands_[i]))
      hw_commands_[i] = 0.0;

    if (isnan(hw_positions_[i]))
      hw_positions_[i] = 0.0;

    if (isnan(hw_velocities_[i]))
      hw_velocities_[i] = 0.0;
  }

  // Initialize node
  node_ = std::make_shared<rclcpp::Node>("former_hardawre_node");
  controller_ = std::make_shared<RoasController>(node_, robot_, info_.joints.size(), wheel_radius_, max_speed_,
                                                 max_rpm_, port_, baud_, feedback_, estop_state_);
  cmd_.command.assign(info_.joints.size(), 0.0);

  // Initialize client
  srv_led_ = node_->create_client<former_msgs::srv::SetLed>("/led/control");

  // Initialize publisher
  pub_feedback_ = node_->create_publisher<former_msgs::msg::Feedback>(robot_ + "/feedback", 1);
  rp_feedback_ = std::make_shared<realtime_tools::RealtimePublisher<former_msgs::msg::Feedback>>(pub_feedback_);
  rp_feedback_->msg_.robot = robot_;

  for (size_t i = 0; i < ceil(info_.joints.size() / 2.0); i++)
  {
    rp_feedback_->msg_.motor_state.push_back(former_msgs::msg::MotorState());
    feedback_.motor_state.push_back(MotorState());
  }

  pub_estop_ = node_->create_publisher<std_msgs::msg::Bool>(robot_ + "/emergency_stop", 1);
  rp_estop_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(pub_estop_);
  rp_estop_->msg_.data = false;

  // Initial setting for serial communication
  if (!controller_->init())
    return hardware_interface::return_type::ERROR;

  if (!controller_->connect())
    return hardware_interface::return_type::ERROR;

  controller_->restartScript();

  auto req = std::make_shared<former_msgs::srv::SetLed::Request>();
  req->color = former_msgs::srv::SetLed::Request::MINT;
  req->mode = former_msgs::srv::SetLed::Request::NORMAL;
  srv_led_->async_send_request(req);

  status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(rclcpp::get_logger("FormerHardware"), "System Successfully started!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FormerHardware::stop()
{
  status_ = hardware_interface::status::STOPPED;
  controller_->serial_->close();
  RCLCPP_INFO(rclcpp::get_logger("FormerHardware"), "System successfully stopped!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FormerHardware::read()
{
  controller_->read();

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_positions_[i] = feedback_.motor_state[i / 2].position[i % 2];
    hw_velocities_[i] = feedback_.motor_state[i / 2].velocity[i % 2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FormerHardware::write()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
    cmd_.command[i] = hw_commands_[i];

  controller_->sendHearbeat();
  controller_->sendCommand(cmd_);

  publishFeedback();
  publishEstopState();

  return hardware_interface::return_type::OK;
}

void FormerHardware::publishFeedback()
{
  if (rp_feedback_->trylock())
  {
    rp_feedback_->msg_.header.stamp = rclcpp::Clock().now();
    rp_feedback_->msg_.robot_state.emergency_stop = feedback_.robot_state.emergency_stop;
    rp_feedback_->msg_.robot_state.battery_voltage = feedback_.robot_state.battery_voltage;
    rp_feedback_->msg_.robot_state.charging_voltage = feedback_.robot_state.charging_voltage;
    rp_feedback_->msg_.robot_state.user_12v_current = feedback_.robot_state.user_12v_current;
    rp_feedback_->msg_.robot_state.user_24v_current = feedback_.robot_state.user_24v_current;

    for (size_t i = 0; i < ceil(info_.joints.size() / 2.0); i++)
    {
      rp_feedback_->msg_.motor_state[i].position = feedback_.motor_state[i].position;
      rp_feedback_->msg_.motor_state[i].velocity = feedback_.motor_state[i].velocity;
      rp_feedback_->msg_.motor_state[i].current = feedback_.motor_state[i].current;
      rp_feedback_->msg_.motor_state[i].temperature = feedback_.motor_state[i].temperature;
      rp_feedback_->msg_.motor_state[i].fault_flags = feedback_.motor_state[i].fault_flags;
    }
    rp_feedback_->unlockAndPublish();
  }
}

void FormerHardware::publishEstopState()
{
  if (estop_state_.data != previous_state_)
  {
    if (rp_estop_->trylock())
    {
      rp_estop_->msg_ = estop_state_;
      rp_estop_->unlockAndPublish();

      if (estop_state_.data)
      {
        auto req = std::make_shared<former_msgs::srv::SetLed::Request>();
        req->color = former_msgs::srv::SetLed::Request::RED;
        req->mode = former_msgs::srv::SetLed::Request::BLINK;
        srv_led_->async_send_request(req);
      }
      else
      {
        auto req = std::make_shared<former_msgs::srv::SetLed::Request>();
        req->color = former_msgs::srv::SetLed::Request::MINT;
        req->mode = former_msgs::srv::SetLed::Request::NORMAL;
        srv_led_->async_send_request(req);
      }
    }
    previous_state_ = rp_estop_->msg_.data;
  }
}
}  // namespace former_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(former_hardware::FormerHardware, hardware_interface::SystemInterface)