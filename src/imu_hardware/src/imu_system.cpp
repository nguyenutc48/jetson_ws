// Copyright 2021 ros2_control Development Team
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

#include "imu_hardware/imu_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace imu_hardware
{
hardware_interface::return_type IMUHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  imu_sensor_state_.resize(info_.sensors[0].state_interfaces.size(), 0.0);

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> IMUHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++)
    // {
    //   state_interfaces.emplace_back(
    //       StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_sensor_state_[i]));
    // }

    return state_interfaces;
}

hardware_interface::return_type IMUHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Starting ...please wait...");



  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IMUHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Stopping ...please wait...");


  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IMUHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Reading...");

  imu_sensor_state_[0] = 0.0;
  imu_sensor_state_[1] = 0.0;
  imu_sensor_state_[2] = 0.0;
  imu_sensor_state_[3] = 0.0;
  imu_sensor_state_[4] = 0.0;
  imu_sensor_state_[5] = 0.0;
  imu_sensor_state_[6] = 0.0;
  imu_sensor_state_[7] = 0.0;
  imu_sensor_state_[8] = 0.0;
  imu_sensor_state_[9] = 0.0;

  return hardware_interface::return_type::OK;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    imu_hardware::IMUHardware, hardware_interface::SensorInterface)
