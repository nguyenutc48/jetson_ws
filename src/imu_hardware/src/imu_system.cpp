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

  err = imu_init(&imu, "/dev/i2c-1", IMU_ACC_RANGE_2G, IMU_GYRO_RANGE_250DPS);
  if (err != NULL)
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

    for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_sensor_state_[i]));
    }

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

  imu_destroy(imu);

      status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IMUHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Reading...");

  imu_output io;
  err = imu_read(imu, &io);
  if (err != NULL)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Dữ liệu từ cảm biến gyro và acc
  double gyro_x = io.gyro.x; // Thay thế bằng giá trị thực tế từ cảm biến gyro
  double gyro_y = io.gyro.y; // Thay thế bằng giá trị thực tế từ cảm biến gyro
  double gyro_z = io.gyro.z; // Thay thế bằng giá trị thực tế từ cảm biến gyro
  double acc_x = io.acc.x; // Thay thế bằng giá trị gia tốc theo trục x (m/s^2)
  double acc_y = io.acc.y;  // Thay thế bằng giá trị gia tốc theo trục y (m/s^2)
  double acc_z = io.acc.z;  // Thay thế bằng giá trị gia tốc theo trục z (m/s^2)

  // Tính góc pitch và roll từ cảm biến gyro
  double roll = std::atan2(acc_y, acc_z);
  double pitch = std::atan2(-acc_x, std::sqrt(acc_y * acc_y + acc_z * acc_z));

  // Tính góc yaw từ cảm biến gyro
  double yaw = gyro_z;

  // Chuyển đổi các góc từ độ sang radian
  double roll_rad = roll * M_PI / 180.0;
  double pitch_rad = pitch * M_PI / 180.0;
  double yaw_rad = yaw * M_PI / 180.0;

  // Tính các phép biến đổi
  double cy = cos(yaw_rad * 0.5);
  double sy = sin(yaw_rad * 0.5);
  double cp = cos(pitch_rad * 0.5);
  double sp = sin(pitch_rad * 0.5);
  double cr = cos(roll_rad * 0.5);
  double sr = sin(roll_rad * 0.5);

  imu_sensor_state_[0] = cy * cp * sr - sy * sp * cr;
  imu_sensor_state_[1] = sy * cp * sr + cy * sp * cr;
  imu_sensor_state_[2] = sy * cp * cr - cy * sp * sr;
  imu_sensor_state_[3] = cy * cp * cr + sy * sp * sr;
  imu_sensor_state_[4] = gyro_x;
  imu_sensor_state_[5] = gyro_y;
  imu_sensor_state_[6] = gyro_z;
  imu_sensor_state_[7] = acc_x;
  imu_sensor_state_[8] = acc_y;
  imu_sensor_state_[9] = acc_z;

  return hardware_interface::return_type::OK;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    imu_hardware::IMUHardware, hardware_interface::SensorInterface)
