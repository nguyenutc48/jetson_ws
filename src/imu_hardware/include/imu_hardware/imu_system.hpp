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

#ifndef IMU_HARDWARE_HPP_
#define IMU_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include "sensor-imu/imu.h"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/system_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
// #include "semantic_components/imu_sensor.hpp"
// #include "sensor_msgs/msg/imu.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "visibility_control.h"

    namespace imu_hardware{
        class IMUHardware: public hardware_interface::BaseInterface <hardware_interface::SensorInterface>
        {

            public :RCLCPP_SHARED_PTR_DEFINITIONS(IMUHardware);

    IMU_HARDWARE_PUBLIC
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;

    IMU_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    // IMU_HARDWARE_PUBLIC
    // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    IMU_HARDWARE_PUBLIC
    hardware_interface::return_type start() override;

    IMU_HARDWARE_PUBLIC
    hardware_interface::return_type stop() override;

    IMU_HARDWARE_PUBLIC
    hardware_interface::return_type read() override;

    // IMU_HARDWARE_PUBLIC
    // hardware_interface::return_type write() override;

  private:
    std::vector<double> imu_sensor_state_;
    imut *imu;
    error *err;
    
};

}  // namespace IMU_HARDWARE

#endif  // IMU_HARDWARE__DIFFBOT_SYSTEM_HPP_
