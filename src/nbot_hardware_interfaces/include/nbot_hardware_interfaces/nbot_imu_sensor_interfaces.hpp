#ifndef NBOT_HARDWARE_INTERFACES__NBOT_IMU_SENSOR_INTERFACES_HPP_
#define NBOT_HARDWARE_INTERFACES__NBOT_IMU_SENSOR_INTERFACES_HPP_

#include "visibility_control.h"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace nbot_hardware_interfaces
{
	using return_type = hardware_interface::return_type;
	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
	using StateInterface = hardware_interface::StateInterface;
	using CommandInterface = hardware_interface::CommandInterface;

	using Imu = sensor_msgs::msg::Imu;
	using String = std_msgs::msg::String;

	class NbotImuSensor 
	: public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
	{
	public:
		RCLCPP_SHARED_PTR_DEFINITIONS(NbotImuSensor)

		NBOT_HARDWARE_INTERFACES_PUBLIC
		hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;

		NBOT_HARDWARE_INTERFACES_PUBLIC
		std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

		NBOT_HARDWARE_INTERFACES_PUBLIC
		hardware_interface::return_type start() override;

		NBOT_HARDWARE_INTERFACES_PUBLIC
		hardware_interface::return_type stop() override;

		NBOT_HARDWARE_INTERFACES_PUBLIC
		hardware_interface::return_type read() override;


	protected:
		void cleanup_node();

		realtime_tools::RealtimeBox<std::shared_ptr<Imu>> received_imu_msg_ptr_{nullptr};
		// realtime_tools::RealtimeBox<std::shared_ptr<String>> received_imu_msg_ptr_{nullptr};

		rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_ = nullptr;
		// rclcpp::Subscription<String>::SharedPtr imu_subscriber_ = nullptr;

		std::vector<double> imu_sensor_state_;

		bool subscriber_is_active_ = false;

		std::shared_ptr<rclcpp::Node> node_;

		void imu_cb(const std::shared_ptr<Imu> msg);
		// void imu_cb(const std::shared_ptr<String> msg);
		rclcpp::executors::MultiThreadedExecutor executor_;
		std::unique_ptr<std::thread> executor_thread_;

		uint connection_check_period_ms_;
		uint connection_timeout_ms_;
	};

} // namespace NBOT_HARDWARE_INTERFACES

#endif // NBOT_HARDWARE_INTERFACES__ROSBOT_IMU_SENSOR_HPP_