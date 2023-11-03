#include "nbot_hardware_interfaces/nbot_imu_sensor_interfaces.hpp"

#include <string>
#include <vector>

// #include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nbot_hardware_interfaces
{
	hardware_interface::return_type NbotImuSensor::configure(
		const hardware_interface::HardwareInfo &info)
	{
		RCLCPP_INFO(rclcpp::get_logger("NbotImuSensor"), "Initializing");

		if (hardware_interface::SensorInterface::configure(info) != hardware_interface::return_type::OK)
		{
			return hardware_interface::return_type::ERROR;
		}

		imu_sensor_state_.resize(info_.sensors[0].state_interfaces.size(), 0.0);

		connection_timeout_ms_ = std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
		connection_check_period_ms_ = std::stoul(info_.hardware_parameters["connection_check_period_ms"]);

		node_ = std::make_shared<rclcpp::Node>("imu_sensor_node");
		executor_.add_node(node_);
		executor_thread_ =
			std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

		return hardware_interface::return_type::OK;
	}

	std::vector<hardware_interface::StateInterface> NbotImuSensor::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> state_interfaces;

		for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++)
		{
			state_interfaces.emplace_back(
				hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_sensor_state_[i]));
		}

		return state_interfaces;
	}

	void NbotImuSensor::cleanup_node()
	{
		imu_subscriber_.reset();
	}

	void NbotImuSensor::imu_cb(const std::shared_ptr<Imu> msg)
	{
		RCLCPP_DEBUG(node_->get_logger(), "Received imu message");
		received_imu_msg_ptr_.set(std::move(msg));
	}

	hardware_interface::return_type NbotImuSensor::start()
	{
		RCLCPP_INFO(rclcpp::get_logger("NbotImuSensor"), "Starting ...please wait...");

		for (auto i = 0u; i < imu_sensor_state_.size(); i++)
		{
			imu_sensor_state_[i] = 0.0;
		}

		imu_subscriber_ = node_->create_subscription<Imu>("~/imu", rclcpp::SensorDataQoS(),
														  std::bind(&NbotImuSensor::imu_cb, this, std::placeholders::_1));

		std::shared_ptr<Imu> imu_msg;
		for (uint wait_time = 0; wait_time <= connection_timeout_ms_; wait_time += connection_check_period_ms_)
		{
			RCLCPP_WARN(rclcpp::get_logger("NbotImuSensor"), "Feedback message from imu wasn't received yet");
			received_imu_msg_ptr_.get(imu_msg);
			if (imu_msg)
			{
				RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
				return hardware_interface::return_type::OK;
			}

			rclcpp::sleep_for(std::chrono::milliseconds(connection_check_period_ms_));
		}

		RCLCPP_FATAL(node_->get_logger(), "Activation failed, timeout reached while waiting for feedback from imu");

		RCLCPP_INFO(rclcpp::get_logger("NbotImuSensor"), "System Successfully started!");

		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type NbotImuSensor::stop()
	{
		RCLCPP_INFO(rclcpp::get_logger("NbotImuSensor"), "Stopping ...please wait...");

		cleanup_node();

		RCLCPP_INFO(rclcpp::get_logger("NbotImuSensor"), "System successfully stopped!");

		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type NbotImuSensor::read()
	{
		std::shared_ptr<Imu> imu_msg;
		received_imu_msg_ptr_.get(imu_msg);

		RCLCPP_INFO(rclcpp::get_logger("NbotImuSensor"), "Reading...");

		if (!imu_msg)
		{
			RCLCPP_ERROR(rclcpp::get_logger("RosbotImuSensor"), "Imu message wasn't received");
			return return_type::ERROR;
		}

		imu_sensor_state_[0] = imu_msg->orientation.x;
		imu_sensor_state_[1] = imu_msg->orientation.y;
		imu_sensor_state_[2] = imu_msg->orientation.z;
		imu_sensor_state_[3] = imu_msg->orientation.w;
		imu_sensor_state_[4] = imu_msg->angular_velocity.x;
		imu_sensor_state_[5] = imu_msg->angular_velocity.y;
		imu_sensor_state_[6] = imu_msg->angular_velocity.z;
		imu_sensor_state_[7] = imu_msg->linear_acceleration.x;
		imu_sensor_state_[8] = imu_msg->linear_acceleration.y;
		imu_sensor_state_[9] = imu_msg->linear_acceleration.z;

		return hardware_interface::return_type::OK;
	}


} // namespace nbot_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
	nbot_hardware_interfaces::NbotImuSensor, hardware_interface::SensorInterface)