/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example (multi-UAV namespace support)
 * @file offboard_control_multi.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Adapted for multi-UAV by GitHub Copilot
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
       OffboardControl() : Node("offboard_control_multi")
       {
	       std::string ns = this->get_namespace(); // ej. "/" o "/px4_2" o "/some/px4_2"
	       RCLCPP_INFO(this->get_logger(), "[NS LOG] get_namespace() returned: '%s'", ns.c_str());
	       if (ns == "/") ns = "";
	       // Normalizar: quitar cualquier '/' inicial y quedarnos con el segmento final
	       if (!ns.empty()) {
		       // quitar slash inicial/final
		       while (!ns.empty() && ns.front() == '/') ns.erase(0,1);
		       while (!ns.empty() && ns.back() == '/') ns.pop_back();
		       // quedarse con el último segmento si hay subnamespaces
		       size_t pos = ns.rfind('/');
		       if (pos != std::string::npos) ns = ns.substr(pos + 1);
	       }
	       RCLCPP_INFO(this->get_logger(), "[NS LOG] Normalized ns for instance extraction: '%s'", ns.c_str());

	       // Extraer número de instancia considerando varios formatos
	       int px4_instance = 1; // fallback
	       if (!ns.empty()) {
		       const std::string prefix = "px4_";
		       size_t pref_pos = ns.find(prefix);
		       if (pref_pos != std::string::npos) {
			       std::string num_str = ns.substr(pref_pos + prefix.size());
			       try {
				       px4_instance = std::stoi(num_str);
				       RCLCPP_INFO(this->get_logger(), "[NS LOG] Extracted px4_instance from ns: %d", px4_instance);
			       } catch (...) {
				       px4_instance = 1;
				       RCLCPP_WARN(this->get_logger(), "[NS LOG] Failed to extract px4_instance from ns, defaulting to 1");
			       }
		       }
	       }
	       px4_instance_ = px4_instance;
	       RCLCPP_INFO(this->get_logger(), "[NS LOG] Final px4_instance_: %d", px4_instance_);

	       // Crear publishers con la namespace normalizada (añadir '/' delante si no está)
	       std::string pub_ns = ns.empty() ? "" : ("/" + ns);
	       RCLCPP_INFO(this->get_logger(), "[NS LOG] pub_ns used for publishers: '%s'", pub_ns.c_str());

	       offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(pub_ns + "/fmu/in/offboard_control_mode", 10);
	       trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(pub_ns + "/fmu/in/trajectory_setpoint", 10);
	       vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(pub_ns + "/fmu/in/vehicle_command", 10);

	       offboard_setpoint_counter_ = 0;

	       auto timer_callback = [this]() -> void {
		       if (offboard_setpoint_counter_ == 10) {
			       // Change to Offboard mode after 10 setpoints
			       this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			       // Arm the vehicle
			       this->arm();
		       }
		       publish_offboard_control_mode();
		       publish_trajectory_setpoint();
		       if (offboard_setpoint_counter_ < 11) {
			       offboard_setpoint_counter_++;
		       }
	       };
	       timer_ = this->create_wall_timer(100ms, timer_callback);
       }

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;
	uint64_t offboard_setpoint_counter_;
	int px4_instance_ = 1;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	// PX4 expects target_system = px4_instance + 1
	msg.target_system = px4_instance_ + 1;
	msg.target_component = 1;
	msg.source_system = px4_instance_ + 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control multi node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
