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
 * @brief lqr control example
 * @file lqr_sim.cpp
 * @addtogroup examples
 * @author Prakrit Tyagi <prakritt@andrew.cmu.edu> <tyagiprakrit@gmail.com>
 */

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include "Eigen/Eigen"

#include <px4_ros_com/quaternion_math.hpp>
#include <px4_ros_com/quadcopter_model.hpp>
#include <px4_ros_com/ricatti.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class LqrSim : public rclcpp::Node
{
public:
    LqrSim();
	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_localposition_subscriber_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
	rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr vehicle_angularvelocity_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();   //!< publish offboard control mode
	void publish_actuator_motors(double *state_error);   //!< publish actuator motors
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);   //!< publish vehicle commands
    void timer_callback();   //!< timer callback for offboard control
	void vehicle_localposition_callback(const VehicleLocalPosition::UniquePtr msg);  //!< callback for vehicle local position topic
	void vehicle_attitude_callback(const VehicleAttitude::UniquePtr msg);   //!< callback for vehicle attitude topic
	void vehicle_angularvelocity_callback(const VehicleAngularVelocity::UniquePtr msg);   //!< callback for vehicle angular velocity topic
	void get_state_error(double *state_error);   //!< get state error

	// Reference vector
	double reference_state[NumStates];
	double reference_input[NumInputs];
	// State variables
	double current_state[NumStates];
	// Kalman gain
	double K[NumErrorInputs * NumErrorStates];
	// params
	double Q[NumErrorStates];
	double S[NumErrorStates];
	double R[NumErrorInputs];

	// data store
	std::ofstream control_output;
	std::ofstream control_norm_output;
	std::ofstream state_error_output;
};

/**
 * @brief Constructor
 */
LqrSim::LqrSim() : Node("lqr_sim"), offboard_setpoint_counter_(0), reference_state{0.0, 0.0, -5.0,
							0.7167351841926575, -0.008279332891106606, 0.0051797619089484215, 0.6972771286964417,
							0.0, 0.0, 0.0,
							0.0, 0.0, 0.0}
{
	// start time chrono
	// auto start = high_resolution_clock::now();
	// Initializing publishers
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
	actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);

	// Initializing subscribers
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	vehicle_localposition_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
	std::bind(&LqrSim::vehicle_localposition_callback, this, _1));

	vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
	std::bind(&LqrSim::vehicle_attitude_callback, this, _1));

	vehicle_angularvelocity_subscriber_ = this->create_subscription<VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", qos,
	std::bind(&LqrSim::vehicle_angularvelocity_callback, this, _1));

	// get dynamics model
	QuadcopterModel quadcopter_model;
	reference_input[0] = mass * gravity / (4 * kt);
	reference_input[1] = mass * gravity / (4 * kt);
	reference_input[2] = mass * gravity / (4 * kt);
	reference_input[3] = mass * gravity / (4 * kt);

	double jac[NumStates * (NumStates + NumInputs)];
	quadcopter_model.Jacobian(jac, reference_state, reference_input);

	double jac_discrete[NumStates * (NumStates + NumInputs)];
	quadcopter_model.Jacobian_RK4(jac_discrete, jac, 0.01); // 0.01 is the time step, 10ms, 100Hz

	double jac_reduced[NumErrorStates * (NumErrorStates + NumErrorInputs)];
	quadcopter_model.get_reduced_jacobian(jac_reduced, jac_discrete, reference_state);

	// read weights from file
	std::ifstream file("/home/prakrit/ws_px4_mpc/src/px4_ros_com/src/examples/lqr/weights_sim.txt");
    if (!file.is_open()) {
        std::cerr << "Error opening weights file!" << std::endl;
        exit(1);
    }
	// Read each line of the file
    std::string line;
	 while (std::getline(file, line)) {
        std::istringstream iss(line);
        char type;
        iss >> type;
		// Read the values based on the type
        if (type == 'Q' || type == 'S') {
            int value;
            char comma;
            for (int i = 0; i < 12; ++i) {
                iss >> value >> comma;
                if (type == 'Q') {
                    Q[i] = value;
                } else {
                    S[i] = value;
                }
            }
        } else if (type == 'R') {
            int value;
            char comma;
            for (int i = 0; i < 4; ++i) {
                iss >> value >> comma;
                R[i] = value;
            }
        }
    }
	file.close();
	// get the kalman gain
	ricatti::get_K(this->K, jac_reduced, Q, S, R);
	// 
	// auto stop = high_resolution_clock::now();
	// auto duration = duration_cast<milliseconds>(stop - start);
	// std::cout << "Time taken by function: "
	// 	 << duration.count() << " milliseconds" << std::endl;

	// open file to store control output
	control_output.open("/home/prakrit/ws_px4_mpc/src/px4_ros_com/src/examples/lqr/data_sim/control_output_sim.txt");
	if(!control_output.is_open())
	{
		std::cerr << "Error opening control output file!" << std::endl;
		exit(1);
	}
	control_norm_output.open("/home/prakrit/ws_px4_mpc/src/px4_ros_com/src/examples/lqr/data_sim/control_norm_output_sim.txt");
	if(!control_norm_output.is_open())
	{
		std::cerr << "Error opening control norm output file!" << std::endl;
		exit(1);
	}
	state_error_output.open("/home/prakrit/ws_px4_mpc/src/px4_ros_com/src/examples/lqr/data_sim/state_error_output_sim.txt");
	if(!state_error_output.is_open())
	{
		std::cerr << "Error opening state error output file!" << std::endl;
		exit(1);
	}
    timer_ = this->create_wall_timer(10ms, std::bind(&LqrSim::timer_callback, this)); // 0.01 is the time step, 10ms, 100Hz
}

/**
 * @brief Timer based callback for offboard control
 */
void LqrSim::timer_callback()
{
	// start timer 
	// auto start = high_resolution_clock::now();
    if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        this->arm();
    }

    // offboard_control_mode needs to be paired with actuator_motors
    publish_offboard_control_mode();
	double state_error[NumErrorStates];
	get_state_error(state_error);
    publish_actuator_motors(state_error);

	// write state error to file use loop since  its  an array
	for (int i = 0; i < NumErrorStates; i++)
	{
		state_error_output << state_error[i] << " ";
	}
	state_error_output << "\n";
    // stop the counter after reaching 11
    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
	// stop timer
	// auto stop = high_resolution_clock::now();
	// auto duration = duration_cast<milliseconds>(stop - start);
	// std::cout << "Time taken by function: "
	// 	 << duration.count() << " milliseconds" << std::endl;
}

/**
 * @brief Callback for vehicle local position
 * @param msg   VehicleLocalPosition message
 */
void LqrSim::vehicle_localposition_callback(const VehicleLocalPosition::UniquePtr msg)
{
	this->current_state[0] = msg->x;
	this->current_state[1] = msg->y;
	this->current_state[2] = msg->z;
	this->current_state[7] = msg->vx;
	this->current_state[8] = msg->vy;
	this->current_state[9] = msg->vz;
}

/**
 * @brief Callback for vehicle attitude
 * @param msg   VehicleAttitude message
 */
void LqrSim::vehicle_attitude_callback(const VehicleAttitude::UniquePtr msg)
{
	this->current_state[3] = msg->q[0];
	this->current_state[4] = msg->q[1];
	this->current_state[5] = msg->q[2];
	this->current_state[6] = msg->q[3];
}

/**
 * @brief Callback for vehicle angular velocity
 * @param msg   VehicleAngularVelocity message
 */
void LqrSim::vehicle_angularvelocity_callback(const VehicleAngularVelocity::UniquePtr msg)
{
	this->current_state[10] = msg->xyz[0];
	this->current_state[11] = msg->xyz[1];
	this->current_state[12] = msg->xyz[2];
}

/**
 * @brief Get state error
 */
void LqrSim::get_state_error(double *state_error)
{
	// map array to eigen vector
	Eigen::Map<Eigen::VectorXd> state_error_vec(state_error, NumErrorStates);
	Eigen::Map<Eigen::VectorXd> current_state_vec(this->current_state, NumStates);
	Eigen::Map<Eigen::VectorXd> reference_state_vec(this->reference_state, NumStates);

	const Eigen::Vector3d r = current_state_vec.block<3, 1>(0, 0);
    const Eigen::Vector4d q = current_state_vec.block<4, 1>(3, 0);
    const Eigen::Vector3d v = current_state_vec.block<3, 1>(7, 0);
    const Eigen::Vector3d w = current_state_vec.block<3, 1>(10, 0);

	const Eigen::Vector3d r_ref = reference_state_vec.block<3, 1>(0, 0);
	const Eigen::Vector4d q_ref = reference_state_vec.block<4, 1>(3, 0);
	const Eigen::Vector3d v_ref = reference_state_vec.block<3, 1>(7, 0);
	const Eigen::Vector3d w_ref = reference_state_vec.block<3, 1>(10, 0);

	// position error
	state_error_vec.block<3, 1>(0, 0) = r - r_ref;

	// attitude error
	state_error_vec.block<3, 1>(3, 0) = quaternion_math::get_qtorp(quaternion_math::get_L(q_ref).transpose() * q);

	// velocity error
	Eigen::Matrix3d Q = quaternion_math::qtoQ(q);
	state_error_vec.block<3, 1>(6, 0) = Q.transpose() * v - v_ref;

	// angular velocity error
	state_error_vec.block<3, 1>(9, 0) = w - w_ref;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void LqrSim::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void LqrSim::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only the flag for actuators is set to true.
 */
void LqrSim::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a thrust command to all 4 motors
 */
void LqrSim::publish_actuator_motors(double *state_error)
{
	// map array to eigen vector
	Eigen::Map<Eigen::VectorXd> state_error_vec(state_error, NumErrorStates);
	Eigen::Map<Eigen::VectorXd> reference_input_vec(this->reference_input, NumInputs);
	Eigen::Map<Eigen::MatrixXd> K_mat(this->K, NumErrorInputs, NumErrorStates);
	Eigen::Vector4d output = reference_input_vec - K_mat * state_error_vec;

	// write control output to file
	control_output << output << "\n";

	Eigen::Vector4d norm_output = output / max_thrust_sim;
	
	for (int i = 0; i <= 3; i++)
	{
		if (norm_output(i) >= 1.0 || norm_output(i) <= 0)
		{
			if (norm_output(i) >= 1.0)
			{
				norm_output(i) = 1.0;
			}
			else
			{
				norm_output(i) = 0.0;
			}
		}
	}

	// write control norm output to file
	control_norm_output << norm_output << "\n";
	ActuatorMotors msg{};
    msg.control[0] = norm_output(0);
	msg.control[1] = norm_output(1);
	msg.control[2] = norm_output(2);
	msg.control[3] = norm_output(3);
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_motors_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void LqrSim::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << R"(
	__/\\\\\\\\\____________/\\\___________/\\\\\\\\\_____ 
	 _\///\\\\\/__________/\\\\/\\\\______/\\\///////\\\___ 
	  ___\/\\\___________/\\\//\////\\\___\/\\\_____\/\\\___  
	   ___\/\\\__________/\\\______\//\\\__\/\\\\\\\\\\\/____ 
	    ___\/\\\_________\//\\\______/\\\___\/\\\//////\\\____ 
	     ___\/\\\__________\///\\\\/\\\\/____\/\\\____\//\\\___ 
	      ___\/\\\____________\////\\\//______\/\\\_____\//\\\__  
	       __/\\\\\\\\\\\\________\///\\\\\\___\/\\\______\//\\\_
	        _\////////////___________\//////____\///________\///__ 

	)" << '\n';
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LqrSim>());

	rclcpp::shutdown();
	return 0;
}
