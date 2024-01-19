/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
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
 * @brief optitrack listener control example
 * @file optitrack_listener.cpp
 * @addtogroup examples
 * @author Prakrit Tyagi <prakritt@andrew.cmu.edu> <tyagiprakrit@gmail.com>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <mocap_msgs/msg/rigid_bodies.hpp>

#include <iostream>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief Optitrack rigid body topic listener and publish to vehicle_visual_odometry topic
 */
class OptitrackListener : public rclcpp::Node
{
public:
	explicit OptitrackListener() : Node("optitrack_listener")
	{
        visual_odometry_publisher_ = this->create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        mocap_subscriber_ = this->create_subscription<mocap_msgs::msg::RigidBodies>("/rigid_bodies", qos,
        [this](const mocap_msgs::msg::RigidBodies::UniquePtr msg) {

            this->x_ = msg->rigidbodies[0].pose.position.x;
            this->y_ = msg->rigidbodies[0].pose.position.y;
            this->z_ = msg->rigidbodies[0].pose.position.z;
            this->q_x_ = msg->rigidbodies[0].pose.orientation.x;    
            this->q_y_ = msg->rigidbodies[0].pose.orientation.y;
            this->q_z_ = msg->rigidbodies[0].pose.orientation.z;
            this->q_w_ = msg->rigidbodies[0].pose.orientation.w;

            this->publish_mocap();

            // print out the position and orientation of the rigid body
            std::cout << "x: " << this->x_ << std::endl;
            std::cout << "y: " << this->y_ << std::endl;
            std::cout << "z: " << this->z_ << std::endl;
            std::cout << "q_w: " << this->q_w_ << std::endl;
            std::cout << "q_x: " << this->q_x_ << std::endl;
            std::cout << "q_y: " << this->q_y_ << std::endl;
            std::cout << "q_z: " << this->q_z_ << std::endl;
            
        
        }); 

        // auto timer_callback = [this]() -> void {
            
        //     this->publish_mocap();

        //     // print out the position and orientation of the rigid body
        //     std::cout << "x: " << this->x_ << std::endl;
        //     std::cout << "y: " << this->y_ << std::endl;
        //     std::cout << "z: " << this->z_ << std::endl;
        //     std::cout << "q_w: " << this->q_w_ << std::endl;
        //     std::cout << "q_x: " << this->q_x_ << std::endl;
        //     std::cout << "q_y: " << this->q_y_ << std::endl;
        //     std::cout << "q_z: " << this->q_z_ << std::endl;
        
        // };

        // timer_ = this->create_wall_timer(10ms, timer_callback);
		
	}

private:

    rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr mocap_subscriber_;
    rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_odometry_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    void publish_mocap();

    float x_;
    float y_;
    float z_;
    float q_x_;
    float q_y_;
    float q_z_;
    float q_w_;

};

void OptitrackListener::publish_mocap()
{
    VehicleOdometry visual_odometry_msg{};

    visual_odometry_msg.pose_frame = 2;
    visual_odometry_msg.position[0] = this->x_;
    visual_odometry_msg.position[1] = -this->y_;
    visual_odometry_msg.position[2] = -this->z_;
    visual_odometry_msg.q[0] = this->q_w_;
    visual_odometry_msg.q[1] = this->q_x_;
    visual_odometry_msg.q[2] = -this->q_y_;
    visual_odometry_msg.q[3] = -this->q_z_;
    visual_odometry_msg.velocity_frame = 1;
    visual_odometry_msg.velocity[0] = NAN;
    visual_odometry_msg.velocity[1] = NAN;
    visual_odometry_msg.velocity[2] = NAN;
    visual_odometry_msg.angular_velocity[0] = NAN;
    visual_odometry_msg.angular_velocity[1] = NAN;
    visual_odometry_msg.angular_velocity[2] = NAN;
    visual_odometry_msg.position_variance[0] = 0.0f;
    visual_odometry_msg.position_variance[1] = 0.0f;
    visual_odometry_msg.position_variance[2] = 0.0f;
    visual_odometry_msg.orientation_variance[0] = 0.0f;
    visual_odometry_msg.orientation_variance[1] = 0.0f;
    visual_odometry_msg.orientation_variance[2] = 0.0f;
    visual_odometry_msg.velocity_variance[0] = 0.0f;
    visual_odometry_msg.velocity_variance[1] = 0.0f;
    visual_odometry_msg.velocity_variance[2] = 0.0f;
    visual_odometry_msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    visual_odometry_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    visual_odometry_publisher_->publish(visual_odometry_msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting Optitrack Listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OptitrackListener>());

	rclcpp::shutdown();
	return 0;
}
