#include "rclcpp/rclcpp.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>  // Replace with the actual message type
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace px4_msgs::msg;

class FrequencyCalculator : public rclcpp::Node
{
public:
  FrequencyCalculator()
    : Node("frequency_calculator"), sample_count_(0)
  {
      // Initializing subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    // Create a subscriber to the input topic
    subscription_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos, std::bind(&FrequencyCalculator::calculateFrequency, this, std::placeholders::_1));

    // Initialize the last received time
    last_received_time_ = std::chrono::high_resolution_clock::now();
  }

  std::chrono::high_resolution_clock::time_point start_time_; // Make start_time_ public

private:
  void calculateFrequency(const VehicleLocalPosition::SharedPtr msg)
  {
    // Check if we've received the desired number of samples
    if (sample_count_ < 100)
    {
      // Update the sample count
      sample_count_++;

      // Calculate the time difference between the current and last received messages
      auto current_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_received_time_);

      // Print the time duration for each sample
      RCLCPP_INFO(get_logger(), "Sample %d: Time = %.2f us", sample_count_, static_cast<double>(duration.count()));

      // Update the last received time
      last_received_time_ = current_time;
    }
    else
    {
      // Calculate the average frequency based on the time duration of the samples
      auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        last_received_time_ - start_time_);

      double average_frequency = (sample_count_ - 1) / (static_cast<double>(total_duration.count()) / 1e6);

      // Print the average frequency
      RCLCPP_INFO(get_logger(), "Average Frequency: %.2f Hz", average_frequency);

      // Shutdown the node after calculating the frequency
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_;
  std::chrono::high_resolution_clock::time_point last_received_time_;
  int sample_count_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<FrequencyCalculator>();

  // Set the start time
  node->start_time_ = std::chrono::high_resolution_clock::now();

  // Spin the node to execute the callback function
  rclcpp::spin(node);

  return 0;
}
