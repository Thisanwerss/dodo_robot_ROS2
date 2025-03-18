#ifndef DODO_SAFETY_NODE_HPP
#define DODO_SAFETY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <dodo_msgs/msg/aligned_sensor_data.hpp>
#include <string>
#include <vector>
#include <fstream>

namespace dodo_safety
{

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode();
  virtual ~SafetyNode();

private:
  // Timer callback
  void checkSafety();
  
  // Subscriber callback
  void alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg);
  
  // Safety check functions
  bool checkIMUSafety(const dodo_msgs::msg::AlignedSensorData::SharedPtr & msg);
  bool checkJointSafety(const dodo_msgs::msg::AlignedSensorData::SharedPtr & msg);
  
  // Event logging
  void logSafetyEvent(const std::string & event, bool is_error);

  // Parameters
  double max_acceleration_;
  double max_tilt_;
  double max_joint_velocity_;
  int check_rate_;
  
  // Data storage
  dodo_msgs::msg::AlignedSensorData::SharedPtr latest_sensor_data_;
  bool emergency_active_;
  
  // File for logging
  std::ofstream log_file_;
  
  // Subscriber
  rclcpp::Subscription<dodo_msgs::msg::AlignedSensorData>::SharedPtr aligned_sensor_data_sub_;
  
  // Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dodo_safety

#endif  // DODO_SAFETY_NODE_HPP