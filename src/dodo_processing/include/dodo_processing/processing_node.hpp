#ifndef DODO_PROCESSING_NODE_HPP
#define DODO_PROCESSING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <dodo_msgs/msg/aligned_sensor_data.hpp>
#include <string>
#include <vector>
#include <mutex>

namespace dodo_processing
{

class ProcessingNode : public rclcpp::Node
{
public:
  ProcessingNode();
  virtual ~ProcessingNode();

private:
  // Timer callback
  void processCommands();
  
  // Subscribers callbacks
  void rlActionsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void usbCommandsCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg);
  
  // Processing functions
  sensor_msgs::msg::JointState combineCommands();
  void applyMotionConstraints(sensor_msgs::msg::JointState & cmd);

  // Parameters
  int command_rate_;
  std::string motion_constraints_json_;
  
  // Data storage
  sensor_msgs::msg::JointState::SharedPtr latest_rl_actions_;
  std_msgs::msg::Int32::SharedPtr latest_usb_command_;
  dodo_msgs::msg::AlignedSensorData::SharedPtr latest_sensor_data_;
  
  // Mutexes for thread safety
  std::mutex rl_actions_mutex_;
  std::mutex usb_command_mutex_;
  std::mutex sensor_data_mutex_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rl_actions_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr usb_commands_sub_;
  rclcpp::Subscription<dodo_msgs::msg::AlignedSensorData>::SharedPtr aligned_sensor_data_sub_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr processed_commands_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dodo_processing

#endif  // DODO_PROCESSING_NODE_HPP