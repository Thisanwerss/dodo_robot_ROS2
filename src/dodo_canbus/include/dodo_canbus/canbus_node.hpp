#ifndef DODO_CANBUS_NODE_HPP
#define DODO_CANBUS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <algorithm>
#include <cmath>
#include "dodo_canbus/odrive_can.hpp"

namespace dodo_canbus
{

class CANBusNode : public rclcpp::Node
{
public:
  CANBusNode();
  virtual ~CANBusNode();

private:
  // Timer callback
  void updateCAN();
  
  // Subscribers callbacks
  void processedCommandsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  
  // Processing functions
  void sendCommands();
  void readMotorStates();
  void applyPIDGains();
  
  // Dummy mode helper function
  void generateDummyMotorStates(sensor_msgs::msg::JointState & msg);

  // Parameters
  std::string can_interface_;
  int update_rate_;
  std::vector<int> motor_ids_;
  std::string pid_gains_json_;
  bool dummy_mode_;
  
  // CAN interface
  std::unique_ptr<OdriveCANInterface> can_interface_ptr_;
  
  // Data storage
  sensor_msgs::msg::JointState::SharedPtr latest_processed_commands_;
  bool emergency_stop_active_;
  
  // Dummy mode state variables
  sensor_msgs::msg::JointState dummy_last_commands_;
  std::vector<double> dummy_last_positions_;
  std::vector<double> dummy_velocities_;
  
  // Mutexes for thread safety
  std::mutex commands_mutex_;
  std::mutex emergency_mutex_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr processed_commands_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_states_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Joint to motor ID mapping
  std::map<std::string, int> joint_to_motor_id_;
};

}  // namespace dodo_canbus

#endif  // DODO_CANBUS_NODE_HPP