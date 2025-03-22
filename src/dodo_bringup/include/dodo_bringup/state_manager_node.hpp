#ifndef STATE_MANAGER_NODE_HPP
#define STATE_MANAGER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dodo_msgs/msg/aligned_sensor_data.hpp>

#include "dodo_bringup/robot_state_machine.hpp"

namespace dodo_bringup
{

class StateManagerNode : public rclcpp::Node
{
public:
  StateManagerNode();
  virtual ~StateManagerNode();

private:
  // Robot state machine
  RobotStateMachine state_machine_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr usb_commands_sub_;
  rclcpp::Subscription<dodo_msgs::msg::AlignedSensorData>::SharedPtr aligned_sensor_data_sub_;
  
  // Timer for publishing state
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Callback for timer
  void publishState();
  
  // Subscriber callbacks
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void usbCommandsCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg);
  
  // State machine transition callback
  void onStateChange(RobotState old_state, RobotState new_state);
  
  // Initialize the robot
  void initializeRobot();
};

} // namespace dodo_bringup

#endif // STATE_MANAGER_NODE_HPP