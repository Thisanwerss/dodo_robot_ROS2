#include "dodo_bringup/state_manager_node.hpp"

namespace dodo_bringup
{

StateManagerNode::StateManagerNode() 
: Node("state_manager_node") 
{
  // Declare parameters
  //this->declare_parameter("use_sim_time", false);
  this->declare_parameter("dummy_mode", false);
  
  // Get parameters
  bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
  bool dummy_mode = this->get_parameter("dummy_mode").as_bool();
  
  RCLCPP_INFO(this->get_logger(), "Parameter values: use_sim_time=%s, dummy_mode=%s",
              use_sim_time ? "true" : "false", dummy_mode ? "true" : "false");
  
  // Create publishers
  robot_state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_state", 10);
  
  // Create subscribers
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", 10, std::bind(&StateManagerNode::emergencyStopCallback, this, std::placeholders::_1));
    
  usb_commands_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/usb_commands", 10, std::bind(&StateManagerNode::usbCommandsCallback, this, std::placeholders::_1));
    
  aligned_sensor_data_sub_ = this->create_subscription<dodo_msgs::msg::AlignedSensorData>(
    "/aligned_sensor_data", 10, std::bind(&StateManagerNode::alignedSensorDataCallback, this, std::placeholders::_1));
  
  // Register state change callback
  state_machine_.registerTransitionCallback(
    std::bind(&StateManagerNode::onStateChange, this, std::placeholders::_1, std::placeholders::_2));
  
  // Create timer for publishing state
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&StateManagerNode::publishState, this));
  
  // Initialize the robot
  initializeRobot();
  
  RCLCPP_INFO(this->get_logger(), "State Manager Node initialized");
}

StateManagerNode::~StateManagerNode() 
{
  RCLCPP_INFO(this->get_logger(), "State Manager Node shutting down");
}

void StateManagerNode::publishState() 
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = state_machine_.getCurrentStateString();
  robot_state_pub_->publish(std::move(msg));
}

void StateManagerNode::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) 
{
  if (msg->data) {
    // Emergency stop activated
    state_machine_.processEvent(StateEvent::EMERGENCY);
    RCLCPP_WARN(this->get_logger(), "Emergency stop activated");
  } else {
    // Emergency stop cleared
    state_machine_.processEvent(StateEvent::EMERGENCY_CLEARED);
    RCLCPP_INFO(this->get_logger(), "Emergency stop cleared");
  }
}

void StateManagerNode::usbCommandsCallback(const std_msgs::msg::Int32::SharedPtr msg) 
{
  // If we receive any USB command and we're in STANDBY, transition to MANUAL control
  if (state_machine_.getCurrentState() == RobotState::STANDBY) {
    state_machine_.processEvent(StateEvent::MANUAL_CONTROL);
  }
}

void StateManagerNode::alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg) 
{
  // Example: Check IMU data for potential error conditions
  const auto& imu = msg->imu_data;
  
  // Calculate total acceleration magnitude
  double accel_x = imu.linear_acceleration.x;
  double accel_y = imu.linear_acceleration.y;
  double accel_z = imu.linear_acceleration.z;
  double accel_magnitude = std::sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  
  // Example condition: If acceleration is extremely high, detect error
  const double MAX_SAFE_ACCEL = 20.0; // m/s²
  if (accel_magnitude > MAX_SAFE_ACCEL) {
    // This would normally be handled by the safety node, but as an example:
    if (state_machine_.getCurrentState() != RobotState::ERROR && 
        state_machine_.getCurrentState() != RobotState::EMERGENCY_STOP) {
      state_machine_.processEvent(StateEvent::ERROR_DETECTED);
      RCLCPP_ERROR(this->get_logger(), "Error detected: Excessive acceleration");
    }
  }
}

void StateManagerNode::onStateChange(RobotState old_state, RobotState new_state) 
{
  RCLCPP_INFO(this->get_logger(), "Robot state changed from %s to %s", 
             RobotStateMachine::stateToString(old_state).c_str(),
             RobotStateMachine::stateToString(new_state).c_str());
  
  // Perform actions based on new state
  switch (new_state) {
    case RobotState::INITIALIZING:
      RCLCPP_INFO(this->get_logger(), "Initializing robot systems...");
      break;
      
    case RobotState::STANDBY:
      RCLCPP_INFO(this->get_logger(), "Robot is in standby mode");
      break;
      
    case RobotState::MANUAL:
      RCLCPP_INFO(this->get_logger(), "Robot is now in manual control mode");
      break;
      
    case RobotState::AUTONOMOUS:
      RCLCPP_INFO(this->get_logger(), "Robot is now in autonomous control mode");
      break;
      
    case RobotState::ERROR:
      RCLCPP_ERROR(this->get_logger(), "Robot is in error state");
      break;
      
    case RobotState::EMERGENCY_STOP:
      RCLCPP_ERROR(this->get_logger(), "Robot is in emergency stop state");
      break;
      
    default:
      break;
  }
}

void StateManagerNode::initializeRobot() 
{
  // Simulate boot sequence
  RCLCPP_INFO(this->get_logger(), "Starting robot initialization...");
  
  // Transition to INITIALIZING state
  state_machine_.processEvent(StateEvent::BOOT_COMPLETE);
  
  // Simulate some initialization tasks
  // In a real system, this would wait for confirmation that subsystems are ready
  
  // For demonstration purposes, automatically transition to STANDBY after a delay
  init_timer_= this->create_wall_timer(
    std::chrono::seconds(2),
    [this]() {
      RCLCPP_INFO(this->get_logger(), "Robot initialization complete");
      state_machine_.processEvent(StateEvent::ACTIVATION);
      
      // Cancel this one-shot timer
      return true;
    });
}

} // namespace dodo_bringup

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_bringup::StateManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}