#include "dodo_canbus/canbus_node.hpp"
#include <nlohmann/json.hpp>
#include "dodo_canbus/odrive_can.hpp"
#include <fstream>

namespace dodo_canbus
{

CANBusNode::CANBusNode()
: Node("canbus_node"), emergency_stop_active_(false)
{
  // Declare parameters
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("update_rate", 100);
  this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8});
  this->declare_parameter("pid_gains", "{}"); 
  this->declare_parameter("dummy_mode", false);

  // Get parameters
  can_interface_ = this->get_parameter("can_interface").as_string();
  update_rate_ = this->get_parameter("update_rate").as_int();
  dummy_mode_ = this->get_parameter("dummy_mode").as_bool();
  
  // Get motor IDs and convert from int64_t to int 
  auto motor_ids_int64 = this->get_parameter("motor_ids").as_integer_array();
  motor_ids_.clear();
  for (const auto & id : motor_ids_int64) {
    motor_ids_.push_back(static_cast<int>(id));
  }
  
  pid_gains_json_ = this->get_parameter("pid_gains").as_string();

  // Create joint to motor ID mapping (assuming joint names are in order with motor IDs) 
  std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
  joint_to_motor_id_.clear();
  for (size_t i = 0; i < joint_names.size() && i < motor_ids_.size(); ++i) {
    joint_to_motor_id_[joint_names[i]] = motor_ids_[i];
  }

  // Initialize CAN interface if not in dummy mode
  if (!dummy_mode_) {
    can_interface_ptr_ = std::make_unique<OdriveCANInterface>(can_interface_);
    if (!can_interface_ptr_->init()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface: %s", can_interface_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Initialized CAN interface: %s", can_interface_.c_str());
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Running in dummy mode - using simulated CAN interface");
  }

  // Create subscribers
  processed_commands_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/processed_commands", 10, std::bind(&CANBusNode::processedCommandsCallback, this, std::placeholders::_1));
    
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", 10, std::bind(&CANBusNode::emergencyStopCallback, this, std::placeholders::_1)); 

  // Create publisher
  motor_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/motor_states", 10);


  // Create timer for updating CAN
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / update_rate_),
    std::bind(&CANBusNode::updateCAN, this));

  RCLCPP_INFO(this->get_logger(), "CANBUS Node initialized with update rate %d Hz", update_rate_);
}

CANBusNode::~CANBusNode()
{
  if (can_interface_ptr_) {
    can_interface_ptr_->close();
  }
  RCLCPP_INFO(this->get_logger(), "CANBUS Node shutting down");
}

void CANBusNode::processedCommandsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(commands_mutex_);
  latest_processed_commands_ = msg;
  //这里不马上添加读取电机状态的函数，而是等到updateCAN函数中调用（防止CAN阻塞）
}

void CANBusNode::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(emergency_mutex_);
  bool was_active = emergency_stop_active_;
  emergency_stop_active_ = msg->data;
  
  if (!was_active && emergency_stop_active_) {
    RCLCPP_WARN(this->get_logger(), "Emergency stop activated");
    if (can_interface_ptr_) {
      can_interface_ptr_->emergencyStop();
    }
  } else if (was_active && !emergency_stop_active_) {
    RCLCPP_INFO(this->get_logger(), "Emergency stop deactivated");
  }
}

void CANBusNode::updateCAN()
{
  // Check if emergency stop is active
  {
    std::lock_guard<std::mutex> lock(emergency_mutex_);
    if (emergency_stop_active_) {
      return;  // Don't send commands if emergency stop is active
    }
  }
  
  // Read motor states and write commands to motors
  //sendCommands();
  //readMotorStates();

  //below is the code for testing the CAN interface
  int can_id = 0x07 | 0x00C; // Simple CAN Protocol 0x00c for set the position
        bool result1 = can_interface_ptr_->sendPositionCommand(can_id, 1.0);
        if (!result1) {
          RCLCPP_WARN(this->get_logger(), "Failed to send position command to motor %d", can_id);

}
  MotorState state;
      result1=can_interface_ptr_->readMotorState(0x07, state);







}


void CANBusNode::sendCommands()
{  

  // Get the latest processed commands
  sensor_msgs::msg::JointState::SharedPtr commands;
  {
    std::lock_guard<std::mutex> lock(commands_mutex_);
    commands = latest_processed_commands_;
  }
  
  // Check if we have valid commands
  if (!commands || commands->name.empty() || commands->position.empty()) {
    return;
  }
  
  if (dummy_mode_) {
    // In dummy mode, just store the commands for future reference
    // but don't try to send them to real hardware
    dummy_last_commands_ = *commands;
    RCLCPP_DEBUG(this->get_logger(), "Simulating sending commands to motors");
    return;
  }

  // closeloop control
  for (int node_id = 1; node_id <= 8; ++node_id) {
  int can_id = node_id<<5 | 0x00C; 
   if (!can_interface_ptr_->sendAxisStateRequest(can_id, 0x03)) {  // AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            std::cerr << "Failed to start calibration for motor " << can_id << std::endl;
        }
  std::this_thread::sleep_for(std::chrono::seconds(5));
   if (!can_interface_ptr_->sendAxisStateRequest(can_id, 0x08)) {  // close loop control
            std::cerr << "Failed to start calibration for motor " << can_id << std::endl;
        } 
}






  // Send position commands to motors (normal mode)
  for (size_t i = 0; i < commands->name.size() && i < commands->position.size(); ++i) {
    const auto& joint_name = commands->name[i];
    
    // Check if we have a mapping for this joint
    auto it = joint_to_motor_id_.find(joint_name);
    if (it != joint_to_motor_id_.end()) {
      int motor_id = it->second;
      double position = commands->position[i];
      
      // Send command to motor
      if (can_interface_ptr_) {
        int can_id = motor_id | 0x00C; // Simple CAN Protocol 0x00c for set the position
        bool result = can_interface_ptr_->sendPositionCommand(can_id, position);
        if (!result) {
          RCLCPP_WARN(this->get_logger(), "Failed to send position command to motor %d", can_id);
        }
      }
    }
  }
}

void CANBusNode::readMotorStates()
{
  // Create JointState message
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = this->now();

  if (dummy_mode_) {
    // Generate dummy motor states in dummy mode
    generateDummyMotorStates(*joint_state_msg);
    RCLCPP_DEBUG(this->get_logger(), "Publishing dummy motor states");
  } else {
    // Read motor states from CAN in normal mode
    std::map<int, MotorState> motor_states;
    bool result = false;

    if (can_interface_ptr_) {
      result = can_interface_ptr_->readMotorStates(motor_ids_, motor_states);
    }

    if (!result) {
      RCLCPP_WARN(this->get_logger(), "Failed to read motor states");
      return;
    }

    // Populate JointState message
    for (const auto& joint_motor_pair : joint_to_motor_id_) {
      const auto& joint_name = joint_motor_pair.first;
      int motor_id = joint_motor_pair.second;

      auto it = motor_states.find(motor_id);
      if (it != motor_states.end()) {
        const auto& state = it->second;

        joint_state_msg->name.push_back(joint_name);
        joint_state_msg->position.push_back(state.position);
        joint_state_msg->velocity.push_back(state.velocity);
        joint_state_msg->effort.push_back(state.torque);
      }
    }
  }

  // Publish motor states
  motor_states_pub_->publish(*joint_state_msg);

  // === record ===
  nlohmann::json frame;
  frame["name"] = joint_state_msg->name;
  frame["position"] = joint_state_msg->position;
  frame["velocity"] = joint_state_msg->velocity;
  frame["effort"] = joint_state_msg->effort;

  {
    std::lock_guard<std::mutex> lock(record_mutex_);  // 如果多线程访问
    recorded_trajectory_.push_back(frame);

    // 每帧都保存到文件（如需减少磁盘写入频率，可单独触发保存）
    std::ofstream file("trajectory/record.json");
    file << recorded_trajectory_.dump(2);
  }
}


void CANBusNode::generateDummyMotorStates(sensor_msgs::msg::JointState & msg)
{
  static double time_counter = 0.0;
  time_counter += 1.0 / static_cast<double>(update_rate_);
  
  // Create a list of all joint names we know about
  for (const auto& joint_motor_pair : joint_to_motor_id_) {
    msg.name.push_back(joint_motor_pair.first);
  }
  
  // Generate dummy position values for each joint
  for (size_t i = 0; i < msg.name.size(); ++i) {
    // Use target positions from dummy_last_commands_ if available, otherwise generate sinusoidal movement
    double position;
    if (!dummy_last_commands_.name.empty() && !dummy_last_commands_.position.empty()) {
      // Find this joint in dummy_last_commands_
      auto it = std::find(dummy_last_commands_.name.begin(), dummy_last_commands_.name.end(), msg.name[i]);
      if (it != dummy_last_commands_.name.end()) {
        int index = std::distance(dummy_last_commands_.name.begin(), it);
        if (index < static_cast<int>(dummy_last_commands_.position.size())) {
          // Gradually move toward target position (simple PD control simulation)
          double target = dummy_last_commands_.position[index];
          double current = dummy_last_positions_[i];
          position = current + 0.1 * (target - current);  // Move 10% of the way to target
        } else {
          position = dummy_last_positions_[i];  // Use last position
        }
      } else {
        position = dummy_last_positions_[i];  // Use last position
      }
    } else {
      // No commands received yet, generate simple sinusoidal movement
      double freq = 0.2 + 0.05 * static_cast<double>(i);  // Different frequency for each joint
      position = 0.2 * sin(freq * time_counter);
    }
    
    // Store the position for next time
    while (dummy_last_positions_.size() <= i) {
      dummy_last_positions_.push_back(0.0);
    }
    dummy_last_positions_[i] = position;
    
    // Calculate velocity as the derivative of position (basic approximation)
    double velocity;
    if (i < dummy_velocities_.size()) {
      velocity = (position - dummy_last_positions_[i]) * update_rate_;
      // Apply low-pass filter to smooth velocity
      velocity = 0.8 * velocity + 0.2 * dummy_velocities_[i];
      dummy_velocities_[i] = velocity;
    } else {
      velocity = 0.0;
      dummy_velocities_.push_back(velocity);
    }
    
    // Calculate effort based on velocity and position (simple simulation)
    double effort = 0.1 * velocity + 0.2 * position;
    
    // Add to message
    msg.position.push_back(position);
    msg.velocity.push_back(velocity);
    msg.effort.push_back(effort);
  }
}

}  // namespace dodo_canbus

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_canbus::CANBusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}