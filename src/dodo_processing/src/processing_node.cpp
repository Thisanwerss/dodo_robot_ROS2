#include "dodo_processing/processing_node.hpp"
#include <nlohmann/json.hpp>

namespace dodo_processing
{

ProcessingNode::ProcessingNode()
: Node("processing_node")
{
  // Declare parameters
  this->declare_parameter("command_rate", 100);
  this->declare_parameter("motion_constraints", "{}");

  // Get parameters
  command_rate_ = this->get_parameter("command_rate").as_int();
  motion_constraints_json_ = this->get_parameter("motion_constraints").as_string();

  // Create subscribers
  rl_actions_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/rl_actions", 10, std::bind(&ProcessingNode::rlActionsCallback, this, std::placeholders::_1));
    
  usb_commands_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/usb_commands", 10, std::bind(&ProcessingNode::usbCommandsCallback, this, std::placeholders::_1));
    
  aligned_sensor_data_sub_ = this->create_subscription<dodo_msgs::msg::AlignedSensorData>(
    "/aligned_sensor_data", 10, std::bind(&ProcessingNode::alignedSensorDataCallback, this, std::placeholders::_1));

  // Create publisher
  processed_commands_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/processed_commands", 10);

  // Create timer for processing commands
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / command_rate_),
    std::bind(&ProcessingNode::processCommands, this));

  RCLCPP_INFO(this->get_logger(), "Processing Node initialized with command rate %d Hz", command_rate_);
}

ProcessingNode::~ProcessingNode()
{
  RCLCPP_INFO(this->get_logger(), "Processing Node shutting down");
}

void ProcessingNode::rlActionsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(rl_actions_mutex_);
  latest_rl_actions_ = msg;
}

void ProcessingNode::usbCommandsCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(usb_command_mutex_);
  latest_usb_command_ = msg;
}

void ProcessingNode::alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  latest_sensor_data_ = msg;
}

void ProcessingNode::processCommands()
{
  // Combine RL actions and USB commands
  auto combined_command = combineCommands();
  
  // Apply motion constraints
  applyMotionConstraints(combined_command);
  
  // Publish processed command
  processed_commands_pub_->publish(combined_command);
}

sensor_msgs::msg::JointState ProcessingNode::combineCommands()
{
  // Create a new joint state message
  sensor_msgs::msg::JointState combined_command;
  combined_command.header.stamp = this->now();
  
  // Add RL actions if available
  {
    std::lock_guard<std::mutex> lock(rl_actions_mutex_);
    if (latest_rl_actions_) {
      combined_command.name = latest_rl_actions_->name;
      combined_command.position = latest_rl_actions_->position;
      combined_command.velocity = latest_rl_actions_->velocity;
      combined_command.effort = latest_rl_actions_->effort;
    }
  }
  
  // Modify based on USB commands if available
  {
    std::lock_guard<std::mutex> lock(usb_command_mutex_);
    if (latest_usb_command_) {
      // Simple example: adjust positions based on command
      // In a real implementation, this would be more sophisticated
      if (!combined_command.position.empty()) {
        switch (latest_usb_command_->data) {
          case 1:  // FORWARD
            combined_command.position[0] += 0.1;
            break;
          case 2:  // BACKWARD
            combined_command.position[0] -= 0.1;
            break;
          case 3:  // LEFT
            combined_command.position[1] += 0.1;
            break;
          case 4:  // RIGHT
            combined_command.position[1] -= 0.1;
            break;
          default:
            // No adjustment for STOP or unknown commands
            break;
        }
      }
    }
  }
  
  return combined_command;
}

void ProcessingNode::applyMotionConstraints(sensor_msgs::msg::JointState & cmd)
{
  // Only apply constraints if we have valid positions
  if (cmd.position.empty()) {
    return;
  }
  
  try {
    // Parse motion constraints JSON
    auto constraints = nlohmann::json::parse(motion_constraints_json_);
    
    // Apply constraints to each joint
    for (size_t i = 0; i < cmd.name.size() && i < cmd.position.size(); ++i) {
      const auto& joint_name = cmd.name[i];
      
      // Check if we have constraints for this joint
      if (constraints.contains(joint_name)) {
        auto& joint_constraints = constraints[joint_name];
        
        // Apply min/max position constraints
        if (joint_constraints.contains("min_position") && 
            joint_constraints.contains("max_position")) {
          double min_pos = joint_constraints["min_position"];
          double max_pos = joint_constraints["max_position"];
          
          // Clamp position to min/max
          cmd.position[i] = std::max(min_pos, std::min(cmd.position[i], max_pos));
        }
      }
    }
  } catch (const nlohmann::json::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error parsing motion constraints JSON: %s", e.what());
  }
}

}  // namespace dodo_processing

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_processing::ProcessingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}