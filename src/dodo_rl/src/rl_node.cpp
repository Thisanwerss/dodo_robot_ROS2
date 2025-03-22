#include "dodo_rl/rl_node.hpp"

namespace dodo_rl
{

RLNode::RLNode()
: Node("rl_node")
{
  // Declare parameters
  this->declare_parameter("model_path", "");
  this->declare_parameter("control_rate", 100);
  this->declare_parameter("joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"});
  this->declare_parameter("dummy_mode", false);

  // Get parameters
  model_path_ = this->get_parameter("model_path").as_string();
  control_rate_ = this->get_parameter("control_rate").as_int();
  joint_names_ = this->get_parameter("joint_names").as_string_array();
  dummy_mode_ = this->get_parameter("dummy_mode").as_bool();

  // Initialize PPO model if not in dummy mode
  if (!dummy_mode_) {
    ppo_model_ = std::make_unique<PPOModel>(model_path_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Running in dummy mode - using simulated RL actions");
  }

  // Create publisher
  rl_actions_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/rl_actions", 10);

  // Create timer for control loop
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / control_rate_),
    std::bind(&RLNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "RL Node initialized with control rate %d Hz", control_rate_);
}

RLNode::~RLNode()
{
  RCLCPP_INFO(this->get_logger(), "RL Node shutting down");
}

void RLNode::controlLoop()
{
  // Create joint state message
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = this->now();
  joint_state_msg->name = joint_names_;
  
  // Get actions either from dummy mode or PPO model
  if (dummy_mode_) {
    // Generate dummy sinusoidal actions for each joint
    std::vector<double> dummy_actions;
    static double time_counter = 0.0;
    time_counter += 0.02;  // Increment by 20ms
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      double freq = 0.5 + 0.1 * static_cast<double>(i);  // Different frequency for each joint
      double amplitude = 0.2 + 0.05 * static_cast<double>(i);  // Different amplitude for each joint
      double value = amplitude * sin(freq * time_counter);
      dummy_actions.push_back(value);
    }
    
    joint_state_msg->position = dummy_actions;
    RCLCPP_DEBUG(this->get_logger(), "Publishing dummy RL actions");
  } else {
    // Normal operation with PPO model
    std::vector<double> observation(16, 0.0);  // Example with 16 observation values
    std::vector<double> actions = ppo_model_->getAction(observation);
    joint_state_msg->position = actions;
  }

  // Publish joint state message
  rl_actions_pub_->publish(std::move(joint_state_msg));
}

}  // namespace dodo_rl

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_rl::RLNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}