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

  // Get parameters
  model_path_ = this->get_parameter("model_path").as_string();
  control_rate_ = this->get_parameter("control_rate").as_int();
  joint_names_ = this->get_parameter("joint_names").as_string_array();

  // Initialize PPO model
  ppo_model_ = std::make_unique<PPOModel>(model_path_);

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
  // Placeholder for observation (would typically come from sensor data)
  std::vector<double> observation(16, 0.0);  // Example with 16 observation values

  // Get action from PPO model
  std::vector<double> actions = ppo_model_->getAction(observation);

  // Create joint state message
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = this->now();
  joint_state_msg->name = joint_names_;
  joint_state_msg->position = actions;

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