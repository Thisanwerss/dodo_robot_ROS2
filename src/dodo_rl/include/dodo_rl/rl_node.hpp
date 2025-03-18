#ifndef DODO_RL_NODE_HPP
#define DODO_RL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>

#include "dodo_rl/ppo_model.hpp"

namespace dodo_rl
{

class RLNode : public rclcpp::Node
{
public:
  RLNode();
  virtual ~RLNode();

private:
  // Timer callback
  void controlLoop();

  // Parameters
  std::string model_path_;
  std::vector<std::string> joint_names_;
  int control_rate_;

  // PPO model
  std::unique_ptr<PPOModel> ppo_model_;

  // Publisher for RL actions
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rl_actions_pub_;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dodo_rl

#endif  // DODO_RL_NODE_HPP