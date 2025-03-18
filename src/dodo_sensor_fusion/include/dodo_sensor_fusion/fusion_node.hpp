#ifndef DODO_FUSION_NODE_HPP
#define DODO_FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dodo_msgs/msg/aligned_sensor_data.hpp>
#include <mutex>
#include <deque>

namespace dodo_sensor_fusion
{

class FusionNode : public rclcpp::Node
{
public:
  FusionNode();
  virtual ~FusionNode();

private:
  // Timer callback
  void fuseSensorData();
  
  // Subscribers callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void motorStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  // Helper functions
  bool findClosestMessages(sensor_msgs::msg::Imu::SharedPtr & imu_msg,
                          sensor_msgs::msg::JointState::SharedPtr & joint_msg,
                          double & time_diff);

  // Parameters
  double max_time_diff_;
  int fusion_rate_;
  
  // Message queues
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
  std::deque<sensor_msgs::msg::JointState::SharedPtr> joint_queue_;
  
  // Mutexes for thread safety
  std::mutex imu_mutex_;
  std::mutex joint_mutex_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  
  // Publisher
  rclcpp::Publisher<dodo_msgs::msg::AlignedSensorData>::SharedPtr aligned_data_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Maximum queue sizes
  const size_t MAX_QUEUE_SIZE = 100;
};

}  // namespace dodo_sensor_fusion

#endif  // DODO_FUSION_NODE_HPP