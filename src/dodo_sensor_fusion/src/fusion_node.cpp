#include "dodo_sensor_fusion/fusion_node.hpp"
#include <algorithm>
#include <cmath>

namespace dodo_sensor_fusion
{

FusionNode::FusionNode()
: Node("fusion_node")
{
  // Declare parameters
  this->declare_parameter("max_time_diff", 0.01);
  this->declare_parameter("fusion_rate", 100);

  // Get parameters
  max_time_diff_ = this->get_parameter("max_time_diff").as_double();
  fusion_rate_ = this->get_parameter("fusion_rate").as_int();

  // Create subscribers
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_raw", 10, std::bind(&FusionNode::imuCallback, this, std::placeholders::_1));
    
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/motor_states", 10, std::bind(&FusionNode::motorStatesCallback, this, std::placeholders::_1));

  // Create publisher
  aligned_data_pub_ = this->create_publisher<dodo_msgs::msg::AlignedSensorData>("/aligned_sensor_data", 10);

  // Create timer for sensor fusion
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / fusion_rate_),
    std::bind(&FusionNode::fuseSensorData, this));

  RCLCPP_INFO(this->get_logger(), "Fusion Node initialized with fusion rate %d Hz", fusion_rate_);
  RCLCPP_INFO(this->get_logger(), "Maximum time difference between sensors: %.3f s", max_time_diff_);
}

FusionNode::~FusionNode()
{
  RCLCPP_INFO(this->get_logger(), "Fusion Node shutting down");
}

void FusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  
  // Add message to queue
  imu_queue_.push_back(msg);
  
  // Limit queue size
  while (imu_queue_.size() > MAX_QUEUE_SIZE) {
    imu_queue_.pop_front();
  }
}

void FusionNode::motorStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_mutex_);
  
  // Add message to queue
  joint_queue_.push_back(msg);
  
  // Limit queue size
  while (joint_queue_.size() > MAX_QUEUE_SIZE) {
    joint_queue_.pop_front();
  }
}

void FusionNode::fuseSensorData()
{
  // Find closest IMU and joint state messages
  sensor_msgs::msg::Imu::SharedPtr imu_msg;
  sensor_msgs::msg::JointState::SharedPtr joint_msg;
  double time_diff;
  
  if (!findClosestMessages(imu_msg, joint_msg, time_diff)) {
    return;  // No matching messages found
  }
  
  // Create aligned sensor data message
  auto aligned_msg = std::make_unique<dodo_msgs::msg::AlignedSensorData>();
  aligned_msg->header.stamp = this->now();
  aligned_msg->imu_data = *imu_msg;
  aligned_msg->joint_states = *joint_msg;
  aligned_msg->time_offset = time_diff;
  
  // Publish aligned sensor data
  aligned_data_pub_->publish(std::move(aligned_msg));
}

bool FusionNode::findClosestMessages(sensor_msgs::msg::Imu::SharedPtr & imu_msg,
                                    sensor_msgs::msg::JointState::SharedPtr & joint_msg,
                                    double & time_diff)
{
  // Get copies of the queues to avoid holding locks for too long
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_copy;
  std::deque<sensor_msgs::msg::JointState::SharedPtr> joint_queue_copy;
  
  {
    std::lock_guard<std::mutex> imu_lock(imu_mutex_);
    imu_queue_copy = imu_queue_;
  }
  
  {
    std::lock_guard<std::mutex> joint_lock(joint_mutex_);
    joint_queue_copy = joint_queue_;
  }
  
  // Check if we have enough messages
  if (imu_queue_copy.empty() || joint_queue_copy.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                         "Not enough messages for fusion: IMU=%zu, Joint=%zu",
                         imu_queue_copy.size(), joint_queue_copy.size());
    return false;
  }
  
  // Find the closest pair of messages by timestamp
  double min_time_diff = std::numeric_limits<double>::max();
  size_t best_imu_idx = 0;
  size_t best_joint_idx = 0;
  
  for (size_t i = 0; i < imu_queue_copy.size(); ++i) {
    const auto& imu = imu_queue_copy[i];
    rclcpp::Time imu_time = imu->header.stamp;
    
    for (size_t j = 0; j < joint_queue_copy.size(); ++j) {
      const auto& joint = joint_queue_copy[j];
      rclcpp::Time joint_time = joint->header.stamp;
      
      double dt = std::abs((imu_time - joint_time).seconds());
      
      if (dt < min_time_diff) {
        min_time_diff = dt;
        best_imu_idx = i;
        best_joint_idx = j;
      }
    }
  }
  
  // Check if the time difference is acceptable
  if (min_time_diff > max_time_diff_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Time difference too large: %.3f s > %.3f s",
                         min_time_diff, max_time_diff_);
    return false;
  }
  
  // Return the best matching messages
  imu_msg = imu_queue_copy[best_imu_idx];
  joint_msg = joint_queue_copy[best_joint_idx];
  time_diff = min_time_diff;
  
  return true;
}

}  // namespace dodo_sensor_fusion

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_sensor_fusion::FusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}