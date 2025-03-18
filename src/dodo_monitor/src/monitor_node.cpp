#include "dodo_monitor/monitor_node.hpp"
#include <cmath>
#include <numeric>
#include <sstream>

namespace dodo_monitor
{

MonitorNode::MonitorNode()
: Node("monitor_node")
{
  // Declare parameters
  this->declare_parameter("imu_noise_threshold", 0.05);       // m/s²
  this->declare_parameter("joint_position_noise_threshold", 0.01); // rad
  this->declare_parameter("check_rate", 10);                  // Hz

  // Get parameters
  imu_noise_threshold_ = this->get_parameter("imu_noise_threshold").as_double();
  joint_position_noise_threshold_ = this->get_parameter("joint_position_noise_threshold").as_double();
  check_rate_ = this->get_parameter("check_rate").as_int();

  // Create subscriber
  aligned_sensor_data_sub_ = this->create_subscription<dodo_msgs::msg::AlignedSensorData>(
    "/aligned_sensor_data", 10, std::bind(&MonitorNode::alignedSensorDataCallback, this, std::placeholders::_1));

  // Create publisher
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/sensor_diagnostics", 10);

  // Create timer for monitoring checks
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / check_rate_),
    std::bind(&MonitorNode::checkSensors, this));

  RCLCPP_INFO(this->get_logger(), "Monitor Node initialized with check rate %d Hz", check_rate_);
  RCLCPP_INFO(this->get_logger(), "IMU noise threshold: %.3f m/s², Joint position noise threshold: %.3f rad",
             imu_noise_threshold_, joint_position_noise_threshold_);
}

MonitorNode::~MonitorNode()
{
  RCLCPP_INFO(this->get_logger(), "Monitor Node shutting down");
}

void MonitorNode::alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg)
{
  // Store sensor data
  recent_sensor_data_.push_back(msg);
  
  // Limit size of recent sensor data
  if (recent_sensor_data_.size() > MAX_HISTORY_SIZE) {
    recent_sensor_data_.erase(recent_sensor_data_.begin());
  }
  
  // Update IMU history
  const auto& imu = msg->imu_data;
  addValue(imu_accel_x_history_, imu.linear_acceleration.x, MAX_HISTORY_SIZE);
  addValue(imu_accel_y_history_, imu.linear_acceleration.y, MAX_HISTORY_SIZE);
  addValue(imu_accel_z_history_, imu.linear_acceleration.z, MAX_HISTORY_SIZE);
  addValue(imu_gyro_x_history_, imu.angular_velocity.x, MAX_HISTORY_SIZE);
  addValue(imu_gyro_y_history_, imu.angular_velocity.y, MAX_HISTORY_SIZE);
  addValue(imu_gyro_z_history_, imu.angular_velocity.z, MAX_HISTORY_SIZE);
  
  // Update joint position history
  const auto& joints = msg->joint_states;
  for (size_t i = 0; i < joints.name.size() && i < joints.position.size(); ++i) {
    const auto& joint_name = joints.name[i];
    double position = joints.position[i];
    
    // Create queue if it doesn't exist
    if (joint_position_history_.find(joint_name) == joint_position_history_.end()) {
      joint_position_history_[joint_name] = std::deque<double>();
    }
    
    // Add value to queue
    addValue(joint_position_history_[joint_name], position, MAX_HISTORY_SIZE);
  }
}

void MonitorNode::checkSensors()
{
  // Check if we have enough data
  if (recent_sensor_data_.empty()) {
    return;
  }
  
  // Create diagnostic array message
  auto diag_array = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  diag_array->header.stamp = this->now();
  
  // Monitor IMU
  diag_array->status.push_back(monitorIMU());
  
  // Monitor joints
  diag_array->status.push_back(monitorJoints());
  
  // Publish diagnostics
  diagnostics_pub_->publish(std::move(diag_array));
}

diagnostic_msgs::msg::DiagnosticStatus MonitorNode::monitorIMU()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "IMU Sensor";
  status.hardware_id = "imu0";
  
  // Check if we have enough data for variance calculation
  if (imu_accel_x_history_.size() < 10) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Not enough data for noise analysis";
    return status;
  }
  
  // Calculate variance of IMU data (as a measure of noise)
  double var_accel_x = calculateVariance(imu_accel_x_history_);
  double var_accel_y = calculateVariance(imu_accel_y_history_);
  double var_accel_z = calculateVariance(imu_accel_z_history_);
  double var_gyro_x = calculateVariance(imu_gyro_x_history_);
  double var_gyro_y = calculateVariance(imu_gyro_y_history_);
  double var_gyro_z = calculateVariance(imu_gyro_z_history_);
  
  // Add values to status message
  diagnostic_msgs::msg::KeyValue kv;
  
  kv.key = "accel_x_variance";
  kv.value = std::to_string(var_accel_x);
  status.values.push_back(kv);
  
  kv.key = "accel_y_variance";
  kv.value = std::to_string(var_accel_y);
  status.values.push_back(kv);
  
  kv.key = "accel_z_variance";
  kv.value = std::to_string(var_accel_z);
  status.values.push_back(kv);
  
  kv.key = "gyro_x_variance";
  kv.value = std::to_string(var_gyro_x);
  status.values.push_back(kv);
  
  kv.key = "gyro_y_variance";
  kv.value = std::to_string(var_gyro_y);
  status.values.push_back(kv);
  
  kv.key = "gyro_z_variance";
  kv.value = std::to_string(var_gyro_z);
  status.values.push_back(kv);
  
  // Check if variance exceeds threshold
  double max_accel_var = std::max({var_accel_x, var_accel_y, var_accel_z});
  
  if (max_accel_var > imu_noise_threshold_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    std::stringstream ss;
    ss << "IMU acceleration noise too high: " << max_accel_var << " > " << imu_noise_threshold_;
    status.message = ss.str();
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "IMU working normally";
  }
  
  return status;
}

diagnostic_msgs::msg::DiagnosticStatus MonitorNode::monitorJoints()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Joint Position Sensors";
  status.hardware_id = "joints";
  
  // Check if we have joint position data
  if (joint_position_history_.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "No joint position data received";
    return status;
  }
  
  // Check each joint's position variance
  bool any_warnings = false;
  std::stringstream warning_msg;
  
  for (const auto& joint_pair : joint_position_history_) {
    const auto& joint_name = joint_pair.first;
    const auto& position_history = joint_pair.second;
    
    // Skip if not enough data
    if (position_history.size() < 10) {
      continue;
    }
    
    // Calculate variance
    double variance = calculateVariance(position_history);
    
    // Add to status message
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = joint_name + "_position_variance";
    kv.value = std::to_string(variance);
    status.values.push_back(kv);
    
    // Check against threshold
    if (variance > joint_position_noise_threshold_) {
      any_warnings = true;
      warning_msg << joint_name << " noise: " << variance << ", ";
    }
  }
  
  if (any_warnings) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Joint position noise too high: " + warning_msg.str();
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Joint position sensors working normally";
  }
  
  return status;
}

double MonitorNode::calculateVariance(const std::deque<double> & values)
{
  if (values.size() < 2) {
    return 0.0;
  }
  
  // Calculate mean
  double sum = std::accumulate(values.begin(), values.end(), 0.0);
  double mean = sum / values.size();
  
  // Calculate variance
  double sq_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0);
  double variance = sq_sum / values.size() - mean * mean;
  
  return variance;
}

void MonitorNode::addValue(std::deque<double> & queue, double value, size_t max_size)
{
  queue.push_back(value);
  
  while (queue.size() > max_size) {
    queue.pop_front();
  }
}

}  // namespace dodo_monitor

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_monitor::MonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}