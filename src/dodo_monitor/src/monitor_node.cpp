#include "dodo_monitor/monitor_node.hpp"
#include <cmath>
#include <numeric>
#include <sstream>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

namespace dodo_monitor
{

MonitorNode::MonitorNode()
: Node("monitor_node")
{
  // Declare parameters
  this->declare_parameter("imu_noise_threshold", 0.05);       // m/s²
  this->declare_parameter("joint_position_noise_threshold", 0.01); // rad
  this->declare_parameter("check_rate", 10);                  // Hz
  this->declare_parameter("system_info_rate", 1);              // Hz

  // Get parameters
  imu_noise_threshold_ = this->get_parameter("imu_noise_threshold").as_double();
  joint_position_noise_threshold_ = this->get_parameter("joint_position_noise_threshold").as_double();
  check_rate_ = this->get_parameter("check_rate").as_int();
  system_info_rate_ = this->get_parameter("system_info_rate").as_int();

  // Create subscriber
  aligned_sensor_data_sub_ = this->create_subscription<dodo_msgs::msg::AlignedSensorData>(
    "/aligned_sensor_data", 10, std::bind(&MonitorNode::alignedSensorDataCallback, this, std::placeholders::_1));

  // Create publisher
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/sensor_diagnostics", 10);

  // Create timers
  sensor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / check_rate_),
    std::bind(&MonitorNode::checkSensors, this));
  
  system_info_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / system_info_rate_),
    std::bind(&MonitorNode::updateSystemInfo, this));

  RCLCPP_INFO(this->get_logger(), "Monitor Node initialized with check rate %d Hz", check_rate_);
  RCLCPP_INFO(this->get_logger(), "System info update rate: %d Hz", system_info_rate_);
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
  
  // Update topic frequency
  updateTopicFrequency("/aligned_sensor_data");
}

// Getter methods
std::map<std::string, NodeInfo> MonitorNode::getNodesInfo()
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  return nodes_info_;
}

std::map<std::string, TopicInfo> MonitorNode::getTopicsInfo()
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  return topics_info_;
}

std::vector<diagnostic_msgs::msg::DiagnosticStatus> MonitorNode::getLatestDiagnostics()
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  return latest_diagnostics_;
}

dodo_msgs::msg::AlignedSensorData::SharedPtr MonitorNode::getLatestSensorData() const
{
  if (recent_sensor_data_.empty()) {
    return nullptr;
  }
  return recent_sensor_data_.back();
}

// Add a node to the known nodes list
void MonitorNode::addKnownNode(const std::string& node_name, const std::string& node_namespace)
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  
  std::string full_name = node_namespace;
  if (full_name.empty() || full_name == "/") {
    full_name = "/" + node_name;
  } else {
    full_name = full_name + "/" + node_name;
  }
  
  // Only add if it doesn't exist already
  if (nodes_info_.find(full_name) == nodes_info_.end()) {
    NodeInfo node_info;
    node_info.node_name = node_name;
    node_info.node_namespace = node_namespace;
    nodes_info_[full_name] = node_info;
    
    RCLCPP_INFO(this->get_logger(), "Added node to tracking: %s", full_name.c_str());
  }
}

// Add a topic to the known topics list
void MonitorNode::addKnownTopic(const std::string& topic_name, const std::string& topic_type)
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  
  // Only add if it doesn't exist already
  if (topics_info_.find(topic_name) == topics_info_.end()) {
    TopicInfo topic_info;
    topic_info.topic_name = topic_name;
    topic_info.topic_types.push_back(topic_type);
    topic_info.msg_frequency = 0.0;
    topic_info.last_msg_time = this->now();
    topics_info_[topic_name] = topic_info;
    
    RCLCPP_INFO(this->get_logger(), "Added topic to tracking: %s (%s)", 
               topic_name.c_str(), topic_type.c_str());
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
  auto imu_status = monitorIMU();
  diag_array->status.push_back(imu_status);
  
  // Monitor joints
  auto joint_status = monitorJoints();
  diag_array->status.push_back(joint_status);
  
  // Monitor system
  auto system_status = monitorSystem();
  diag_array->status.push_back(system_status);
  
  // Store latest diagnostics for GUI
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    latest_diagnostics_.clear();
    latest_diagnostics_.push_back(imu_status);
    latest_diagnostics_.push_back(joint_status);
    latest_diagnostics_.push_back(system_status);
  }
  
  // Publish diagnostics
  diagnostics_pub_->publish(std::move(diag_array));
}

void MonitorNode::updateSystemInfo()
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  
  // For ROS2, we need to use a different approach than get_node_names_and_namespaces()
  // In this implementation, we'll just maintain a simple list of known nodes
  
  // Make sure we have the aligned_sensor_data topic
  if (topics_info_.find("/aligned_sensor_data") == topics_info_.end()) {
    TopicInfo topic_info;
    topic_info.topic_name = "/aligned_sensor_data";
    topic_info.topic_types.push_back("dodo_msgs/msg/AlignedSensorData");
    topic_info.msg_frequency = 0.0;
    topic_info.last_msg_time = this->now();
    topics_info_["/aligned_sensor_data"] = topic_info;
  }
  
  // Make sure we have the sensor_diagnostics topic
  if (topics_info_.find("/sensor_diagnostics") == topics_info_.end()) {
    TopicInfo topic_info;
    topic_info.topic_name = "/sensor_diagnostics";
    topic_info.topic_types.push_back("diagnostic_msgs/msg/DiagnosticArray");
    topic_info.msg_frequency = 0.0;
    topic_info.last_msg_time = this->now();
    topics_info_["/sensor_diagnostics"] = topic_info;
  }
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

diagnostic_msgs::msg::DiagnosticStatus MonitorNode::monitorSystem()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ROS2 System Status";
  status.hardware_id = "ros2_system";
  
  std::lock_guard<std::mutex> lock(info_mutex_);
  
  // Add number of nodes and topics
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "active_nodes";
  kv.value = std::to_string(nodes_info_.size());
  status.values.push_back(kv);
  
  kv.key = "active_topics";
  kv.value = std::to_string(topics_info_.size());
  status.values.push_back(kv);
  
  // Add information about which nodes are publishing to sensor topics
  // Note: This would be better with actual publisher tracking
  bool sensor_publishers_active = false;
  for (const auto& topic_pair : topics_info_) {
    if (topic_pair.first == "/aligned_sensor_data" && !topic_pair.second.publisher_nodes.empty()) {
      sensor_publishers_active = true;
      break;
    }
  }
  
  kv.key = "sensor_data_available";
  kv.value = recent_sensor_data_.empty() ? "false" : "true";
  status.values.push_back(kv);
  
  // Add this node's name and namespace
  kv.key = "monitor_node_name";
  kv.value = this->get_name();
  status.values.push_back(kv);
  
  kv.key = "monitor_node_namespace";
  kv.value = this->get_namespace();
  status.values.push_back(kv);
  
  // Set status level
  if (!sensor_publishers_active || recent_sensor_data_.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Sensor data not available or incomplete";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ROS2 system running normally";
  }
  
  return status;
}

void MonitorNode::updateTopicFrequency(const std::string& topic_name)
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  
  if (topics_info_.find(topic_name) != topics_info_.end()) {
    auto& topic_info = topics_info_[topic_name];
    auto now = this->now();
    
    // Skip first message or if last time is in future (clock jump)
    if (topic_info.last_msg_time.nanoseconds() == 0 || 
        topic_info.last_msg_time > now) {
      topic_info.last_msg_time = now;
      return;
    }
    
    // Calculate time difference in seconds
    double dt = (now - topic_info.last_msg_time).seconds();
    if (dt > 0.0) {
      // Simple low-pass filter for frequency calculation
      double alpha = 0.3; // Smoothing factor
      double instant_frequency = 1.0 / dt;
      
      if (topic_info.msg_frequency == 0.0) {
        topic_info.msg_frequency = instant_frequency;
      } else {
        topic_info.msg_frequency = 
          alpha * instant_frequency + (1.0 - alpha) * topic_info.msg_frequency;
      }
    }
    
    topic_info.last_msg_time = now;
  }
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
  
  // Add some known nodes for demo purposes
  node->addKnownNode("monitor_node", "/");
  node->addKnownNode("gui_node", "/");
  node->addKnownNode("imu_node", "/");
  node->addKnownNode("canbus_node", "/");
  node->addKnownNode("usb_command_node", "/");
  node->addKnownNode("processing_node", "/");
  node->addKnownNode("fusion_node", "/");
  node->addKnownNode("safety_node", "/");
  node->addKnownNode("state_manager_node", "/");
  
  // Add some known topics for demo purposes
  node->addKnownTopic("/aligned_sensor_data", "dodo_msgs/msg/AlignedSensorData");
  node->addKnownTopic("/imu_data", "sensor_msgs/msg/Imu");
  node->addKnownTopic("/joint_states", "sensor_msgs/msg/JointState");
  node->addKnownTopic("/motor_commands", "std_msgs/msg/Float32MultiArray");
  node->addKnownTopic("/sensor_diagnostics", "diagnostic_msgs/msg/DiagnosticArray");
  node->addKnownTopic("/robot_state", "std_msgs/msg/String");
  node->addKnownTopic("/safety_status", "std_msgs/msg/Bool");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}