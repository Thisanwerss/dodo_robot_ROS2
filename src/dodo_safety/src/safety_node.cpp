#include "dodo_safety/safety_node.hpp"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace dodo_safety
{

SafetyNode::SafetyNode()
: Node("safety_node"), emergency_active_(false)
{
  // Declare parameters
  this->declare_parameter("max_acceleration", 12.0);  // m/s²
  this->declare_parameter("max_tilt", 0.5);          // rad (about 28.6 degrees)
  this->declare_parameter("max_joint_velocity", 5.0); // rad/s
  this->declare_parameter("check_rate", 50);          // Hz

  // Get parameters
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();
  max_tilt_ = this->get_parameter("max_tilt").as_double();
  max_joint_velocity_ = this->get_parameter("max_joint_velocity").as_double();
  check_rate_ = this->get_parameter("check_rate").as_int();

  // Create log directory if it doesn't exist
  std::filesystem::create_directories("logs");
  
  // Open log file with timestamp in filename
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "logs/safety_log_%Y%m%d_%H%M%S.txt");
  log_file_.open(ss.str(), std::ios::out | std::ios::app);
  
  if (!log_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open log file");
  }

  // Create subscriber
  aligned_sensor_data_sub_ = this->create_subscription<dodo_msgs::msg::AlignedSensorData>(
    "/aligned_sensor_data", 10, std::bind(&SafetyNode::alignedSensorDataCallback, this, std::placeholders::_1));

  // Create publisher
  emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);

  // Create timer for safety checks
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / check_rate_),
    std::bind(&SafetyNode::checkSafety, this));

  // Log initialization
  logSafetyEvent("Safety node initialized", false);
  
  RCLCPP_INFO(this->get_logger(), "Safety Node initialized with check rate %d Hz", check_rate_);
  RCLCPP_INFO(this->get_logger(), "Safety thresholds: max_accel=%.1f m/s², max_tilt=%.1f rad, max_joint_vel=%.1f rad/s",
              max_acceleration_, max_tilt_, max_joint_velocity_);
}

SafetyNode::~SafetyNode()
{
  // Log shutdown
  logSafetyEvent("Safety node shutting down", false);
  
  // Close log file
  if (log_file_.is_open()) {
    log_file_.close();
  }
  
  RCLCPP_INFO(this->get_logger(), "Safety Node shutting down");
}

void SafetyNode::alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg)
{
  latest_sensor_data_ = msg;
}

void SafetyNode::checkSafety()
{
  if (!latest_sensor_data_) {
    return;  // No sensor data yet
  }
  
  // Check IMU safety
  bool imu_safe = checkIMUSafety(latest_sensor_data_);
  
  // Check joint safety
  bool joint_safe = checkJointSafety(latest_sensor_data_);
  
  // Determine if emergency stop should be active
  bool should_be_active = !imu_safe || !joint_safe;
  
  // Only publish if state changes
  if (should_be_active != emergency_active_) {
    emergency_active_ = should_be_active;
    
    // Create and publish message
    auto msg = std::make_unique<std_msgs::msg::Bool>();
    msg->data = emergency_active_;
    emergency_stop_pub_->publish(std::move(msg));
    
    // Log event
    if (emergency_active_) {
      logSafetyEvent("Emergency stop activated", true);
    } else {
      logSafetyEvent("Emergency stop deactivated", false);
    }
  }
}

bool SafetyNode::checkIMUSafety(const dodo_msgs::msg::AlignedSensorData::SharedPtr & msg)
{
  const auto& imu = msg->imu_data;
  
  // Calculate total acceleration magnitude
  double accel_x = imu.linear_acceleration.x;
  double accel_y = imu.linear_acceleration.y;
  double accel_z = imu.linear_acceleration.z;
  double accel_magnitude = std::sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  
  // Check if acceleration exceeds threshold
  if (accel_magnitude > max_acceleration_) {
    std::stringstream ss;
    ss << "Excessive acceleration detected: " << accel_magnitude << " m/s² > " << max_acceleration_ << " m/s²";
    logSafetyEvent(ss.str(), true);
    return false;
  }
  
  // Calculate tilt from vertical (assume z is up when robot is level)
  double tilt = std::acos(accel_z / accel_magnitude);
  
  // Check if tilt exceeds threshold
  if (tilt > max_tilt_) {
    std::stringstream ss;
    ss << "Excessive tilt detected: " << tilt << " rad > " << max_tilt_ << " rad";
    logSafetyEvent(ss.str(), true);
    return false;
  }
  
  return true;
}

bool SafetyNode::checkJointSafety(const dodo_msgs::msg::AlignedSensorData::SharedPtr & msg)
{
  const auto& joints = msg->joint_states;
  
  // Check joint velocities
  for (size_t i = 0; i < joints.name.size() && i < joints.velocity.size(); ++i) {
    double velocity = std::abs(joints.velocity[i]);
    
    // Check if velocity exceeds threshold
    if (velocity > max_joint_velocity_) {
      std::stringstream ss;
      ss << "Excessive joint velocity detected on " << joints.name[i] 
         << ": " << velocity << " rad/s > " << max_joint_velocity_ << " rad/s";
      logSafetyEvent(ss.str(), true);
      return false;
    }
  }
  
  return true;
}

void SafetyNode::logSafetyEvent(const std::string & event, bool is_error)
{
  // Log to console
  if (is_error) {
    RCLCPP_ERROR(this->get_logger(), "%s", event.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "%s", event.c_str());
  }
  
  // Log to file
  if (log_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    log_file_ << std::put_time(std::localtime(&in_time_t), "[%Y-%m-%d %H:%M:%S] ");
    if (is_error) {
      log_file_ << "ERROR: ";
    } else {
      log_file_ << "INFO: ";
    }
    log_file_ << event << std::endl;
    log_file_.flush();
  }
}

}  // namespace dodo_safety

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_safety::SafetyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}