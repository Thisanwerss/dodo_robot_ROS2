#ifndef DODO_MONITOR_NODE_HPP
#define DODO_MONITOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <dodo_msgs/msg/aligned_sensor_data.hpp>
#include <deque>
#include <string>
#include <vector>
#include <map>

namespace dodo_monitor
{

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode();
  virtual ~MonitorNode();

private:
  // Timer callback
  void checkSensors();
  
  // Subscriber callback
  void alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg);
  
  // Monitoring functions
  diagnostic_msgs::msg::DiagnosticStatus monitorIMU();
  diagnostic_msgs::msg::DiagnosticStatus monitorJoints();
  
  // Helper functions
  double calculateVariance(const std::deque<double> & values);
  void addValue(std::deque<double> & queue, double value, size_t max_size);

  // Parameters
  double imu_noise_threshold_;
  double joint_position_noise_threshold_;
  int check_rate_;
  
  // Data storage
  std::vector<dodo_msgs::msg::AlignedSensorData::SharedPtr> recent_sensor_data_;
  
  // Sensor data history for noise calculation
  std::map<std::string, std::deque<double>> joint_position_history_;
  std::deque<double> imu_accel_x_history_;
  std::deque<double> imu_accel_y_history_;
  std::deque<double> imu_accel_z_history_;
  std::deque<double> imu_gyro_x_history_;
  std::deque<double> imu_gyro_y_history_;
  std::deque<double> imu_gyro_z_history_;
  
  // Maximum history size
  const size_t MAX_HISTORY_SIZE = 50;
  
  // Subscriber
  rclcpp::Subscription<dodo_msgs::msg::AlignedSensorData>::SharedPtr aligned_sensor_data_sub_;
  
  // Publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dodo_monitor

#endif  // DODO_MONITOR_NODE_HPP