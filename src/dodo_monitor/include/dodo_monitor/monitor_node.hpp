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

struct NodeInfo {
  std::string node_name;
  std::string node_namespace;
  std::vector<std::string> published_topics;
  std::vector<std::string> subscribed_topics;
  std::vector<std::string> services;
};

struct TopicInfo {
  std::string topic_name;
  std::vector<std::string> topic_types;
  std::vector<std::string> publisher_nodes;
  std::vector<std::string> subscriber_nodes;
  double msg_frequency;
  rclcpp::Time last_msg_time;
};

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode();
  virtual ~MonitorNode();

  // System information getters
  std::map<std::string, NodeInfo> getNodesInfo();
  std::map<std::string, TopicInfo> getTopicsInfo();
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> getLatestDiagnostics();
  dodo_msgs::msg::AlignedSensorData::SharedPtr getLatestSensorData() const;
  
  // Add a node to the node list for display
  void addKnownNode(const std::string& node_name, const std::string& node_namespace);
  
  // Add a topic to the topic list for display
  void addKnownTopic(const std::string& topic_name, const std::string& topic_type);

private:
  // Timer callbacks
  void checkSensors();
  void updateSystemInfo();
  
  // Subscriber callback
  void alignedSensorDataCallback(const dodo_msgs::msg::AlignedSensorData::SharedPtr msg);
  
  // Monitoring functions
  diagnostic_msgs::msg::DiagnosticStatus monitorIMU();
  diagnostic_msgs::msg::DiagnosticStatus monitorJoints();
  diagnostic_msgs::msg::DiagnosticStatus monitorSystem();
  
  // Helper functions
  double calculateVariance(const std::deque<double> & values);
  void addValue(std::deque<double> & queue, double value, size_t max_size);
  void updateTopicFrequency(const std::string& topic_name);

  // Parameters
  double imu_noise_threshold_;
  double joint_position_noise_threshold_;
  int check_rate_;
  int system_info_rate_;
  
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
  
  // System information
  std::map<std::string, NodeInfo> nodes_info_;
  std::map<std::string, TopicInfo> topics_info_;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> latest_diagnostics_;
  mutable std::mutex info_mutex_;
  
  // Maximum history size
  const size_t MAX_HISTORY_SIZE = 50;
  
  // Subscriber
  rclcpp::Subscription<dodo_msgs::msg::AlignedSensorData>::SharedPtr aligned_sensor_data_sub_;
  
  // Publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr system_info_timer_;
};

}  // namespace dodo_monitor

#endif  // DODO_MONITOR_NODE_HPP