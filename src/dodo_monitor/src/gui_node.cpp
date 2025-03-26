#include "dodo_monitor/gui_node.hpp"
#include "dodo_monitor/monitor_gui.hpp"
#include <QApplication>
#include <string>
#include <vector>
#include <algorithm>

namespace dodo_monitor
{

GUINode::GUINode(std::shared_ptr<MonitorNode> monitor_node)
: Node("gui_node"), monitor_node_(monitor_node)
{
  RCLCPP_INFO(this->get_logger(), "GUI Node initialized");
}

GUINode::~GUINode()
{
  RCLCPP_INFO(this->get_logger(), "GUI Node shutting down");
}

void GUINode::startGUI(int argc, char** argv)
{
  QApplication app(argc, argv);
  gui_ = std::make_shared<MonitorGUI>(monitor_node_);
  gui_->show();
  app.exec();
}

}  // namespace dodo_monitor

// GUI Main already defined in CMakeLists.txt
// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create monitor node (will run in a separate thread)
  auto monitor_node = std::make_shared<dodo_monitor::MonitorNode>();
  
  // Add some known nodes for demo purposes
  monitor_node->addKnownNode("monitor_node", "/");
  monitor_node->addKnownNode("gui_node", "/");
  monitor_node->addKnownNode("imu_node", "/");
  monitor_node->addKnownNode("canbus_node", "/");
  monitor_node->addKnownNode("usb_command_node", "/");
  monitor_node->addKnownNode("processing_node", "/");
  monitor_node->addKnownNode("fusion_node", "/");
  monitor_node->addKnownNode("safety_node", "/");
  monitor_node->addKnownNode("state_manager_node", "/");
  
  // Add some known topics for demo purposes
  monitor_node->addKnownTopic("/aligned_sensor_data", "dodo_msgs/msg/AlignedSensorData");
  monitor_node->addKnownTopic("/imu_data", "sensor_msgs/msg/Imu");
  monitor_node->addKnownTopic("/joint_states", "sensor_msgs/msg/JointState");
  monitor_node->addKnownTopic("/motor_commands", "std_msgs/msg/Float32MultiArray");
  monitor_node->addKnownTopic("/sensor_diagnostics", "diagnostic_msgs/msg/DiagnosticArray");
  monitor_node->addKnownTopic("/robot_state", "std_msgs/msg/String");
  monitor_node->addKnownTopic("/safety_status", "std_msgs/msg/Bool");
  
  // Create GUI node
  auto gui_node = std::make_shared<dodo_monitor::GUINode>(monitor_node);
  
  // Create separate thread for ROS 2 spinning
  std::thread spin_thread([&monitor_node]() {
    rclcpp::spin(monitor_node);
  });
  
  // Start GUI (this will block until GUI is closed)
  gui_node->startGUI(argc, argv);
  
  // Cleanup and exit
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  
  return 0;
}