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
  QTimer::singleShot(2000, [this]() {
    RCLCPP_INFO(this->get_logger(), "Delayed node population triggered");

    monitor_node_->addKnownNode("monitor_node", "/");
    monitor_node_->addKnownNode("gui_node", "/");
    monitor_node_->addKnownNode("imu_node", "/");
    monitor_node_->addKnownNode("canbus_node", "/");
    monitor_node_->addKnownNode("usb_command_node", "/");
    monitor_node_->addKnownNode("processing_node", "/");
    monitor_node_->addKnownNode("fusion_node", "/");
    monitor_node_->addKnownNode("safety_node", "/");
    monitor_node_->addKnownNode("state_manager_node", "/");

    monitor_node_->addKnownTopic("/aligned_sensor_data", "dodo_msgs/msg/AlignedSensorData");
    monitor_node_->addKnownTopic("/imu_raw", "sensor_msgs/msg/Imu");
    monitor_node_->addKnownTopic("/rl_actions", "sensor_msgs/msg/JointState");
    monitor_node_->addKnownTopic("/motor_states", "sensor_msgs/msg/JointState");
    monitor_node_->addKnownTopic("/sensor_diagnostics", "diagnostic_msgs/msg/DiagnosticArray");
    monitor_node_->addKnownTopic("/robot_state", "std_msgs/msg/String");
    monitor_node_->addKnownTopic("/usb_commands", "std_msgs/msg/Int32");

    gui_->refreshData(); 
  });




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