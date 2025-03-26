#ifndef DODO_MONITOR_GUI_NODE_HPP
#define DODO_MONITOR_GUI_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "dodo_monitor/monitor_node.hpp"

// Forward declare Qt classes
class QApplication;
class QMainWindow;
class QWidget;
class QTreeWidget;
class QTreeWidgetItem;
class QTableWidget;
class QTabWidget; 
class QTimer;

namespace dodo_monitor
{

// Forward declaration of our GUI class
class MonitorGUI;

class GUINode : public rclcpp::Node
{
public:
  GUINode(std::shared_ptr<MonitorNode> monitor_node);
  virtual ~GUINode();
  
  void startGUI(int argc, char** argv);

private:
  std::shared_ptr<MonitorNode> monitor_node_;
  std::shared_ptr<MonitorGUI> gui_;
};

}  // namespace dodo_monitor

#endif  // DODO_MONITOR_GUI_NODE_HPP