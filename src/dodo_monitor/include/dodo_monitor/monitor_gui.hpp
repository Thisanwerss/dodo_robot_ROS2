#ifndef DODO_MONITOR_GUI_HPP
#define DODO_MONITOR_GUI_HPP

#include <QMainWindow>
#include <QTimer>
#include <QTableWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QStandardItemModel>
#include <QStandardItem>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "dodo_monitor/monitor_node.hpp"

namespace dodo_monitor
{

class MonitorGUI : public QMainWindow
{
  Q_OBJECT // <-- This macro requires Qt MOC processing

public:
  MonitorGUI(std::shared_ptr<MonitorNode> monitor_node, QWidget* parent = nullptr);
  virtual ~MonitorGUI();

public slots:
  void updateGUI();
  void handleNodeSelectionChanged();
  void handleTopicSelectionChanged();
  void refreshData();

private:
  void setupUI();
  void updateNodesView();
  void updateTopicsView();
  void updateDiagnosticsView();
  void updateSensorDataView();

  std::shared_ptr<MonitorNode> monitor_node_;
  
  // UI Components
  QTabWidget* tab_widget_;
  
  // Nodes tab
  QWidget* nodes_tab_;
  QTreeWidget* nodes_tree_;
  QTableWidget* node_details_table_;
  
  // Topics tab
  QWidget* topics_tab_;
  QTreeWidget* topics_tree_;
  QTableWidget* topic_details_table_;
  
  // Diagnostics tab
  QWidget* diagnostics_tab_;
  QTreeWidget* diagnostics_tree_;
  
  // Sensor Data tab
  QWidget* sensor_tab_;
  QTableWidget* imu_table_;
  QTableWidget* joints_table_;
  
  QTimer* update_timer_;
  
  // Selected items
  QString selected_node_;
  QString selected_topic_;
};

}  // namespace dodo_monitor

#endif  // DODO_MONITOR_GUI_HPP