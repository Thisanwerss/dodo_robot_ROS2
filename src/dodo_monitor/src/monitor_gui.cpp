#include "dodo_monitor/monitor_gui.hpp"
#include <QGridLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <string>
#include <vector>
#include <algorithm>

namespace dodo_monitor
{

MonitorGUI::MonitorGUI(std::shared_ptr<MonitorNode> monitor_node, QWidget* parent)
: QMainWindow(parent), monitor_node_(monitor_node)
{
  setupUI();
  
  // Set up timer for periodic updates
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &MonitorGUI::updateGUI);
  update_timer_->start(500);  // Update every 500ms
  
  // Initialize with a first update
  updateGUI();
  
  // Set window properties
  setWindowTitle("DODO Robot Monitor");
  resize(1024, 768);
}

MonitorGUI::~MonitorGUI()
{
  update_timer_->stop();
}

void MonitorGUI::setupUI()
{
  tab_widget_ = new QTabWidget(this);
  setCentralWidget(tab_widget_);
  
  // Nodes tab
  nodes_tab_ = new QWidget();
  QHBoxLayout* nodes_layout = new QHBoxLayout(nodes_tab_);
  
  // Set up nodes tree
  nodes_tree_ = new QTreeWidget();
  nodes_tree_->setHeaderLabels({"Node Name", "Namespace"});
  nodes_tree_->setColumnWidth(0, 300);
  connect(nodes_tree_, &QTreeWidget::itemSelectionChanged, this, &MonitorGUI::handleNodeSelectionChanged);
  
  // Set up node details
  node_details_table_ = new QTableWidget();
  node_details_table_->setColumnCount(2);
  node_details_table_->setHorizontalHeaderLabels({"Property", "Value"});
  node_details_table_->horizontalHeader()->setStretchLastSection(true);
  node_details_table_->verticalHeader()->setVisible(false);
  
  nodes_layout->addWidget(nodes_tree_, 1);
  nodes_layout->addWidget(node_details_table_, 1);
  
  // Topics tab
  topics_tab_ = new QWidget();
  QHBoxLayout* topics_layout = new QHBoxLayout(topics_tab_);
  
  // Set up topics tree
  topics_tree_ = new QTreeWidget();
  topics_tree_->setHeaderLabels({"Topic Name", "Type"});
  topics_tree_->setColumnWidth(0, 300);
  connect(topics_tree_, &QTreeWidget::itemSelectionChanged, this, &MonitorGUI::handleTopicSelectionChanged);
  
  // Set up topic details
  topic_details_table_ = new QTableWidget();
  topic_details_table_->setColumnCount(2);
  topic_details_table_->setHorizontalHeaderLabels({"Property", "Value"});
  topic_details_table_->horizontalHeader()->setStretchLastSection(true);
  topic_details_table_->verticalHeader()->setVisible(false);
  
  topics_layout->addWidget(topics_tree_, 1);
  topics_layout->addWidget(topic_details_table_, 1);
  
  // Diagnostics tab
  diagnostics_tab_ = new QWidget();
  QVBoxLayout* diagnostics_layout = new QVBoxLayout(diagnostics_tab_);
  
  // Set up diagnostics tree
  diagnostics_tree_ = new QTreeWidget();
  diagnostics_tree_->setHeaderLabels({"Component", "Status", "Message"});
  diagnostics_tree_->setColumnWidth(0, 200);
  diagnostics_tree_->setColumnWidth(1, 100);
  diagnostics_tree_->setColumnWidth(2, 400);
  
  // Manual refresh button
  QPushButton* refresh_button = new QPushButton("Refresh Data");
  connect(refresh_button, &QPushButton::clicked, this, &MonitorGUI::refreshData);
  
  diagnostics_layout->addWidget(diagnostics_tree_);
  diagnostics_layout->addWidget(refresh_button);
  
  // Sensor Data tab
  sensor_tab_ = new QWidget();
  QVBoxLayout* sensor_layout = new QVBoxLayout(sensor_tab_);
  
  // IMU data
  QGroupBox* imu_group = new QGroupBox("IMU Data");
  QVBoxLayout* imu_layout = new QVBoxLayout(imu_group);
  
  imu_table_ = new QTableWidget();
  imu_table_->setColumnCount(2);
  imu_table_->setHorizontalHeaderLabels({"Parameter", "Value"});
  imu_table_->horizontalHeader()->setStretchLastSection(true);
  imu_table_->verticalHeader()->setVisible(false);
  
  imu_layout->addWidget(imu_table_);
  
  // Joint data
  QGroupBox* joints_group = new QGroupBox("Joint States");
  QVBoxLayout* joints_layout = new QVBoxLayout(joints_group);
  
  joints_table_ = new QTableWidget();
  joints_table_->setColumnCount(3);
  joints_table_->setHorizontalHeaderLabels({"Joint Name", "Position", "Velocity"});
  joints_table_->horizontalHeader()->setStretchLastSection(true);
  joints_table_->verticalHeader()->setVisible(false);
  
  joints_layout->addWidget(joints_table_);
  
  sensor_layout->addWidget(imu_group);
  sensor_layout->addWidget(joints_group);
  
  // Add all tabs
  tab_widget_->addTab(nodes_tab_, "Nodes");
  tab_widget_->addTab(topics_tab_, "Topics");
  tab_widget_->addTab(diagnostics_tab_, "Diagnostics");
  tab_widget_->addTab(sensor_tab_, "Sensor Data");
}

void MonitorGUI::updateGUI()
{
  updateNodesView();
  updateTopicsView();
  updateDiagnosticsView();
  updateSensorDataView();
}

void MonitorGUI::refreshData()
{
  updateGUI();
}

void MonitorGUI::updateNodesView()
{
  // Get node information
  auto nodes_info = monitor_node_->getNodesInfo();
  
  // Store current expanded state
  QMap<QString, bool> expanded_state;
  for (int i = 0; i < nodes_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = nodes_tree_->topLevelItem(i);
    expanded_state[item->text(0)] = item->isExpanded();
  }
  
  // Clear the tree
  nodes_tree_->clear();
  
  // Organize nodes by namespace
  std::map<std::string, std::vector<std::string>> nodes_by_namespace;
  
  for (const auto& node_pair : nodes_info) {
    const auto& node_info = node_pair.second;
    nodes_by_namespace[node_info.node_namespace].push_back(node_info.node_name);
  }
  
  // Add nodes to tree
  for (const auto& ns_pair : nodes_by_namespace) {
    QTreeWidgetItem* ns_item = new QTreeWidgetItem(nodes_tree_);
    ns_item->setText(0, QString::fromStdString(ns_pair.first));
    ns_item->setText(1, "Namespace");
    
    for (const auto& node_name : ns_pair.second) {
      QTreeWidgetItem* node_item = new QTreeWidgetItem(ns_item);
      node_item->setText(0, QString::fromStdString(node_name));
      node_item->setText(1, QString::fromStdString(ns_pair.first));
    }
    
    // Restore expanded state
    if (expanded_state.contains(QString::fromStdString(ns_pair.first))) {
      ns_item->setExpanded(expanded_state[QString::fromStdString(ns_pair.first)]);
    }
  }
  
  // Update details if a node is selected
  if (!selected_node_.isEmpty()) {
    // This would require finding the node again and updating details
    handleNodeSelectionChanged();
  }
}

void MonitorGUI::handleNodeSelectionChanged()
{
  QList<QTreeWidgetItem*> selected_items = nodes_tree_->selectedItems();
  if (selected_items.isEmpty()) {
    selected_node_ = "";
    node_details_table_->setRowCount(0);
    return;
  }
  
  QTreeWidgetItem* selected_item = selected_items.first();
  if (selected_item->parent() != nullptr) {  // Only handle node items (not namespace items)
    selected_node_ = selected_item->text(0) + " " + selected_item->text(1);  // Node name + namespace
    
    // Get node info
    auto nodes_info = monitor_node_->getNodesInfo();
    std::string full_name = selected_item->text(1).toStdString() + "/" + selected_item->text(0).toStdString();
    
    if (nodes_info.find(full_name) != nodes_info.end()) {
      const auto& node_info = nodes_info[full_name];
      
      // Update details table
      node_details_table_->setRowCount(0);
      
      // Add node name
      int row = node_details_table_->rowCount();
      node_details_table_->insertRow(row);
      node_details_table_->setItem(row, 0, new QTableWidgetItem("Node Name"));
      node_details_table_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(node_info.node_name)));
      
      // Add namespace
      row = node_details_table_->rowCount();
      node_details_table_->insertRow(row);
      node_details_table_->setItem(row, 0, new QTableWidgetItem("Namespace"));
      node_details_table_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(node_info.node_namespace)));
      
      // Add published topics
      row = node_details_table_->rowCount();
      node_details_table_->insertRow(row);
      node_details_table_->setItem(row, 0, new QTableWidgetItem("Published Topics"));
      QString published_topics = "";
      for (const auto& topic : node_info.published_topics) {
        published_topics += QString::fromStdString(topic) + "\n";
      }
      node_details_table_->setItem(row, 1, new QTableWidgetItem(published_topics));
      
      // Add subscribed topics
      row = node_details_table_->rowCount();
      node_details_table_->insertRow(row);
      node_details_table_->setItem(row, 0, new QTableWidgetItem("Subscribed Topics"));
      QString subscribed_topics = "";
      for (const auto& topic : node_info.subscribed_topics) {
        subscribed_topics += QString::fromStdString(topic) + "\n";
      }
      node_details_table_->setItem(row, 1, new QTableWidgetItem(subscribed_topics));
      
      // Add services
      row = node_details_table_->rowCount();
      node_details_table_->insertRow(row);
      node_details_table_->setItem(row, 0, new QTableWidgetItem("Services"));
      QString services = "";
      for (const auto& service : node_info.services) {
        services += QString::fromStdString(service) + "\n";
      }
      node_details_table_->setItem(row, 1, new QTableWidgetItem(services));
    }
  }
}

void MonitorGUI::updateTopicsView()
{
  // Get topic information
  auto topics_info = monitor_node_->getTopicsInfo();
  
  // Store current expanded state
  QMap<QString, bool> expanded_state;
  for (int i = 0; i < topics_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = topics_tree_->topLevelItem(i);
    expanded_state[item->text(0)] = item->isExpanded();
  }
  
  // Clear the tree
  topics_tree_->clear();
  
  // Organize topics by namespace
  std::map<std::string, std::vector<std::pair<std::string, std::vector<std::string>>>> topics_by_namespace;
  
  for (const auto& topic_pair : topics_info) {
    const auto& topic_name = topic_pair.first;
    const auto& topic_info = topic_pair.second;
    
    // Extract namespace from topic name
    std::string ns = "/";
    size_t last_slash = topic_name.find_last_of("/");
    if (last_slash != std::string::npos && last_slash > 0) {
      ns = topic_name.substr(0, last_slash);
    }
    
    // Extract the short name (without namespace)
    std::string short_name = topic_name;
    if (last_slash != std::string::npos) {
      short_name = topic_name.substr(last_slash + 1);
    }
    
    topics_by_namespace[ns].push_back({short_name, topic_info.topic_types});
  }
  
  // Add topics to tree
  for (const auto& ns_pair : topics_by_namespace) {
    QTreeWidgetItem* ns_item = new QTreeWidgetItem(topics_tree_);
    ns_item->setText(0, QString::fromStdString(ns_pair.first));
    ns_item->setText(1, "Namespace");
    
    for (const auto& topic_info : ns_pair.second) {
      QTreeWidgetItem* topic_item = new QTreeWidgetItem(ns_item);
      topic_item->setText(0, QString::fromStdString(topic_info.first));
      
      // Use first type as display type
      std::string display_type = "Unknown";
      if (!topic_info.second.empty()) {
        display_type = topic_info.second[0];
        // Strip off namespaces for display clarity
        size_t last_slash = display_type.find_last_of("/");
        if (last_slash != std::string::npos) {
          display_type = display_type.substr(last_slash + 1);
        }
      }
      topic_item->setText(1, QString::fromStdString(display_type));
      
      // Store full topic name as data
      std::string full_name = ns_pair.first + "/" + topic_info.first;
      if (ns_pair.first == "/" && !topic_info.first.empty() && topic_info.first[0] == '/') {
        full_name = topic_info.first;  // Already has leading slash
      } else if (ns_pair.first == "/") {
        full_name = "/" + topic_info.first;
      }
      topic_item->setData(0, Qt::UserRole, QString::fromStdString(full_name));
    }
    
    // Restore expanded state
    if (expanded_state.contains(QString::fromStdString(ns_pair.first))) {
      ns_item->setExpanded(expanded_state[QString::fromStdString(ns_pair.first)]);
    }
  }
  
  // Update details if a topic is selected
  if (!selected_topic_.isEmpty()) {
    handleTopicSelectionChanged();
  }
}

void MonitorGUI::handleTopicSelectionChanged()
{
  QList<QTreeWidgetItem*> selected_items = topics_tree_->selectedItems();
  if (selected_items.isEmpty()) {
    selected_topic_ = "";
    topic_details_table_->setRowCount(0);
    return;
  }
  
  QTreeWidgetItem* selected_item = selected_items.first();
  if (selected_item->parent() != nullptr) {  // Only handle topic items (not namespace items)
    QString full_topic_name = selected_item->data(0, Qt::UserRole).toString();
    if (full_topic_name.isEmpty()) {
      // Construct full name from namespace and topic name
      QString ns = selected_item->parent()->text(0);
      QString topic_name = selected_item->text(0);
      if (ns == "/") {
        full_topic_name = "/" + topic_name;
      } else {
        full_topic_name = ns + "/" + topic_name;
      }
    }
    
    selected_topic_ = full_topic_name;
    
    // Get topic info
    auto topics_info = monitor_node_->getTopicsInfo();
    std::string std_topic_name = full_topic_name.toStdString();
    
    if (topics_info.find(std_topic_name) != topics_info.end()) {
      const auto& topic_info = topics_info[std_topic_name];
      
      // Update details table
      topic_details_table_->setRowCount(0);
      
      // Add topic name
      int row = topic_details_table_->rowCount();
      topic_details_table_->insertRow(row);
      topic_details_table_->setItem(row, 0, new QTableWidgetItem("Topic Name"));
      topic_details_table_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(topic_info.topic_name)));
      
      // Add topic types
      row = topic_details_table_->rowCount();
      topic_details_table_->insertRow(row);
      topic_details_table_->setItem(row, 0, new QTableWidgetItem("Topic Types"));
      QString types = "";
      for (const auto& type : topic_info.topic_types) {
        types += QString::fromStdString(type) + "\n";
      }
      topic_details_table_->setItem(row, 1, new QTableWidgetItem(types));
      
      // Add publishers
      row = topic_details_table_->rowCount();
      topic_details_table_->insertRow(row);
      topic_details_table_->setItem(row, 0, new QTableWidgetItem("Publishers"));
      QString publishers = "";
      for (const auto& publisher : topic_info.publisher_nodes) {
        publishers += QString::fromStdString(publisher) + "\n";
      }
      topic_details_table_->setItem(row, 1, new QTableWidgetItem(publishers));
      
      // Add subscribers
      row = topic_details_table_->rowCount();
      topic_details_table_->insertRow(row);
      topic_details_table_->setItem(row, 0, new QTableWidgetItem("Subscribers"));
      QString subscribers = "";
      for (const auto& subscriber : topic_info.subscriber_nodes) {
        subscribers += QString::fromStdString(subscriber) + "\n";
      }
      topic_details_table_->setItem(row, 1, new QTableWidgetItem(subscribers));
      
      // Add frequency
      row = topic_details_table_->rowCount();
      topic_details_table_->insertRow(row);
      topic_details_table_->setItem(row, 0, new QTableWidgetItem("Message Frequency"));
      topic_details_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 Hz").arg(topic_info.msg_frequency, 0, 'f', 2)));
    }
  }
}

void MonitorGUI::updateDiagnosticsView()
{
  // Get diagnostic information
  auto diagnostics = monitor_node_->getLatestDiagnostics();
  
  // Clear the tree
  diagnostics_tree_->clear();
  
  // Add diagnostics to tree
  for (const auto& status : diagnostics) {
    QTreeWidgetItem* status_item = new QTreeWidgetItem(diagnostics_tree_);
    status_item->setText(0, QString::fromStdString(status.name));
    
    // Set status level and color
    QString status_text;
    QColor status_color;
    switch (status.level) {
      case 0:  // OK
        status_text = "OK";
        status_color = QColor(0, 170, 0);  // Green
        break;
      case 1:  // WARN
        status_text = "WARNING";
        status_color = QColor(255, 170, 0);  // Yellow/Orange
        break;
      case 2:  // ERROR
        status_text = "ERROR";
        status_color = QColor(255, 0, 0);  // Red
        break;
      case 3:  // STALE
        status_text = "STALE";
        status_color = QColor(150, 150, 150);  // Gray
        break;
      default:
        status_text = "UNKNOWN";
        status_color = QColor(100, 100, 100);  // Dark Gray
    }
    
    status_item->setText(1, status_text);
    status_item->setBackground(1, status_color);
    status_item->setForeground(1, QColor(255, 255, 255));  // White text
    
    status_item->setText(2, QString::fromStdString(status.message));
    
    // Add values as children
    for (const auto& kv : status.values) {
      QTreeWidgetItem* value_item = new QTreeWidgetItem(status_item);
      value_item->setText(0, QString::fromStdString(kv.key));
      value_item->setText(2, QString::fromStdString(kv.value));
    }
  }
  
  // Expand all items
  diagnostics_tree_->expandAll();
}

void MonitorGUI::updateSensorDataView()
{
  // Get latest sensor data
  auto sensor_data = monitor_node_->getLatestSensorData();
  if (sensor_data == nullptr) {
    return;
  }
  
  // Update IMU table
  imu_table_->setRowCount(0);
  
  // Linear acceleration
  int row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Linear Acceleration X"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 m/s²").arg(sensor_data->imu_data.linear_acceleration.x, 0, 'f', 3)));
  
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Linear Acceleration Y"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 m/s²").arg(sensor_data->imu_data.linear_acceleration.y, 0, 'f', 3)));
  
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Linear Acceleration Z"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 m/s²").arg(sensor_data->imu_data.linear_acceleration.z, 0, 'f', 3)));
  
  // Angular velocity
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Angular Velocity X"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 rad/s").arg(sensor_data->imu_data.angular_velocity.x, 0, 'f', 3)));
  
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Angular Velocity Y"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 rad/s").arg(sensor_data->imu_data.angular_velocity.y, 0, 'f', 3)));
  
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Angular Velocity Z"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 rad/s").arg(sensor_data->imu_data.angular_velocity.z, 0, 'f', 3)));
  
  // Orientation (quaternion)
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Orientation X"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1").arg(sensor_data->imu_data.orientation.x, 0, 'f', 3)));
  
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Orientation Y"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1").arg(sensor_data->imu_data.orientation.y, 0, 'f', 3)));
  
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Orientation Z"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1").arg(sensor_data->imu_data.orientation.z, 0, 'f', 3)));
  
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Orientation W"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1").arg(sensor_data->imu_data.orientation.w, 0, 'f', 3)));
  
  // Time offset
  row = imu_table_->rowCount();
  imu_table_->insertRow(row);
  imu_table_->setItem(row, 0, new QTableWidgetItem("Time Offset"));
  imu_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 s").arg(sensor_data->time_offset, 0, 'f', 6)));
  
  // Update Joints table
  joints_table_->setRowCount(0);
  
  // Populate joints data
  for (size_t i = 0; i < sensor_data->joint_states.name.size() && 
                     i < sensor_data->joint_states.position.size() && 
                     i < sensor_data->joint_states.velocity.size(); ++i) {
    row = joints_table_->rowCount();
    joints_table_->insertRow(row);
    
    joints_table_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(sensor_data->joint_states.name[i])));
    joints_table_->setItem(row, 1, new QTableWidgetItem(QString("%1 rad").arg(sensor_data->joint_states.position[i], 0, 'f', 3)));
    joints_table_->setItem(row, 2, new QTableWidgetItem(QString("%1 rad/s").arg(sensor_data->joint_states.velocity[i], 0, 'f', 3)));
  }
}

}  // namespace dodo_monitor