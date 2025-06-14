#include "dodo_processing/processing_node.hpp"
#include <nlohmann/json.hpp>
#include <fstream>       
#include <sstream>       

namespace dodo_processing
{

ProcessingNode::ProcessingNode()
: Node("processing_node")
{
  // Declare parameters
  this->declare_parameter("command_rate", 100);
  this->declare_parameter("motion_constraints", "{}");

  // Get parameters
  command_rate_ = this->get_parameter("command_rate").as_int();
  motion_constraints_json_ = this->get_parameter("motion_constraints").as_string();

  // Create subscribers
  
    
  usb_commands_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/usb_commands", 10, std::bind(&ProcessingNode::usbCommandsCallback, this, std::placeholders::_1));
    
 

  // Create publisher
  processed_commands_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/processed_commands", 10);

  // Create timer for processing commands
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / command_rate_),
    std::bind(&ProcessingNode::processCommands, this));

  RCLCPP_INFO(this->get_logger(), "Processing Node initialized with command rate %d Hz", command_rate_);
}

ProcessingNode::~ProcessingNode()
{
  RCLCPP_INFO(this->get_logger(), "Processing Node shutting down");
}


void ProcessingNode::loadTrajectoryFromFile(const std::string& filepath)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  trajectory_.clear();
  current_trajectory_index_ = 0;

  std::ifstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file: %s", filepath.c_str());
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();

  try {
    auto traj_json = nlohmann::json::parse(buffer.str());

    if (!traj_json.is_array()) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory JSON is not an array.");
      return;
    }

    for (const auto& point : traj_json) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();

      if (point.contains("name") && point["name"].is_array()) {
        js.name = point["name"].get<std::vector<std::string>>();
      }

      if (point.contains("position") && point["position"].is_array()) {
        js.position = point["position"].get<std::vector<double>>();
      }

      if (point.contains("velocity") && point["velocity"].is_array()) {
        js.velocity = point["velocity"].get<std::vector<double>>();
      }

      if (point.contains("effort") && point["effort"].is_array()) {
        js.effort = point["effort"].get<std::vector<double>>();
      }

      trajectory_.push_back(js);
    }

    trajectory_playback_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Trajectory loaded with %zu points.", trajectory_.size());

  } catch (const nlohmann::json::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
  }
}




void ProcessingNode::usbCommandsCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(usb_command_mutex_);
  latest_usb_command_ = msg;
   switch (msg->data) {
    case 1:
      loadTrajectoryFromFile("trajectory/forward.json");
      break;
    case 2:
      loadTrajectoryFromFile("trajectory/backward.json");
      break;
    case 3:
      loadTrajectoryFromFile("trajectory/left.json");
      break;
    case 4:
      loadTrajectoryFromFile("trajectory/right.json");
      break;
    default:
      break;
  }
}



void ProcessingNode::processCommands()
{
   {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (trajectory_playback_active_ && current_trajectory_index_ < trajectory_.size()) {
      auto cmd = trajectory_[current_trajectory_index_++];
      cmd.header.stamp = this->now();
      processed_commands_pub_->publish(cmd);
      return;
    } else if (trajectory_playback_active_) {
      trajectory_playback_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Trajectory playback completed.");
    }
  }
  
}





}  // namespace dodo_processing

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_processing::ProcessingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}