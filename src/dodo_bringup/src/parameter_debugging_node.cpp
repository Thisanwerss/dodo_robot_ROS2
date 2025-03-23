#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <string>

class ParameterDebuggingNode : public rclcpp::Node
{
public:
  ParameterDebuggingNode() : Node("parameter_debugging_node")
  {
    // Declare parameters
    this->declare_parameter("debug_mode", false);

    // Create a timer to print parameters from other nodes
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ParameterDebuggingNode::printParameters, this));

    RCLCPP_INFO(this->get_logger(), "Parameter Debugging Node initialized");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  
  void printParameters()
  {
    // One-time execution
    timer_->cancel();
    
    // Print parameters for the canbus_node
    try {
      auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "canbus_node");
      
      if (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "canbus_node parameter service not available");
        return;
      }
      
      auto parameters = parameters_client->get_parameters({"pid_gains"});
      
      if (parameters.size() > 0) {
        auto param_type = parameters[0].get_type();
        RCLCPP_INFO(this->get_logger(), "pid_gains parameter type: %d", static_cast<int>(param_type));
        
        if (param_type == rclcpp::ParameterType::PARAMETER_STRING) {
          RCLCPP_INFO(this->get_logger(), "pid_gains value: %s", parameters[0].value_to_string().c_str());
          
          // Try to parse the JSON
          try {
            auto j = nlohmann::json::parse(parameters[0].as_string());
            RCLCPP_INFO(this->get_logger(), "JSON parsing successful");
          } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
          }
        } else if (param_type == rclcpp::ParameterType::PARAMETER_NOT_SET) {
          RCLCPP_ERROR(this->get_logger(), "pid_gains parameter not set");
        } else {
          RCLCPP_ERROR(this->get_logger(), "pid_gains has unexpected type");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_gains parameter");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while getting parameters: %s", e.what());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParameterDebuggingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}