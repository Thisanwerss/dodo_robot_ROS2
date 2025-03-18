#ifndef DODO_USB_COMMAND_NODE_HPP
#define DODO_USB_COMMAND_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>
#include <fstream>

namespace dodo_usb_command
{

// Command definitions
enum Command {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
};

class USBCommandNode : public rclcpp::Node
{
public:
  USBCommandNode();
  virtual ~USBCommandNode();

private:
  // Timer callback
  void readUSBDevice();
  
  // Process USB input and convert to command
  Command processJoystickInput(const std::vector<int> & raw_data);
  
  // Read raw data from USB device
  bool readRawData(std::vector<int> & raw_data);

  // Parameters
  std::string usb_device_path_;
  int publish_rate_;
  
  // USB device file stream
  std::ifstream usb_device_;
  
  // Publisher for USB commands
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr usb_commands_pub_;
  
  // Timer for reading USB device
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dodo_usb_command

#endif  // DODO_USB_COMMAND_NODE_HPP