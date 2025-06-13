#include "dodo_usb_command/usb_command_node.hpp"
#include <fcntl.h>

namespace dodo_usb_command
{

USBCommandNode::USBCommandNode()
: Node("usb_command_node")
{
  // Declare parameters
  this->declare_parameter("usb_device", "/dev/input/js0");
  this->declare_parameter("publish_rate", 20);

  // Get parameters
  usb_device_path_ = this->get_parameter("usb_device").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_int();

  // Create publisher
  usb_commands_pub_ = this->create_publisher<std_msgs::msg::Int32>("/usb_commands", 10);

  // Open USB device
  usb_device_.open(usb_device_path_, std::ios::binary);
  if (!usb_device_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open USB device: %s", usb_device_path_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Opened USB device: %s", usb_device_path_.c_str());
  }

  // Create timer for reading USB device
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / publish_rate_),
    std::bind(&USBCommandNode::readUSBDevice, this));

  RCLCPP_INFO(this->get_logger(), "USB Command Node initialized with publish rate %d Hz", publish_rate_);
}


USBCommandNode::~USBCommandNode()
{
  if (usb_device_.is_open()) {
    usb_device_.close();
  }
  RCLCPP_INFO(this->get_logger(), "USB Command Node shutting down");
}

void USBCommandNode::readUSBDevice()
{
  if (!usb_device_.is_open()) {
    // Try to reopen the device
    usb_device_.open(usb_device_path_, std::ios::binary);
    if (!usb_device_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open USB device: %s", usb_device_path_.c_str());
      return;
    }
  }

  // Read raw data from USB device
  std::vector<int> raw_data;
  if (!readRawData(raw_data)) {
    return;
  }
  
  // Process raw data to get command
  Command cmd = processJoystickInput(raw_data);

  // Create and publish message
  auto msg = std::make_unique<std_msgs::msg::Int32>();
  msg->data = static_cast<int>(cmd);
  usb_commands_pub_->publish(std::move(msg));
}

bool USBCommandNode::readRawData(std::vector<int> & raw_data)
{
   int fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    perror("open js0 failed");
    return false;
  }

  struct js_event e;
  raw_data.clear();
  int max_events = 1;
  int count = 0;

  while (count < max_events) {
    ssize_t bytes = read(fd, &e, sizeof(e));
    if (bytes == sizeof(e)) {
      if (e.type & JS_EVENT_INIT) {
        continue;
      }

      raw_data.push_back(e.type);
      raw_data.push_back(e.number);
      raw_data.push_back(e.value);
      count++;
    } else {
      usleep(1000);
    }
  }

  close(fd);
  return true;


}

Command USBCommandNode::processJoystickInput(const std::vector<int> & raw_data)
{
  for (size_t i = 0; i + 2 < raw_data.size(); i += 3) {
    int raw_type = raw_data[i];
    int number = raw_data[i + 1];
    int value = raw_data[i + 2];

    int type = raw_type & ~JS_EVENT_INIT;  

    if (type == JS_EVENT_BUTTON) {
      if (number == 0 && value == 1) {
        return Command::STOP;  
      }
    }

    if (type == JS_EVENT_AXIS) {
      if (number == 6) { 
        if (value < -10000) return Command::LEFT;
        if (value >  10000) return Command::RIGHT;
      }
      if (number == 7) { 
        if (value < -10000) return Command::FORWARD;
        if (value >  10000) return Command::BACKWARD;
      }
    }
  }

  return Command::STOP; 

}

}  // namespace dodo_usb_command

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_usb_command::USBCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}