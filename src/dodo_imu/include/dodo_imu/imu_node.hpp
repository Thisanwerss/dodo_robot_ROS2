#ifndef DODO_IMU_NODE_HPP
#define DODO_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <memory>
#include "dodo_imu/imu_driver.hpp"

namespace dodo_imu
{

class IMUNode : public rclcpp::Node
{
public:
  IMUNode();
  virtual ~IMUNode();

private:
  // Timer callback
  void publishIMUData();
  
  // Convert raw IMU data to ROS message
  sensor_msgs::msg::Imu convertToROSMsg(const IMUData & data);

  // Parameters
  std::string imu_device_;
  uint8_t imu_address_;
  int publish_rate_;
  std::string frame_id_;
  
  // IMU driver
  std::unique_ptr<IMUDriver> imu_driver_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dodo_imu

#endif  // DODO_IMU_NODE_HPP