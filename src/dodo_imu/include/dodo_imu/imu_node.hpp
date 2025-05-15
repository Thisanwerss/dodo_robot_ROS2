#ifndef DODO_IMU_NODE_HPP
#define DODO_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <memory>
#include <cmath>


namespace dodo_imu
{
struct IMUData {
  double timestamp;            // Timestamp in seconds
  double accel_x;              // Acceleration in m/s^2
  double accel_y;
  double accel_z;
  double gyro_x;               // Angular velocity in rad/s
  double gyro_y;
  double gyro_z;
  double quaternion_w;
  double quaternion_x;
  double quaternion_y;
  double quaternion_z;
  std::array<double, 9> cov_accel;  // Covariance matrix for acceleration
  std::array<double, 9> cov_gyro;   // Covariance matrix for angular velocity
};

class IMUNode : public rclcpp::Node
{
public:
  IMUNode();
 

private:
  // Timer callback
  void publishIMUData();
  
  // Convert raw IMU data to ROS message
  sensor_msgs::msg::Imu convertToROSMsg(const IMUData & data);
  
  // Generate dummy IMU data for testing
  void generateDummyIMUData(IMUData & data);
  bool readIMU(IMUData& data);
  bool initIMU();

  // Parameters
  std::string imu_device_;
  uint8_t imu_address_;
  int publish_rate_;
  std::string frame_id_;
  bool dummy_mode_;
  
  int i2c_file_ = -1;

  // IMU driver
 
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dodo_imu

#endif  // DODO_IMU_NODE_HPP