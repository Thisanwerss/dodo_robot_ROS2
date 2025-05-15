#include "dodo_imu/imu_node.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <cerrno> 

namespace dodo_imu
{


IMUNode::IMUNode()
: Node("imu_node")
{
  // Declare parameters
  this->declare_parameter("imu_device", "/dev/i2c-7");
  this->declare_parameter("imu_address", 0x55);
  this->declare_parameter("publish_rate", 200);
  this->declare_parameter("frame_id", "imu_link");
  this->declare_parameter("dummy_mode", false);

  // Get parameters
  imu_device_ = this->get_parameter("imu_device").as_string();
  imu_address_ = static_cast<uint8_t>(this->get_parameter("imu_address").as_int());
  publish_rate_ = this->get_parameter("publish_rate").as_int();
  frame_id_ = this->get_parameter("frame_id").as_string();
  dummy_mode_ = this->get_parameter("dummy_mode").as_bool();

  // Initialize IMU driver if not in dummy mode
  if (!dummy_mode_) {

  if (!initIMU()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize I2C IMU.");
        }
  
  } else {
    RCLCPP_INFO(this->get_logger(), "Running in dummy mode - using simulated IMU data");
  }

  // Create publisher
  imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_raw", 10);

  // Create timer for publishing IMU data
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / publish_rate_),
    std::bind(&IMUNode::publishIMUData, this));

  RCLCPP_INFO(this->get_logger(), "IMU Node initialized with publish rate %d Hz", publish_rate_);
}



void IMUNode::publishIMUData()
{
  IMUData data;
  if (dummy_mode_) {
    // Generate dummy IMU data
    IMUData dummy_data;
    generateDummyIMUData(dummy_data);
    
    // Convert to ROS message
    auto imu_msg = convertToROSMsg(dummy_data);
    
    // Publish IMU data
    imu_raw_pub_->publish(imu_msg);
    RCLCPP_DEBUG(this->get_logger(), "Publishing dummy IMU data");
  } else {
     if (!readIMU(data)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data.");
            return;
        }
     else{
      auto msg = convertToROSMsg(data);
      imu_raw_pub_->publish(msg);
     }
  
   
}
}

bool IMUNode::initIMU() {
    i2c_file_ = open(imu_device_.c_str(), O_RDWR);
    if (i2c_file_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "open() failed: %s", strerror(errno));
        return false;
    }

    if (ioctl(i2c_file_, I2C_SLAVE, imu_address_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "ioctl() failed: %s", strerror(errno));
        return false;
    }

    return true;
}


bool IMUNode::readIMU(IMUData& data) {
    uint8_t buf[40]; // 增加缓冲区大小以确保足够读取四元数
    if (read(i2c_file_, buf, 40) != 40) { // 修改读取长度为 40 字节
        RCLCPP_ERROR(this->get_logger(), "Failed to read IMU data: expected 40 bytes.");
        return false;
    }

    // 将缓冲区数据解释为浮点数
    float* imu_floats = reinterpret_cast<float*>(buf);

    // 读取加速度数据
    data.accel_x = imu_floats[0];
    data.accel_y = imu_floats[1];
    data.accel_z = imu_floats[2];

    // 读取陀螺仪数据
    data.gyro_x = imu_floats[3];
    data.gyro_y = imu_floats[4];
    data.gyro_z = imu_floats[5];

    // 读取四元数数据
    data.quaternion_w = imu_floats[6];
    data.quaternion_x = imu_floats[7];
    data.quaternion_y = imu_floats[8];
    data.quaternion_z = imu_floats[9];

    data.timestamp = this->now().seconds();

    data.cov_accel.fill(0.01);
    data.cov_gyro.fill(0.01);

    return true;
}


sensor_msgs::msg::Imu IMUNode::convertToROSMsg(const IMUData & data)
{
  sensor_msgs::msg::Imu imu_msg;
  
  // Set header
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = frame_id_;
  
  // Set linear acceleration
  imu_msg.linear_acceleration.x = data.accel_x;
  imu_msg.linear_acceleration.y = data.accel_y;
  imu_msg.linear_acceleration.z = data.accel_z;
  
  // Set angular velocity
  imu_msg.angular_velocity.x = data.gyro_x;
  imu_msg.angular_velocity.y = data.gyro_y;
  imu_msg.angular_velocity.z = data.gyro_z;
  
  // Set orientation (identity quaternion as placeholder)
  imu_msg.orientation.w = data.quaternion_w;
  imu_msg.orientation.x = data.quaternion_x;
  imu_msg.orientation.y = data.quaternion_y;
  imu_msg.orientation.z = data.quaternion_z;
  
  // Set covariance matrices
  for (size_t i = 0; i < 9; ++i) {
    imu_msg.linear_acceleration_covariance[i] = data.cov_accel[i];
    imu_msg.angular_velocity_covariance[i] = data.cov_gyro[i];
    // Set orientation covariance to unknown
    imu_msg.orientation_covariance[i] = i == 0 ? -1.0 : 0.0;
  }
  
  return imu_msg;
}












void IMUNode::generateDummyIMUData(IMUData & data)
{
  static double time_counter = 0.0;
  time_counter += 1.0 / static_cast<double>(publish_rate_);
  
  // Generate sinusoidal pattern with noise for accelerometer
  data.accel_x = 0.1 * sin(0.5 * time_counter) + 0.05 * sin(5.0 * time_counter);
  data.accel_y = 0.1 * cos(0.5 * time_counter) + 0.05 * sin(7.0 * time_counter);
  data.accel_z = 9.81 + 0.2 * sin(0.2 * time_counter);  // Gravity + small oscillation
  
  // Generate sinusoidal pattern with noise for gyroscope
  data.gyro_x = 0.05 * sin(2.0 * time_counter) + 0.02 * sin(10.0 * time_counter);
  data.gyro_y = 0.05 * cos(2.0 * time_counter) + 0.02 * sin(11.0 * time_counter);
  data.gyro_z = 0.02 * sin(1.0 * time_counter);
  
  // Set identity covariance matrices
  for (size_t i = 0; i < 9; ++i) {
    data.cov_accel[i] = (i % 4 == 0) ? 0.01 : 0.0;  // Diagonal elements
    data.cov_gyro[i] = (i % 4 == 0) ? 0.01 : 0.0;   // Diagonal elements
  }
}

}  // namespace dodo_imu

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodo_imu::IMUNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}