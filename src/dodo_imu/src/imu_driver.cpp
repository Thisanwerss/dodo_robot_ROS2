#include "dodo_imu/imu_driver.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>

namespace dodo_imu
{

// IMU register addresses (example for MPU-6050)
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU6050_REG_CONFIG = 0x1A;
constexpr uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;

IMUDriver::IMUDriver(const std::string & device, uint8_t address)
: device_(device), address_(address), fd_(-1), is_open_(false)
{
}

IMUDriver::~IMUDriver()
{
  close();
}

bool IMUDriver::init()
{
  // Open I2C device
  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "Error opening I2C device: " << device_ << std::endl;
    return false;
  }
  
  // Set I2C slave address
  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    std::cerr << "Error setting I2C slave address: 0x" << std::hex << static_cast<int>(address_) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  is_open_ = true;
  return true;
}

void IMUDriver::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  is_open_ = false;
}

bool IMUDriver::readData(IMUData & data)
{
  if (!is_open_) {
    std::cerr << "IMU not initialized" << std::endl;
    return false;
  }
  
  // In a real implementation, this would read acceleration and gyro data
  // from the IMU registers
  
  // For this simplified implementation, just create dummy values
  data.timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  data.accel_x = 0.0;  // m/s^2
  data.accel_y = 0.0;
  data.accel_z = 9.81;  // 1g downward
  data.gyro_x = 0.0;   // rad/s
  data.gyro_y = 0.0;
  data.gyro_z = 0.0;
  
  // Set covariance matrices (diagonal with small values)
  for (size_t i = 0; i < 9; ++i) {
    data.cov_accel[i] = (i % 4 == 0) ? 0.01 : 0.0;  // Diagonal elements to 0.01
    data.cov_gyro[i] = (i % 4 == 0) ? 0.001 : 0.0;  // Diagonal elements to 0.001
  }
  
  return true;
}

bool IMUDriver::configure()
{
  if (!is_open_) {
    std::cerr << "IMU not initialized" << std::endl;
    return false;
  }
  
  // Wake up the IMU from sleep
  if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  
  // Set sample rate divider
  if (!writeRegister(MPU6050_REG_SMPLRT_DIV, 0x07)) {  // 1kHz / (1 + 7) = 125Hz
    return false;
  }
  
  // Set digital low pass filter
  if (!writeRegister(MPU6050_REG_CONFIG, 0x06)) {  // 5Hz bandwidth
    return false;
  }
  
  // Set gyro range to ±250 deg/s
  if (!writeRegister(MPU6050_REG_GYRO_CONFIG, 0x00)) {
    return false;
  }
  
  // Set accelerometer range to ±2g
  if (!writeRegister(MPU6050_REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }
  
  return true;
}

bool IMUDriver::writeRegister(uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};
  if (write(fd_, buf, 2) != 2) {
    std::cerr << "Error writing to register 0x" << std::hex << static_cast<int>(reg) << std::endl;
    return false;
  }
  return true;
}

bool IMUDriver::readRegister(uint8_t reg, uint8_t & value)
{
  // Write register address
  if (write(fd_, &reg, 1) != 1) {
    std::cerr << "Error writing register address" << std::endl;
    return false;
  }
  
  // Read register value
  if (read(fd_, &value, 1) != 1) {
    std::cerr << "Error reading register value" << std::endl;
    return false;
  }
  
  return true;
}

bool IMUDriver::readRegisters(uint8_t reg, uint8_t * values, size_t count)
{
  // Write register address
  if (write(fd_, &reg, 1) != 1) {
    std::cerr << "Error writing register address" << std::endl;
    return false;
  }
  
  // Read register values
  if (read(fd_, values, count) != static_cast<ssize_t>(count)) {
    std::cerr << "Error reading register values" << std::endl;
    return false;
  }
  
  return true;
}

}  // namespace dodo_imu