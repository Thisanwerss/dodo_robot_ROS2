#ifndef DODO_IMU_DRIVER_HPP
#define DODO_IMU_DRIVER_HPP

#include <string>
#include <array>

namespace dodo_imu
{

// IMU data structure
struct IMUData {
  double timestamp;            // Timestamp in seconds
  double accel_x;              // Acceleration in m/s^2
  double accel_y;
  double accel_z;
  double gyro_x;               // Angular velocity in rad/s
  double gyro_y;
  double gyro_z;
  std::array<double, 9> cov_accel;  // Covariance matrix for acceleration
  std::array<double, 9> cov_gyro;   // Covariance matrix for angular velocity
};

class IMUDriver
{
public:
  IMUDriver(const std::string & device, uint8_t address);
  virtual ~IMUDriver();

  // Initialize the IMU
  bool init();
  
  // Close the IMU
  void close();
  
  // Read data from the IMU
  bool readData(IMUData & data);
  
  // Configure the IMU
  bool configure();
  
private:
  std::string device_;
  uint8_t address_;
  int fd_;
  bool is_open_;
  
  // Helper functions
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t & value);
  bool readRegisters(uint8_t reg, uint8_t * values, size_t count);
};

}  // namespace dodo_imu

#endif  // DODO_IMU_DRIVER_HPP