#ifndef DODO_ODRIVE_CAN_HPP
#define DODO_ODRIVE_CAN_HPP

#include <string>
#include <vector>
#include <map>

namespace dodo_canbus
{

// Motor state structure
struct MotorState {
  double position;        // Current position in radians
  double velocity;        // Current velocity in radians/second
  double torque;          // Current torque in Nm
  bool calibrated;        // Whether the motor is calibrated
  bool error;             // Whether there is an error
  int error_code;         // Error code (if error is true)
};

// PID gains structure
struct PIDGains {
  double kp;  // Proportional gain
  double ki;  // Integral gain
  double kd;  // Derivative gain
};

class OdriveCANInterface
{
public:
  OdriveCANInterface(const std::string & can_interface);
  virtual ~OdriveCANInterface();

  // Initialize the CAN interface
  bool init();
  
  // Close the CAN interface
  void close();
  
  // Send a position command to a motor
  bool sendPositionCommand(int motor_id, double position);

  // Send a velocity command to a motor
  bool sendVelocityCommand(int motor_id, double velocity);
  
  // Send PID gains to a motor
  bool setPIDGains(int motor_id, const PIDGains & gains);
  
  // Read motor state from CAN
  bool readMotorState(int motor_id, MotorState & state);
  
  // Read multiple motor states
  bool readMotorStates(const std::vector<int> & motor_ids, std::map<int, MotorState> & motor_states);
  
  // Emergency stop all motors
  bool emergencyStop();
  
private:
  std::string can_interface_;
  int socket_fd_;
  bool is_open_;
};

}  // namespace dodo_canbus

#endif  // DODO_ODRIVE_CAN_HPP