#include "dodo_canbus/odrive_can.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace dodo_canbus
{

OdriveCANInterface::OdriveCANInterface(const std::string & can_interface)
: can_interface_(can_interface), socket_fd_(-1), is_open_(false)
{
}

OdriveCANInterface::~OdriveCANInterface()
{
  close();
}

bool OdriveCANInterface::init()
{
  // Create a socket
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "Error creating socket" << std::endl;
    return false;
  }
  
  // Specify the CAN interface
  //通过接口名如can0获取接口索引,这个索引是在内核中唯一的
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Error getting interface index" << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }
  
  // Bind the socket to the CAN interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  
  if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    std::cerr << "Error binding socket to interface" << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }
  
  is_open_ = true;
  return true;
}


void OdriveCANInterface::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  is_open_ = false;
}

bool OdriveCANInterface::sendPositionCommand(int can_id, double position)
{
 if (socket_fd_ < 0) {
        std::cerr << "Socket not initialized!" << std::endl;
        return false;
    }

    struct can_frame frame;
    frame.can_id = can_id;        // already includes the base + command (e.g. motor_id | 0x00C)
    frame.can_dlc = 8;            // ODrive position command is 8 bytes

    // Convert double (position in rad) to float
    float pos_f = static_cast<float>(position);

    // Encode position (float) into first 4 bytes
    std::memcpy(&frame.data[0], &pos_f, 4);

    // Velocity and torque feedforward (set to 0)
    int16_t vel_ff = 0;
    int16_t torque_ff = 0;
    std::memcpy(&frame.data[4], &vel_ff, 2);
    std::memcpy(&frame.data[6], &torque_ff, 2);

    // Send the frame via socketCAN
    ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        std::cerr << "Failed to send CAN frame." << std::endl;
        return false;
    }

  return true;
}

can_frame OdriveCANInterface::buildODriveRequestFrame(int motor_id, uint16_t base_cmd_id) {
    can_frame frame{};
    frame.can_id = (motor_id << 5)| base_cmd_id ;
    frame.can_dlc = 0;  
    return frame;
}

bool OdriveCANInterface::readODriveResponse(int socket_fd, int motor_id, uint16_t base_cmd_id, double& pos_out, double& vel_out) {
    can_frame recv_frame{};

    // 等待接收 CAN 响应帧（阻塞方式）
    ssize_t nbytes = read(socket_fd, &recv_frame, sizeof(recv_frame));
    if (nbytes < 0) {
        perror("Read failed");
        return false;
    }

    // 计算预期的 CAN ID（必须和请求一致）
    uint16_t expected_id = (motor_id << 5)| base_cmd_id;

    // 校验 ID 和数据长度
    if (recv_frame.can_id != expected_id || recv_frame.can_dlc != 8) {
        std::cerr << "Unexpected CAN ID or data length\n";
        return false;
    }

    std::memcpy(&pos_out, &recv_frame.data[0], 4);
    std::memcpy(&vel_out, &recv_frame.data[4], 4);

    return true;
}



bool OdriveCANInterface::readMotorState(int motor_id, MotorState & state)
{ can_frame req_frame = buildODriveRequestFrame(motor_id, 0x009); //request for position and velocity
  ssize_t nbytes = write(socket_fd_, &req_frame, sizeof(req_frame));
  if (nbytes != sizeof(req_frame)) {
    std::cerr << "Failed to send CAN request frame." << std::endl;
    return false;
  }
  // Read response frame
  if (readODriveResponse(socket_fd_, motor_id, 0x009, state.position, state.velocity)) {
    std::cout << "Axis - Position: " << state.position << " turns, Velocity: " << state.velocity<< " turns/sec\n";
    double iq_setpoint;
    if (readODriveResponse(socket_fd_, motor_id, 0x014, iq_setpoint, state.torque)) {
        std::cout << "Axis - Iq Setpoint: " << iq_setpoint << " A, Torque: " << state.torque << " Nm\n";
        return true;
    } else {
        return false;
    }

} else {
    return false;
}

}

bool OdriveCANInterface::readMotorStates(const std::vector<int> & motor_ids, std::map<int, MotorState> & motor_states)
{
  // Read state for each motor
  for (const auto & motor_id : motor_ids) {
    MotorState state;
    if (readMotorState(motor_id, state)) {
      motor_states[motor_id] = state;
    } else {
      return false;
    }
  }
  
  return true;
}

bool OdriveCANInterface::emergencyStop()
{
  // In a real implementation, this would send an emergency stop command
  // to all motors
  
  // For this simplified implementation, just log the command
  std::cout << "Emergency stop all motors" << std::endl;
  
  // Placeholder - always return success
  return true;
}

}  // namespace dodo_canbus