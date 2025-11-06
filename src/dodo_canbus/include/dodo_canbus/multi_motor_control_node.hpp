#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "damiao_can.hpp"
#include "damiao.hpp"

#include <vector>
#include <array>
#include <memory>


// ============================= SerialGroup =============================
class SerialGroup
{
public:
  SerialGroup(const std::string &port_name,
              const std::vector<int> &motor_ids,
              const std::vector<std::string> &motor_types)
  {
    serial_port_ = std::make_shared<SerialPort>(port_name);
    controller_ = std::make_shared<damiao::Motor_Control>(serial_port_,B921600);

    for (size_t i = 0; i < motor_ids.size(); ++i)
    {
      damiao::DM_Motor_Type type = damiao::DM4310;
      if (i < motor_types.size())
        type = parse_motor_type(motor_types[i]);

      auto m = std::make_shared<damiao::Motor>(type, motor_ids[i], 0x00);
      controller_->addMotor(m.get());
      controller_->enable(*m);
      motors_.push_back(m);

      RCLCPP_INFO(rclcpp::get_logger("SerialGroup"),
                  "Added motor ID %d on %s", motor_ids[i], port_name.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("SerialGroup"),
                "✅ %s initialized with %zu motors", port_name.c_str(), motors_.size());
  }

  damiao::DM_Motor_Type parse_motor_type(const std::string &type_str)
  {
    using namespace damiao;
    if (type_str == "DM4310") return DM4310;
    if (type_str == "DM4340") return DM4340;
    if (type_str == "DMH6215") return DMH6215;
    RCLCPP_WARN(rclcpp::get_logger("SerialGroup"),
                "Unknown motor type '%s', defaulting to DM4310", type_str.c_str());
    return DM4310;
  }

  void send_commands(const std::vector<std::array<float, 5>> &cmds)
  {
    for (size_t i = 0; i < motors_.size() && i < cmds.size(); ++i)
    {
      controller_->control_mit(*motors_[i],
                               cmds[i][0], cmds[i][1], cmds[i][2], cmds[i][3], cmds[i][4]);
    }
  }

  void update_states()
  {
    for (auto &m : motors_)
      controller_->refresh_motor_status(*m);
  }

  std::vector<std::shared_ptr<damiao::Motor>> get_motors() const { return motors_; }

private:
  std::shared_ptr<SerialPort> serial_port_;
  std::shared_ptr<damiao::Motor_Control> controller_;
  std::vector<std::shared_ptr<damiao::Motor>> motors_;
};


class CanGroup
{
public:
  /**
   * @param ifname  CAN 接口名 (如 "can0")
   * @param motor_ids 电机 ID 列表
   */
  CanGroup(const std::string &ifname, const std::vector<int> &motor_ids, const std::vector<std::string> &motor_types);

  /// 发送 MIT 控制命令（每个电机 [kp, kd, q, dq, tau]）
  void send_commands(const std::vector<std::array<float, 5>> &cmds);

  /// 更新电机状态（调用 CAN 接口刷新）
  void update_states();

  /// 获取电机对象列表
  std::vector<std::shared_ptr<damiao::Motor>> get_motors() const;

private:
  damiao::DM_Motor_Type parse_motor_type(const std::string &type_str);
  std::shared_ptr<CanPort> can_port_;
  std::shared_ptr<damiao::Motor_Control> controller_;
  std::vector<std::shared_ptr<damiao::Motor>> motors_;
};

/**
 * @brief ROS2 主节点类：支持多 CAN + 多电机（MIT 控制模式）
 */
class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode();

private:
  void cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void publish_state();
  std::vector<std::shared_ptr<SerialGroup>> serial_groups_;
  std::vector<std::shared_ptr<CanGroup>> can_groups_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  int update_rate_;
};

