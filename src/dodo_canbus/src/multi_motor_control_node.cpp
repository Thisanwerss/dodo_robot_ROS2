#include "dodo_canbus/multi_motor_control_node.hpp"
using namespace std::chrono_literals;

// ============================= CanGroup Implementation =============================

CanGroup::CanGroup(const std::string &ifname,
                   const std::vector<int> &motor_ids,
                   const std::vector<std::string> &motor_types)
{
  can_port_ = std::make_shared<CanPort>(ifname);
  controller_ = std::make_shared<damiao::Motor_Control>(can_port_);

  for (size_t i = 0; i < motor_ids.size(); ++i)
  {
    damiao::DM_Motor_Type type = damiao::DM4310;
    if (i < motor_types.size())
      type = parse_motor_type(motor_types[i]);
    uint8_t slave_id  = motor_ids[i];
    uint8_t master_id = static_cast<uint8_t>(slave_id + 0x10);
    //这里的motor_id就是当作can_id处理，创建对象的参数顺序为电机类型，can_id, master_id: 暂定规则master_id=can_id+0x10
    auto m = std::make_shared<damiao::Motor>(type, slave_id, master_id);
    controller_->addMotor(m.get());
    controller_->enable(*m);
    motors_.push_back(m);

    RCLCPP_INFO(rclcpp::get_logger("CanGroup"),
                "Added motor ID %d, Type %s", motor_ids[i], motor_types[i].c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("CanGroup"),
              "✅ %s initialized with %zu motors", ifname.c_str(), motors_.size());
}

damiao::DM_Motor_Type CanGroup::parse_motor_type(const std::string &type_str)
{
  using namespace damiao;
  if (type_str == "DM4310") return DM4310;
  if (type_str == "DM4310_48V") return DM4310_48V;
  if (type_str == "DM4340") return DM4340;
  if (type_str == "DM4340_48V") return DM4340_48V;
  if (type_str == "DM6006") return DM6006;
  if (type_str == "DM8006") return DM8006;
  if (type_str == "DM8009") return DM8009;
  if (type_str == "DM10010L") return DM10010L;
  if (type_str == "DM10010") return DM10010;
  if (type_str == "DMH3510") return DMH3510;
  if (type_str == "DMH6215") return DMH6215;
  if (type_str == "DMG6220") return DMG6220;
  RCLCPP_WARN(rclcpp::get_logger("CanGroup"),
              "Unknown motor type '%s', defaulting to DM4310", type_str.c_str());
  return DM4310;
}


void CanGroup::send_commands(const std::vector<std::array<float, 5>> &cmds)
{
  for (size_t i = 0; i < motors_.size() && i < cmds.size(); ++i)
  {
    try
    {
      //第一个参数电机对象，第二个是kp，第三个是kd，第四个是位置，第五个是速度，第六个是扭矩。
      controller_->control_mit(*motors_[i],
                               cmds[i][0], cmds[i][1], cmds[i][2], cmds[i][3], cmds[i][4]);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanGroup"),
                   "Motor %u control error: %s",
                   motors_[i]->GetSlaveId(), e.what());
    }
  }
}

void CanGroup::update_states()
{
  for (auto &m : motors_)
    controller_->refresh_motor_status(*m);
}

std::vector<std::shared_ptr<damiao::Motor>> CanGroup::get_motors() const
{
  return motors_;
}

// ============================= MotorControlNode Implementation =============================

MotorControlNode::MotorControlNode()
    : Node("multi_motor_control_node")
{
  // ===== Declare parameters =====
  this->declare_parameter<std::vector<std::string>>("can_interfaces", {"can0"});
  this->declare_parameter<std::vector<std::string>>("serial_ports", {"/dev/ttyAM0"});
  this->declare_parameter<std::vector<int64_t>>("motor_ids_can0", {1, 2, 3, 4});
  this->declare_parameter<std::vector<int64_t>>("motor_ids_serial0", {5, 6, 7, 8});
  this->declare_parameter<std::vector<std::string>>("motor_types_can0",
      {"DM4310", "DM4310", "DM4310", "DMH6215"});
  this->declare_parameter<std::vector<std::string>>("motor_ids_serial0",
      {"DM4310", "DM4310", "DM4310", "DMH6215"});
  this->declare_parameter<int>("update_rate", 100);

  // ===== Get parameters =====
  auto can_interfaces = this->get_parameter("can_interfaces").as_string_array();
  auto ids_can0 = this->get_parameter("motor_ids_can0").as_integer_array();
  auto ids_serial0 = this->get_parameter("motor_ids_serial0").as_integer_array();
  auto types_can0 = this->get_parameter("motor_types_can0").as_string_array();
  auto types_serial0 = this->get_parameter("motor_types_serial0").as_string_array();
  update_rate_ = this->get_parameter("update_rate").as_int();

  std::vector<int> can0_ids(ids_can0.begin(), ids_can0.end());
  std::vector<int> serial0_ids(ids_serial0.begin(), ids_serial0.end());

  // ===== Initialize CAN groups =====
  if (can_interfaces.size() > 0)
    can_groups_.push_back(std::make_shared<CanGroup>(can_interfaces[0], can0_ids, types_can0));
  // ===== Initialize SERIAL groups =====
  if (serial_ports.size() > 0)
  serial_groups_.push_back(std::make_shared<SerialGroup>(serial_ports[0], serial0_ids, types_serial0));
  // ===== ROS interfaces =====
  sub_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_cmd", 10, std::bind(&MotorControlNode::cmd_callback, this, _1));
  /* Beispiel 左腿 4 个电机
  10.0, 0.5, 0.0, 0.0, 0.0,      # 电机0
  10.0, 0.5, 0.1, 0.0, 0.0,      # 电机1
  10.0, 0.5, 0.2, 0.0, 0.0,      # 电机2
  10.0, 0.5, 0.3, 0.0, 0.0,      # 电机3
  # 右腿 4 个电机
  10.0, 0.5, 0.4, 0.0, 0.0,      # 电机4
  10.0, 0.5, 0.5, 0.0, 0.0,      # 电机5
  10.0, 0.5, 0.6, 0.0, 0.0,      # 电机6
  10.0, 0.5, 0.7, 0.0, 0.0       # 电机7
  */

  pub_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/motor_state", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / update_rate_),
      std::bind(&MotorControlNode::publish_state, this));

  RCLCPP_INFO(this->get_logger(), "✅ Multi-motor control node running at %d Hz", update_rate_);
}

void MotorControlNode::cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  size_t total_motors = 0;
  for (auto &g : can_groups_)
  total_motors += g->get_motors().size();
  for (auto &g : serial_groups_)
  total_motors += g->get_motors().size();
//检查指令数量的长度是否符合需求
  if (msg->data.size() < total_motors * 5)
  {
    RCLCPP_WARN(this->get_logger(),
                "Expected %zu floats (%zu motors × 5 values), got %zu",
                total_motors * 5, total_motors, msg->data.size());
    return;
  }
  
  //填充cmds
  std::vector<std::array<float, 5>> cmds(total_motors);
  for (size_t i = 0; i < total_motors; ++i)
    for (int j = 0; j < 5; ++j)
      cmds[i][j] = msg->data[i * 5 + j];

  // Split commands by CAN group
  size_t offset = 0;
  for (auto &g : can_groups_) {
  auto motors = g->get_motors();
  size_t count = motors.size();
  std::vector<std::array<float, 5>> sub_cmds(cmds.begin() + offset, cmds.begin() + offset + count);
  g->send_commands(sub_cmds);
  offset += count;
}

for (auto &g : serial_groups_) {
  auto motors = g->get_motors();
  size_t count = motors.size();
  std::vector<std::array<float, 5>> sub_cmds(cmds.begin() + offset, cmds.begin() + offset + count);
  g->send_commands(sub_cmds);
  offset += count;
}
  
}

void MotorControlNode::publish_state()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->get_clock()->now();

  for (auto &group : can_groups_)
  {
    group->update_states();
    for (auto &m : group->get_motors())
    {
      msg.name.push_back("motor_" + std::to_string(m->GetSlaveId()));
      msg.position.push_back(m->Get_Position());
      msg.velocity.push_back(m->Get_Velocity());
      msg.effort.push_back(m->Get_tau());
    }
  }

  for (auto &group : serial_groups_) {
  group->update_states();
  for (auto &m : group->get_motors()) {
    msg.name.push_back("motor_" + std::to_string(m->GetSlaveId()));
    msg.position.push_back(m->Get_Position());
    msg.velocity.push_back(m->Get_Velocity());
    msg.effort.push_back(m->Get_tau());
  }
}

  pub_state_->publish(msg);
}

// ============================= Main =============================

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
}
