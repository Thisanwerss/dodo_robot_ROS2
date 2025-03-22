// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dodo_msgs:msg/AlignedSensorData.idl
// generated code does not contain a copyright notice

#ifndef DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__BUILDER_HPP_
#define DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dodo_msgs/msg/detail/aligned_sensor_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dodo_msgs
{

namespace msg
{

namespace builder
{

class Init_AlignedSensorData_time_offset
{
public:
  explicit Init_AlignedSensorData_time_offset(::dodo_msgs::msg::AlignedSensorData & msg)
  : msg_(msg)
  {}
  ::dodo_msgs::msg::AlignedSensorData time_offset(::dodo_msgs::msg::AlignedSensorData::_time_offset_type arg)
  {
    msg_.time_offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dodo_msgs::msg::AlignedSensorData msg_;
};

class Init_AlignedSensorData_joint_states
{
public:
  explicit Init_AlignedSensorData_joint_states(::dodo_msgs::msg::AlignedSensorData & msg)
  : msg_(msg)
  {}
  Init_AlignedSensorData_time_offset joint_states(::dodo_msgs::msg::AlignedSensorData::_joint_states_type arg)
  {
    msg_.joint_states = std::move(arg);
    return Init_AlignedSensorData_time_offset(msg_);
  }

private:
  ::dodo_msgs::msg::AlignedSensorData msg_;
};

class Init_AlignedSensorData_imu_data
{
public:
  explicit Init_AlignedSensorData_imu_data(::dodo_msgs::msg::AlignedSensorData & msg)
  : msg_(msg)
  {}
  Init_AlignedSensorData_joint_states imu_data(::dodo_msgs::msg::AlignedSensorData::_imu_data_type arg)
  {
    msg_.imu_data = std::move(arg);
    return Init_AlignedSensorData_joint_states(msg_);
  }

private:
  ::dodo_msgs::msg::AlignedSensorData msg_;
};

class Init_AlignedSensorData_header
{
public:
  Init_AlignedSensorData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AlignedSensorData_imu_data header(::dodo_msgs::msg::AlignedSensorData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AlignedSensorData_imu_data(msg_);
  }

private:
  ::dodo_msgs::msg::AlignedSensorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dodo_msgs::msg::AlignedSensorData>()
{
  return dodo_msgs::msg::builder::Init_AlignedSensorData_header();
}

}  // namespace dodo_msgs

#endif  // DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__BUILDER_HPP_
