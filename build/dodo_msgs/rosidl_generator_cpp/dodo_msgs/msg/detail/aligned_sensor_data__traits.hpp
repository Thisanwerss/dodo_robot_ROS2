// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dodo_msgs:msg/AlignedSensorData.idl
// generated code does not contain a copyright notice

#ifndef DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__TRAITS_HPP_
#define DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dodo_msgs/msg/detail/aligned_sensor_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'imu_data'
#include "sensor_msgs/msg/detail/imu__traits.hpp"
// Member 'joint_states'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"

namespace dodo_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AlignedSensorData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: imu_data
  {
    out << "imu_data: ";
    to_flow_style_yaml(msg.imu_data, out);
    out << ", ";
  }

  // member: joint_states
  {
    out << "joint_states: ";
    to_flow_style_yaml(msg.joint_states, out);
    out << ", ";
  }

  // member: time_offset
  {
    out << "time_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.time_offset, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AlignedSensorData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: imu_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_data:\n";
    to_block_style_yaml(msg.imu_data, out, indentation + 2);
  }

  // member: joint_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_states:\n";
    to_block_style_yaml(msg.joint_states, out, indentation + 2);
  }

  // member: time_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.time_offset, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AlignedSensorData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dodo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use dodo_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dodo_msgs::msg::AlignedSensorData & msg,
  std::ostream & out, size_t indentation = 0)
{
  dodo_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dodo_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dodo_msgs::msg::AlignedSensorData & msg)
{
  return dodo_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dodo_msgs::msg::AlignedSensorData>()
{
  return "dodo_msgs::msg::AlignedSensorData";
}

template<>
inline const char * name<dodo_msgs::msg::AlignedSensorData>()
{
  return "dodo_msgs/msg/AlignedSensorData";
}

template<>
struct has_fixed_size<dodo_msgs::msg::AlignedSensorData>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Imu>::value && has_fixed_size<sensor_msgs::msg::JointState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<dodo_msgs::msg::AlignedSensorData>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Imu>::value && has_bounded_size<sensor_msgs::msg::JointState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<dodo_msgs::msg::AlignedSensorData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__TRAITS_HPP_
