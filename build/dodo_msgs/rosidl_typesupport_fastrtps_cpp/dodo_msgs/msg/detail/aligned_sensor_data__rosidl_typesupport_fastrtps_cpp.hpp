// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dodo_msgs:msg/AlignedSensorData.idl
// generated code does not contain a copyright notice

#ifndef DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dodo_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dodo_msgs/msg/detail/aligned_sensor_data__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace dodo_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dodo_msgs
cdr_serialize(
  const dodo_msgs::msg::AlignedSensorData & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dodo_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dodo_msgs::msg::AlignedSensorData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dodo_msgs
get_serialized_size(
  const dodo_msgs::msg::AlignedSensorData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dodo_msgs
max_serialized_size_AlignedSensorData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dodo_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dodo_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dodo_msgs, msg, AlignedSensorData)();

#ifdef __cplusplus
}
#endif

#endif  // DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
