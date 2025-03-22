// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dodo_msgs:msg/AlignedSensorData.idl
// generated code does not contain a copyright notice

#ifndef DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__STRUCT_H_
#define DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'imu_data'
#include "sensor_msgs/msg/detail/imu__struct.h"
// Member 'joint_states'
#include "sensor_msgs/msg/detail/joint_state__struct.h"

/// Struct defined in msg/AlignedSensorData in the package dodo_msgs.
/**
  * AlignedSensorData.msg
  * This message contains time-aligned data from multiple sensors
 */
typedef struct dodo_msgs__msg__AlignedSensorData
{
  std_msgs__msg__Header header;
  /// IMU data
  sensor_msgs__msg__Imu imu_data;
  /// Motor joint states
  sensor_msgs__msg__JointState joint_states;
  /// Time alignment information
  /// Time offset between sensors in seconds
  double time_offset;
} dodo_msgs__msg__AlignedSensorData;

// Struct for a sequence of dodo_msgs__msg__AlignedSensorData.
typedef struct dodo_msgs__msg__AlignedSensorData__Sequence
{
  dodo_msgs__msg__AlignedSensorData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dodo_msgs__msg__AlignedSensorData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__STRUCT_H_
