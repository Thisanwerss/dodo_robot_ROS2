// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dodo_msgs:msg/AlignedSensorData.idl
// generated code does not contain a copyright notice

#ifndef DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__STRUCT_HPP_
#define DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'imu_data'
#include "sensor_msgs/msg/detail/imu__struct.hpp"
// Member 'joint_states'
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dodo_msgs__msg__AlignedSensorData __attribute__((deprecated))
#else
# define DEPRECATED__dodo_msgs__msg__AlignedSensorData __declspec(deprecated)
#endif

namespace dodo_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AlignedSensorData_
{
  using Type = AlignedSensorData_<ContainerAllocator>;

  explicit AlignedSensorData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    imu_data(_init),
    joint_states(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_offset = 0.0;
    }
  }

  explicit AlignedSensorData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    imu_data(_alloc, _init),
    joint_states(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_offset = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _imu_data_type =
    sensor_msgs::msg::Imu_<ContainerAllocator>;
  _imu_data_type imu_data;
  using _joint_states_type =
    sensor_msgs::msg::JointState_<ContainerAllocator>;
  _joint_states_type joint_states;
  using _time_offset_type =
    double;
  _time_offset_type time_offset;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__imu_data(
    const sensor_msgs::msg::Imu_<ContainerAllocator> & _arg)
  {
    this->imu_data = _arg;
    return *this;
  }
  Type & set__joint_states(
    const sensor_msgs::msg::JointState_<ContainerAllocator> & _arg)
  {
    this->joint_states = _arg;
    return *this;
  }
  Type & set__time_offset(
    const double & _arg)
  {
    this->time_offset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dodo_msgs::msg::AlignedSensorData_<ContainerAllocator> *;
  using ConstRawPtr =
    const dodo_msgs::msg::AlignedSensorData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dodo_msgs::msg::AlignedSensorData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dodo_msgs::msg::AlignedSensorData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dodo_msgs__msg__AlignedSensorData
    std::shared_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dodo_msgs__msg__AlignedSensorData
    std::shared_ptr<dodo_msgs::msg::AlignedSensorData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignedSensorData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->imu_data != other.imu_data) {
      return false;
    }
    if (this->joint_states != other.joint_states) {
      return false;
    }
    if (this->time_offset != other.time_offset) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignedSensorData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignedSensorData_

// alias to use template instance with default allocator
using AlignedSensorData =
  dodo_msgs::msg::AlignedSensorData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dodo_msgs

#endif  // DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__STRUCT_HPP_
