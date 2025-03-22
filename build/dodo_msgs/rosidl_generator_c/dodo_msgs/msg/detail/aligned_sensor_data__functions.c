// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dodo_msgs:msg/AlignedSensorData.idl
// generated code does not contain a copyright notice
#include "dodo_msgs/msg/detail/aligned_sensor_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `imu_data`
#include "sensor_msgs/msg/detail/imu__functions.h"
// Member `joint_states`
#include "sensor_msgs/msg/detail/joint_state__functions.h"

bool
dodo_msgs__msg__AlignedSensorData__init(dodo_msgs__msg__AlignedSensorData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    dodo_msgs__msg__AlignedSensorData__fini(msg);
    return false;
  }
  // imu_data
  if (!sensor_msgs__msg__Imu__init(&msg->imu_data)) {
    dodo_msgs__msg__AlignedSensorData__fini(msg);
    return false;
  }
  // joint_states
  if (!sensor_msgs__msg__JointState__init(&msg->joint_states)) {
    dodo_msgs__msg__AlignedSensorData__fini(msg);
    return false;
  }
  // time_offset
  return true;
}

void
dodo_msgs__msg__AlignedSensorData__fini(dodo_msgs__msg__AlignedSensorData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // imu_data
  sensor_msgs__msg__Imu__fini(&msg->imu_data);
  // joint_states
  sensor_msgs__msg__JointState__fini(&msg->joint_states);
  // time_offset
}

bool
dodo_msgs__msg__AlignedSensorData__are_equal(const dodo_msgs__msg__AlignedSensorData * lhs, const dodo_msgs__msg__AlignedSensorData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // imu_data
  if (!sensor_msgs__msg__Imu__are_equal(
      &(lhs->imu_data), &(rhs->imu_data)))
  {
    return false;
  }
  // joint_states
  if (!sensor_msgs__msg__JointState__are_equal(
      &(lhs->joint_states), &(rhs->joint_states)))
  {
    return false;
  }
  // time_offset
  if (lhs->time_offset != rhs->time_offset) {
    return false;
  }
  return true;
}

bool
dodo_msgs__msg__AlignedSensorData__copy(
  const dodo_msgs__msg__AlignedSensorData * input,
  dodo_msgs__msg__AlignedSensorData * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // imu_data
  if (!sensor_msgs__msg__Imu__copy(
      &(input->imu_data), &(output->imu_data)))
  {
    return false;
  }
  // joint_states
  if (!sensor_msgs__msg__JointState__copy(
      &(input->joint_states), &(output->joint_states)))
  {
    return false;
  }
  // time_offset
  output->time_offset = input->time_offset;
  return true;
}

dodo_msgs__msg__AlignedSensorData *
dodo_msgs__msg__AlignedSensorData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dodo_msgs__msg__AlignedSensorData * msg = (dodo_msgs__msg__AlignedSensorData *)allocator.allocate(sizeof(dodo_msgs__msg__AlignedSensorData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dodo_msgs__msg__AlignedSensorData));
  bool success = dodo_msgs__msg__AlignedSensorData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dodo_msgs__msg__AlignedSensorData__destroy(dodo_msgs__msg__AlignedSensorData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dodo_msgs__msg__AlignedSensorData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dodo_msgs__msg__AlignedSensorData__Sequence__init(dodo_msgs__msg__AlignedSensorData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dodo_msgs__msg__AlignedSensorData * data = NULL;

  if (size) {
    data = (dodo_msgs__msg__AlignedSensorData *)allocator.zero_allocate(size, sizeof(dodo_msgs__msg__AlignedSensorData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dodo_msgs__msg__AlignedSensorData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dodo_msgs__msg__AlignedSensorData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dodo_msgs__msg__AlignedSensorData__Sequence__fini(dodo_msgs__msg__AlignedSensorData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dodo_msgs__msg__AlignedSensorData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dodo_msgs__msg__AlignedSensorData__Sequence *
dodo_msgs__msg__AlignedSensorData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dodo_msgs__msg__AlignedSensorData__Sequence * array = (dodo_msgs__msg__AlignedSensorData__Sequence *)allocator.allocate(sizeof(dodo_msgs__msg__AlignedSensorData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dodo_msgs__msg__AlignedSensorData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dodo_msgs__msg__AlignedSensorData__Sequence__destroy(dodo_msgs__msg__AlignedSensorData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dodo_msgs__msg__AlignedSensorData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dodo_msgs__msg__AlignedSensorData__Sequence__are_equal(const dodo_msgs__msg__AlignedSensorData__Sequence * lhs, const dodo_msgs__msg__AlignedSensorData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dodo_msgs__msg__AlignedSensorData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dodo_msgs__msg__AlignedSensorData__Sequence__copy(
  const dodo_msgs__msg__AlignedSensorData__Sequence * input,
  dodo_msgs__msg__AlignedSensorData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dodo_msgs__msg__AlignedSensorData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dodo_msgs__msg__AlignedSensorData * data =
      (dodo_msgs__msg__AlignedSensorData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dodo_msgs__msg__AlignedSensorData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dodo_msgs__msg__AlignedSensorData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dodo_msgs__msg__AlignedSensorData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
