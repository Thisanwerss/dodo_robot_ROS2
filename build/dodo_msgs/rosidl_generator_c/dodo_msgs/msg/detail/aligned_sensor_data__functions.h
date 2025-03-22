// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dodo_msgs:msg/AlignedSensorData.idl
// generated code does not contain a copyright notice

#ifndef DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__FUNCTIONS_H_
#define DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dodo_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dodo_msgs/msg/detail/aligned_sensor_data__struct.h"

/// Initialize msg/AlignedSensorData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dodo_msgs__msg__AlignedSensorData
 * )) before or use
 * dodo_msgs__msg__AlignedSensorData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
bool
dodo_msgs__msg__AlignedSensorData__init(dodo_msgs__msg__AlignedSensorData * msg);

/// Finalize msg/AlignedSensorData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
void
dodo_msgs__msg__AlignedSensorData__fini(dodo_msgs__msg__AlignedSensorData * msg);

/// Create msg/AlignedSensorData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dodo_msgs__msg__AlignedSensorData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
dodo_msgs__msg__AlignedSensorData *
dodo_msgs__msg__AlignedSensorData__create();

/// Destroy msg/AlignedSensorData message.
/**
 * It calls
 * dodo_msgs__msg__AlignedSensorData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
void
dodo_msgs__msg__AlignedSensorData__destroy(dodo_msgs__msg__AlignedSensorData * msg);

/// Check for msg/AlignedSensorData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
bool
dodo_msgs__msg__AlignedSensorData__are_equal(const dodo_msgs__msg__AlignedSensorData * lhs, const dodo_msgs__msg__AlignedSensorData * rhs);

/// Copy a msg/AlignedSensorData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
bool
dodo_msgs__msg__AlignedSensorData__copy(
  const dodo_msgs__msg__AlignedSensorData * input,
  dodo_msgs__msg__AlignedSensorData * output);

/// Initialize array of msg/AlignedSensorData messages.
/**
 * It allocates the memory for the number of elements and calls
 * dodo_msgs__msg__AlignedSensorData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
bool
dodo_msgs__msg__AlignedSensorData__Sequence__init(dodo_msgs__msg__AlignedSensorData__Sequence * array, size_t size);

/// Finalize array of msg/AlignedSensorData messages.
/**
 * It calls
 * dodo_msgs__msg__AlignedSensorData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
void
dodo_msgs__msg__AlignedSensorData__Sequence__fini(dodo_msgs__msg__AlignedSensorData__Sequence * array);

/// Create array of msg/AlignedSensorData messages.
/**
 * It allocates the memory for the array and calls
 * dodo_msgs__msg__AlignedSensorData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
dodo_msgs__msg__AlignedSensorData__Sequence *
dodo_msgs__msg__AlignedSensorData__Sequence__create(size_t size);

/// Destroy array of msg/AlignedSensorData messages.
/**
 * It calls
 * dodo_msgs__msg__AlignedSensorData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
void
dodo_msgs__msg__AlignedSensorData__Sequence__destroy(dodo_msgs__msg__AlignedSensorData__Sequence * array);

/// Check for msg/AlignedSensorData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
bool
dodo_msgs__msg__AlignedSensorData__Sequence__are_equal(const dodo_msgs__msg__AlignedSensorData__Sequence * lhs, const dodo_msgs__msg__AlignedSensorData__Sequence * rhs);

/// Copy an array of msg/AlignedSensorData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dodo_msgs
bool
dodo_msgs__msg__AlignedSensorData__Sequence__copy(
  const dodo_msgs__msg__AlignedSensorData__Sequence * input,
  dodo_msgs__msg__AlignedSensorData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DODO_MSGS__MSG__DETAIL__ALIGNED_SENSOR_DATA__FUNCTIONS_H_
