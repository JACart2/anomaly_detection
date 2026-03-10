// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "anomaly_msg/msg/anomaly_log.h"


#ifndef ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__FUNCTIONS_H_
#define ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "anomaly_msg/msg/rosidl_generator_c__visibility_control.h"

#include "anomaly_msg/msg/detail/anomaly_log__struct.h"

/// Initialize msg/AnomalyLog message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * anomaly_msg__msg__AnomalyLog
 * )) before or use
 * anomaly_msg__msg__AnomalyLog__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
bool
anomaly_msg__msg__AnomalyLog__init(anomaly_msg__msg__AnomalyLog * msg);

/// Finalize msg/AnomalyLog message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
void
anomaly_msg__msg__AnomalyLog__fini(anomaly_msg__msg__AnomalyLog * msg);

/// Create msg/AnomalyLog message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * anomaly_msg__msg__AnomalyLog__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
anomaly_msg__msg__AnomalyLog *
anomaly_msg__msg__AnomalyLog__create(void);

/// Destroy msg/AnomalyLog message.
/**
 * It calls
 * anomaly_msg__msg__AnomalyLog__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
void
anomaly_msg__msg__AnomalyLog__destroy(anomaly_msg__msg__AnomalyLog * msg);

/// Check for msg/AnomalyLog message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
bool
anomaly_msg__msg__AnomalyLog__are_equal(const anomaly_msg__msg__AnomalyLog * lhs, const anomaly_msg__msg__AnomalyLog * rhs);

/// Copy a msg/AnomalyLog message.
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
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
bool
anomaly_msg__msg__AnomalyLog__copy(
  const anomaly_msg__msg__AnomalyLog * input,
  anomaly_msg__msg__AnomalyLog * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
const rosidl_type_hash_t *
anomaly_msg__msg__AnomalyLog__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
const rosidl_runtime_c__type_description__TypeDescription *
anomaly_msg__msg__AnomalyLog__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
const rosidl_runtime_c__type_description__TypeSource *
anomaly_msg__msg__AnomalyLog__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
const rosidl_runtime_c__type_description__TypeSource__Sequence *
anomaly_msg__msg__AnomalyLog__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/AnomalyLog messages.
/**
 * It allocates the memory for the number of elements and calls
 * anomaly_msg__msg__AnomalyLog__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
bool
anomaly_msg__msg__AnomalyLog__Sequence__init(anomaly_msg__msg__AnomalyLog__Sequence * array, size_t size);

/// Finalize array of msg/AnomalyLog messages.
/**
 * It calls
 * anomaly_msg__msg__AnomalyLog__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
void
anomaly_msg__msg__AnomalyLog__Sequence__fini(anomaly_msg__msg__AnomalyLog__Sequence * array);

/// Create array of msg/AnomalyLog messages.
/**
 * It allocates the memory for the array and calls
 * anomaly_msg__msg__AnomalyLog__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
anomaly_msg__msg__AnomalyLog__Sequence *
anomaly_msg__msg__AnomalyLog__Sequence__create(size_t size);

/// Destroy array of msg/AnomalyLog messages.
/**
 * It calls
 * anomaly_msg__msg__AnomalyLog__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
void
anomaly_msg__msg__AnomalyLog__Sequence__destroy(anomaly_msg__msg__AnomalyLog__Sequence * array);

/// Check for msg/AnomalyLog message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
bool
anomaly_msg__msg__AnomalyLog__Sequence__are_equal(const anomaly_msg__msg__AnomalyLog__Sequence * lhs, const anomaly_msg__msg__AnomalyLog__Sequence * rhs);

/// Copy an array of msg/AnomalyLog messages.
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
ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
bool
anomaly_msg__msg__AnomalyLog__Sequence__copy(
  const anomaly_msg__msg__AnomalyLog__Sequence * input,
  anomaly_msg__msg__AnomalyLog__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__FUNCTIONS_H_
