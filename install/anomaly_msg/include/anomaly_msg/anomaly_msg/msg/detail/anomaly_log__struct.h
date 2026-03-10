// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "anomaly_msg/msg/anomaly_log.h"


#ifndef ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__STRUCT_H_
#define ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'node_name'
// Member 'source'
// Member 'description'
// Member 'topic_name'
// Member 'data_type'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AnomalyLog in the package anomaly_msg.
/**
  * Anomaly_log.msg
 */
typedef struct anomaly_msg__msg__AnomalyLog
{
  builtin_interfaces__msg__Time stamp;
  /// name of node publishing
  rosidl_runtime_c__String node_name;
  /// camera, lidar, mic, etc.
  rosidl_runtime_c__String source;
  /// relevant info, e.g. front camera
  rosidl_runtime_c__String description;
  /// original topic name
  rosidl_runtime_c__String topic_name;
  /// image, float32, pointcloud, etc.
  rosidl_runtime_c__String data_type;
  /// relevant data
  rosidl_runtime_c__uint8__Sequence data;
} anomaly_msg__msg__AnomalyLog;

// Struct for a sequence of anomaly_msg__msg__AnomalyLog.
typedef struct anomaly_msg__msg__AnomalyLog__Sequence
{
  anomaly_msg__msg__AnomalyLog * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} anomaly_msg__msg__AnomalyLog__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__STRUCT_H_
