// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "anomaly_msg/msg/detail/anomaly_log__rosidl_typesupport_introspection_c.h"
#include "anomaly_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "anomaly_msg/msg/detail/anomaly_log__functions.h"
#include "anomaly_msg/msg/detail/anomaly_log__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `node_name`
// Member `source`
// Member `description`
// Member `topic_name`
// Member `data_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  anomaly_msg__msg__AnomalyLog__init(message_memory);
}

void anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_fini_function(void * message_memory)
{
  anomaly_msg__msg__AnomalyLog__fini(message_memory);
}

size_t anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__size_function__AnomalyLog__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__get_const_function__AnomalyLog__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__get_function__AnomalyLog__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__fetch_function__AnomalyLog__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__get_const_function__AnomalyLog__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__assign_function__AnomalyLog__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__get_function__AnomalyLog__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__resize_function__AnomalyLog__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_member_array[7] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(anomaly_msg__msg__AnomalyLog, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "node_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(anomaly_msg__msg__AnomalyLog, node_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(anomaly_msg__msg__AnomalyLog, source),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(anomaly_msg__msg__AnomalyLog, description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "topic_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(anomaly_msg__msg__AnomalyLog, topic_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(anomaly_msg__msg__AnomalyLog, data_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(anomaly_msg__msg__AnomalyLog, data),  // bytes offset in struct
    NULL,  // default value
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__size_function__AnomalyLog__data,  // size() function pointer
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__get_const_function__AnomalyLog__data,  // get_const(index) function pointer
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__get_function__AnomalyLog__data,  // get(index) function pointer
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__fetch_function__AnomalyLog__data,  // fetch(index, &value) function pointer
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__assign_function__AnomalyLog__data,  // assign(index, value) function pointer
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__resize_function__AnomalyLog__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_members = {
  "anomaly_msg__msg",  // message namespace
  "AnomalyLog",  // message name
  7,  // number of fields
  sizeof(anomaly_msg__msg__AnomalyLog),
  false,  // has_any_key_member_
  anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_member_array,  // message members
  anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_init_function,  // function to initialize message memory (memory has to be allocated)
  anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_type_support_handle = {
  0,
  &anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_members,
  get_message_typesupport_handle_function,
  &anomaly_msg__msg__AnomalyLog__get_type_hash,
  &anomaly_msg__msg__AnomalyLog__get_type_description,
  &anomaly_msg__msg__AnomalyLog__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_anomaly_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, anomaly_msg, msg, AnomalyLog)() {
  anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_type_support_handle.typesupport_identifier) {
    anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &anomaly_msg__msg__AnomalyLog__rosidl_typesupport_introspection_c__AnomalyLog_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
