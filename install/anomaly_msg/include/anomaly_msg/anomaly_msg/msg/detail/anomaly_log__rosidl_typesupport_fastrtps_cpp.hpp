// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

#ifndef ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "anomaly_msg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "anomaly_msg/msg/detail/anomaly_log__struct.hpp"

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

namespace anomaly_msg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
cdr_serialize(
  const anomaly_msg::msg::AnomalyLog & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  anomaly_msg::msg::AnomalyLog & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
get_serialized_size(
  const anomaly_msg::msg::AnomalyLog & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
max_serialized_size_AnomalyLog(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
cdr_serialize_key(
  const anomaly_msg::msg::AnomalyLog & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
get_serialized_size_key(
  const anomaly_msg::msg::AnomalyLog & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
max_serialized_size_key_AnomalyLog(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace anomaly_msg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_anomaly_msg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, anomaly_msg, msg, AnomalyLog)();

#ifdef __cplusplus
}
#endif

#endif  // ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
