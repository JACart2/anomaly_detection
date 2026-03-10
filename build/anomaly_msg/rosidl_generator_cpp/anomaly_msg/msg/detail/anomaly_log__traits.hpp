// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "anomaly_msg/msg/anomaly_log.hpp"


#ifndef ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__TRAITS_HPP_
#define ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "anomaly_msg/msg/detail/anomaly_log__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace anomaly_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const AnomalyLog & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: node_name
  {
    out << "node_name: ";
    rosidl_generator_traits::value_to_yaml(msg.node_name, out);
    out << ", ";
  }

  // member: source
  {
    out << "source: ";
    rosidl_generator_traits::value_to_yaml(msg.source, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << ", ";
  }

  // member: topic_name
  {
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
    out << ", ";
  }

  // member: data_type
  {
    out << "data_type: ";
    rosidl_generator_traits::value_to_yaml(msg.data_type, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AnomalyLog & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: node_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "node_name: ";
    rosidl_generator_traits::value_to_yaml(msg.node_name, out);
    out << "\n";
  }

  // member: source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source: ";
    rosidl_generator_traits::value_to_yaml(msg.source, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }

  // member: topic_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
    out << "\n";
  }

  // member: data_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_type: ";
    rosidl_generator_traits::value_to_yaml(msg.data_type, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AnomalyLog & msg, bool use_flow_style = false)
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

}  // namespace anomaly_msg

namespace rosidl_generator_traits
{

[[deprecated("use anomaly_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const anomaly_msg::msg::AnomalyLog & msg,
  std::ostream & out, size_t indentation = 0)
{
  anomaly_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use anomaly_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const anomaly_msg::msg::AnomalyLog & msg)
{
  return anomaly_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<anomaly_msg::msg::AnomalyLog>()
{
  return "anomaly_msg::msg::AnomalyLog";
}

template<>
inline const char * name<anomaly_msg::msg::AnomalyLog>()
{
  return "anomaly_msg/msg/AnomalyLog";
}

template<>
struct has_fixed_size<anomaly_msg::msg::AnomalyLog>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<anomaly_msg::msg::AnomalyLog>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<anomaly_msg::msg::AnomalyLog>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__TRAITS_HPP_
