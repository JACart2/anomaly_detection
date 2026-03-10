// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "anomaly_msg/msg/anomaly_log.hpp"


#ifndef ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__STRUCT_HPP_
#define ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__anomaly_msg__msg__AnomalyLog __attribute__((deprecated))
#else
# define DEPRECATED__anomaly_msg__msg__AnomalyLog __declspec(deprecated)
#endif

namespace anomaly_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AnomalyLog_
{
  using Type = AnomalyLog_<ContainerAllocator>;

  explicit AnomalyLog_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_name = "";
      this->source = "";
      this->description = "";
      this->topic_name = "";
      this->data_type = "";
    }
  }

  explicit AnomalyLog_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    node_name(_alloc),
    source(_alloc),
    description(_alloc),
    topic_name(_alloc),
    data_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_name = "";
      this->source = "";
      this->description = "";
      this->topic_name = "";
      this->data_type = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _node_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _node_name_type node_name;
  using _source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_type source;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;
  using _topic_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_name_type topic_name;
  using _data_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type_type data_type;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__node_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->node_name = _arg;
    return *this;
  }
  Type & set__source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->source = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }
  Type & set__topic_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic_name = _arg;
    return *this;
  }
  Type & set__data_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data_type = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    anomaly_msg::msg::AnomalyLog_<ContainerAllocator> *;
  using ConstRawPtr =
    const anomaly_msg::msg::AnomalyLog_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      anomaly_msg::msg::AnomalyLog_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      anomaly_msg::msg::AnomalyLog_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__anomaly_msg__msg__AnomalyLog
    std::shared_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__anomaly_msg__msg__AnomalyLog
    std::shared_ptr<anomaly_msg::msg::AnomalyLog_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AnomalyLog_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->node_name != other.node_name) {
      return false;
    }
    if (this->source != other.source) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    if (this->topic_name != other.topic_name) {
      return false;
    }
    if (this->data_type != other.data_type) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const AnomalyLog_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AnomalyLog_

// alias to use template instance with default allocator
using AnomalyLog =
  anomaly_msg::msg::AnomalyLog_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace anomaly_msg

#endif  // ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__STRUCT_HPP_
