// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "anomaly_msg/msg/anomaly_log.hpp"


#ifndef ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__BUILDER_HPP_
#define ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "anomaly_msg/msg/detail/anomaly_log__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace anomaly_msg
{

namespace msg
{

namespace builder
{

class Init_AnomalyLog_data
{
public:
  explicit Init_AnomalyLog_data(::anomaly_msg::msg::AnomalyLog & msg)
  : msg_(msg)
  {}
  ::anomaly_msg::msg::AnomalyLog data(::anomaly_msg::msg::AnomalyLog::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::anomaly_msg::msg::AnomalyLog msg_;
};

class Init_AnomalyLog_data_type
{
public:
  explicit Init_AnomalyLog_data_type(::anomaly_msg::msg::AnomalyLog & msg)
  : msg_(msg)
  {}
  Init_AnomalyLog_data data_type(::anomaly_msg::msg::AnomalyLog::_data_type_type arg)
  {
    msg_.data_type = std::move(arg);
    return Init_AnomalyLog_data(msg_);
  }

private:
  ::anomaly_msg::msg::AnomalyLog msg_;
};

class Init_AnomalyLog_topic_name
{
public:
  explicit Init_AnomalyLog_topic_name(::anomaly_msg::msg::AnomalyLog & msg)
  : msg_(msg)
  {}
  Init_AnomalyLog_data_type topic_name(::anomaly_msg::msg::AnomalyLog::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return Init_AnomalyLog_data_type(msg_);
  }

private:
  ::anomaly_msg::msg::AnomalyLog msg_;
};

class Init_AnomalyLog_description
{
public:
  explicit Init_AnomalyLog_description(::anomaly_msg::msg::AnomalyLog & msg)
  : msg_(msg)
  {}
  Init_AnomalyLog_topic_name description(::anomaly_msg::msg::AnomalyLog::_description_type arg)
  {
    msg_.description = std::move(arg);
    return Init_AnomalyLog_topic_name(msg_);
  }

private:
  ::anomaly_msg::msg::AnomalyLog msg_;
};

class Init_AnomalyLog_source
{
public:
  explicit Init_AnomalyLog_source(::anomaly_msg::msg::AnomalyLog & msg)
  : msg_(msg)
  {}
  Init_AnomalyLog_description source(::anomaly_msg::msg::AnomalyLog::_source_type arg)
  {
    msg_.source = std::move(arg);
    return Init_AnomalyLog_description(msg_);
  }

private:
  ::anomaly_msg::msg::AnomalyLog msg_;
};

class Init_AnomalyLog_node_name
{
public:
  explicit Init_AnomalyLog_node_name(::anomaly_msg::msg::AnomalyLog & msg)
  : msg_(msg)
  {}
  Init_AnomalyLog_source node_name(::anomaly_msg::msg::AnomalyLog::_node_name_type arg)
  {
    msg_.node_name = std::move(arg);
    return Init_AnomalyLog_source(msg_);
  }

private:
  ::anomaly_msg::msg::AnomalyLog msg_;
};

class Init_AnomalyLog_stamp
{
public:
  Init_AnomalyLog_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AnomalyLog_node_name stamp(::anomaly_msg::msg::AnomalyLog::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_AnomalyLog_node_name(msg_);
  }

private:
  ::anomaly_msg::msg::AnomalyLog msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::anomaly_msg::msg::AnomalyLog>()
{
  return anomaly_msg::msg::builder::Init_AnomalyLog_stamp();
}

}  // namespace anomaly_msg

#endif  // ANOMALY_MSG__MSG__DETAIL__ANOMALY_LOG__BUILDER_HPP_
