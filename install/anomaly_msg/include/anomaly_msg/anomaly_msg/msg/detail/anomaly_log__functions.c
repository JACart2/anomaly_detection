// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice
#include "anomaly_msg/msg/detail/anomaly_log__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `node_name`
// Member `source`
// Member `description`
// Member `topic_name`
// Member `data_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
anomaly_msg__msg__AnomalyLog__init(anomaly_msg__msg__AnomalyLog * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
    return false;
  }
  // node_name
  if (!rosidl_runtime_c__String__init(&msg->node_name)) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
    return false;
  }
  // source
  if (!rosidl_runtime_c__String__init(&msg->source)) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__init(&msg->description)) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__init(&msg->topic_name)) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
    return false;
  }
  // data_type
  if (!rosidl_runtime_c__String__init(&msg->data_type)) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
    return false;
  }
  return true;
}

void
anomaly_msg__msg__AnomalyLog__fini(anomaly_msg__msg__AnomalyLog * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // node_name
  rosidl_runtime_c__String__fini(&msg->node_name);
  // source
  rosidl_runtime_c__String__fini(&msg->source);
  // description
  rosidl_runtime_c__String__fini(&msg->description);
  // topic_name
  rosidl_runtime_c__String__fini(&msg->topic_name);
  // data_type
  rosidl_runtime_c__String__fini(&msg->data_type);
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
anomaly_msg__msg__AnomalyLog__are_equal(const anomaly_msg__msg__AnomalyLog * lhs, const anomaly_msg__msg__AnomalyLog * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // node_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->node_name), &(rhs->node_name)))
  {
    return false;
  }
  // source
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->source), &(rhs->source)))
  {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->description), &(rhs->description)))
  {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic_name), &(rhs->topic_name)))
  {
    return false;
  }
  // data_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->data_type), &(rhs->data_type)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
anomaly_msg__msg__AnomalyLog__copy(
  const anomaly_msg__msg__AnomalyLog * input,
  anomaly_msg__msg__AnomalyLog * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // node_name
  if (!rosidl_runtime_c__String__copy(
      &(input->node_name), &(output->node_name)))
  {
    return false;
  }
  // source
  if (!rosidl_runtime_c__String__copy(
      &(input->source), &(output->source)))
  {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__copy(
      &(input->description), &(output->description)))
  {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__copy(
      &(input->topic_name), &(output->topic_name)))
  {
    return false;
  }
  // data_type
  if (!rosidl_runtime_c__String__copy(
      &(input->data_type), &(output->data_type)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

anomaly_msg__msg__AnomalyLog *
anomaly_msg__msg__AnomalyLog__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  anomaly_msg__msg__AnomalyLog * msg = (anomaly_msg__msg__AnomalyLog *)allocator.allocate(sizeof(anomaly_msg__msg__AnomalyLog), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(anomaly_msg__msg__AnomalyLog));
  bool success = anomaly_msg__msg__AnomalyLog__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
anomaly_msg__msg__AnomalyLog__destroy(anomaly_msg__msg__AnomalyLog * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    anomaly_msg__msg__AnomalyLog__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
anomaly_msg__msg__AnomalyLog__Sequence__init(anomaly_msg__msg__AnomalyLog__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  anomaly_msg__msg__AnomalyLog * data = NULL;

  if (size) {
    data = (anomaly_msg__msg__AnomalyLog *)allocator.zero_allocate(size, sizeof(anomaly_msg__msg__AnomalyLog), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = anomaly_msg__msg__AnomalyLog__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        anomaly_msg__msg__AnomalyLog__fini(&data[i - 1]);
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
anomaly_msg__msg__AnomalyLog__Sequence__fini(anomaly_msg__msg__AnomalyLog__Sequence * array)
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
      anomaly_msg__msg__AnomalyLog__fini(&array->data[i]);
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

anomaly_msg__msg__AnomalyLog__Sequence *
anomaly_msg__msg__AnomalyLog__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  anomaly_msg__msg__AnomalyLog__Sequence * array = (anomaly_msg__msg__AnomalyLog__Sequence *)allocator.allocate(sizeof(anomaly_msg__msg__AnomalyLog__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = anomaly_msg__msg__AnomalyLog__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
anomaly_msg__msg__AnomalyLog__Sequence__destroy(anomaly_msg__msg__AnomalyLog__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    anomaly_msg__msg__AnomalyLog__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
anomaly_msg__msg__AnomalyLog__Sequence__are_equal(const anomaly_msg__msg__AnomalyLog__Sequence * lhs, const anomaly_msg__msg__AnomalyLog__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!anomaly_msg__msg__AnomalyLog__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
anomaly_msg__msg__AnomalyLog__Sequence__copy(
  const anomaly_msg__msg__AnomalyLog__Sequence * input,
  anomaly_msg__msg__AnomalyLog__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(anomaly_msg__msg__AnomalyLog);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    anomaly_msg__msg__AnomalyLog * data =
      (anomaly_msg__msg__AnomalyLog *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!anomaly_msg__msg__AnomalyLog__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          anomaly_msg__msg__AnomalyLog__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!anomaly_msg__msg__AnomalyLog__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
