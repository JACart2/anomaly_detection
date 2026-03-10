// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from anomaly_msg:msg/AnomalyLog.idl
// generated code does not contain a copyright notice

#include "anomaly_msg/msg/detail/anomaly_log__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_anomaly_msg
const rosidl_type_hash_t *
anomaly_msg__msg__AnomalyLog__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x95, 0x9a, 0x43, 0x07, 0x19, 0xed, 0x3f, 0x23,
      0xe2, 0x95, 0xf2, 0x5d, 0x64, 0x81, 0x59, 0x65,
      0xc0, 0xf3, 0x7c, 0x26, 0x51, 0x59, 0x73, 0xe4,
      0x69, 0xe3, 0xf9, 0xca, 0x2f, 0x44, 0x9e, 0xe6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char anomaly_msg__msg__AnomalyLog__TYPE_NAME[] = "anomaly_msg/msg/AnomalyLog";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char anomaly_msg__msg__AnomalyLog__FIELD_NAME__stamp[] = "stamp";
static char anomaly_msg__msg__AnomalyLog__FIELD_NAME__node_name[] = "node_name";
static char anomaly_msg__msg__AnomalyLog__FIELD_NAME__source[] = "source";
static char anomaly_msg__msg__AnomalyLog__FIELD_NAME__description[] = "description";
static char anomaly_msg__msg__AnomalyLog__FIELD_NAME__topic_name[] = "topic_name";
static char anomaly_msg__msg__AnomalyLog__FIELD_NAME__data_type[] = "data_type";
static char anomaly_msg__msg__AnomalyLog__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field anomaly_msg__msg__AnomalyLog__FIELDS[] = {
  {
    {anomaly_msg__msg__AnomalyLog__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {anomaly_msg__msg__AnomalyLog__FIELD_NAME__node_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {anomaly_msg__msg__AnomalyLog__FIELD_NAME__source, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {anomaly_msg__msg__AnomalyLog__FIELD_NAME__description, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {anomaly_msg__msg__AnomalyLog__FIELD_NAME__topic_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {anomaly_msg__msg__AnomalyLog__FIELD_NAME__data_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {anomaly_msg__msg__AnomalyLog__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription anomaly_msg__msg__AnomalyLog__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
anomaly_msg__msg__AnomalyLog__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {anomaly_msg__msg__AnomalyLog__TYPE_NAME, 26, 26},
      {anomaly_msg__msg__AnomalyLog__FIELDS, 7, 7},
    },
    {anomaly_msg__msg__AnomalyLog__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Anomaly_log.msg\n"
  "builtin_interfaces/Time stamp\n"
  "\n"
  "string node_name            # name of node publishing\n"
  "string source               # camera, lidar, mic, etc. \n"
  "string description          # relevant info, e.g. front camera\n"
  "\n"
  "string topic_name           # original topic name\n"
  "string data_type            # image, float32, pointcloud, etc. \n"
  "uint8[] data                # relevant data\n"
  "\n"
  "\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
anomaly_msg__msg__AnomalyLog__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {anomaly_msg__msg__AnomalyLog__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 384, 384},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
anomaly_msg__msg__AnomalyLog__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *anomaly_msg__msg__AnomalyLog__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
