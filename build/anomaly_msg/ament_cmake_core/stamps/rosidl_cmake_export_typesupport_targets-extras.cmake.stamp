# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:anomaly_msg__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:anomaly_msg__rosidl_typesupport_fastrtps_c;__rosidl_typesupport_introspection_c:anomaly_msg__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:anomaly_msg__rosidl_typesupport_c;__rosidl_generator_cpp:anomaly_msg__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:anomaly_msg__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_cpp:anomaly_msg__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:anomaly_msg__rosidl_typesupport_cpp;:anomaly_msg__rosidl_generator_py")

# populate anomaly_msg_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "anomaly_msg::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'anomaly_msg' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND anomaly_msg_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
