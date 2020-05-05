cmake_minimum_required(VERSION 3.5.0)

project(soss_ros2_test_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

set(msg_files
  "msg/BoundedArrayNested.msg"
  "msg/Primitives.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ""
  ""
)

ament_package()
