cmake_minimum_required(VERSION 3.5.0)

project(is_ros2_turtle_types)

find_package(ament_cmake REQUIRED)
find_package(is-core REQUIRED)
find_package(is-ros2 REQUIRED)
find_package(is-ros2-rosidl REQUIRED)
find_package(geometry_msgs REQUIRED)


if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 17)
endif()

is_ros2_rosidl_mix(
    PACKAGES geometry_msgs
    MIDDLEWARES ros2
)

ament_package()
