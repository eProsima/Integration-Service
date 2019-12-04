cmake_minimum_required(VERSION 3.5.0)

project(soss-ros2)

find_package(xtypes REQUIRED)
find_package(soss-core REQUIRED)
find_package(soss-rosidl REQUIRED)
find_package(rclcpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

message(STATUS "Configuring [soss-ros2]")

add_library(soss-ros2 SHARED
  src/Factory.cpp
  src/SystemHandle.cpp
  src/MetaPublisher.cpp
)

soss_generate_export_header(ros2)

target_link_libraries(soss-ros2
  PUBLIC
    soss::core
    ${rclcpp_LIBRARIES}
)

target_include_directories(soss-ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
)

if (NOT rosidl_runtime_cpp_FOUND)
  target_compile_definitions(soss-ros2
    PUBLIC
      "SOSS_ROS2__ROSIDL_GENERATOR_CPP"
  )
endif()


###############################
# Install soss-ros2
soss_install_middleware_plugin(
  MIDDLEWARE ros2
  TARGET soss-ros2
  DEPENDENCIES rclcpp
)

install(
  DIRECTORY   ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT soss-ros2
)


###############################
# Install soss-rosidl-ros2-mix
soss_rosidl_install_extension(
  MIDDLEWARE ros2
  MESSAGE
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.cpp.em"
    HEADERS "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.hpp.em"
  SERVICE
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__srv.cpp.em"
)
