# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# is-core library and integration-service executable CMake project

##################################################################################
# CMake build rules for the Integration Service ROS 2 SystemHandle library
##################################################################################
cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

project(is-sh-ros2 VERSION "0.1.0" LANGUAGES CXX)

find_package(is-core REQUIRED)
find_package(is-sh-ros2-rosidl REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Sanitizers QUIET)

if(SANITIZE_ADDRESS)
    message(STATUS "Preloading AddressSanitizer library could be done using \"${ASan_WRAPPER}\" to run your program.")
endif()

##################################################################################
# Configure the Integration Service ROS 2 SystemHandle library
##################################################################################
message(STATUS "Configuring [${PROJECT_NAME}]...")

add_library(${PROJECT_NAME}
  SHARED
    src/Factory.cpp
    src/SystemHandle.cpp
    src/MetaPublisher.cpp
  )

if (Sanitizers_FOUND)
  add_sanitizers(${PROJECT_NAME})
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION
    ${PROJECT_VERSION}
  SOVERSION
    ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
  CXX_STANDARD
    17
  CXX_STANDARD_REQUIRED
    YES
  )

target_compile_options(${PROJECT_NAME}
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-pedantic>
    $<$<CXX_COMPILER_ID:GNU>:-fstrict-aliasing>
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CXX_COMPILER_ID:GNU>:-Wextra>
    $<$<CXX_COMPILER_ID:GNU>:-Wcast-align>
    $<$<CXX_COMPILER_ID:GNU>:-Wshadow>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<CXX_COMPILER_ID:MSVC>:/wd4700>
    $<$<CXX_COMPILER_ID:MSVC>:/wd4996>
    $<$<CXX_COMPILER_ID:MSVC>:/wd4820>
    $<$<CXX_COMPILER_ID:MSVC>:/wd4255>
    $<$<CXX_COMPILER_ID:MSVC>:/wd4668>
  )

# Generate the export macro header
include(GNUInstallDirs)
is_generate_export_header(sh-ros2)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    is::core
    ${rclcpp_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
)

if (NOT rosidl_runtime_cpp_FOUND)
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC
      "IS_SH_ROS2__ROSIDL_GENERATOR_CPP"
  )
endif()

##################################################################################
# Install the Integration Service Mock SystemHandle library
##################################################################################
is_install_middleware_plugin(
  MIDDLEWARE
    ros2
  TARGET
    ${PROJECT_NAME}
  DEPENDENCIES
    rclcpp
  )

install(
  DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT
    ${PROJECT_NAME}
  )


  ##################################################################################
# Install the Integration Service is-sh-ros2-rosidl-mix plugin
##################################################################################
is_sh_ros2_rosidl_install_extension(
  MIDDLEWARE
    ros2
  MESSAGE
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.cpp.em"
    HEADERS "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.hpp.em"
  SERVICE
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__srv.cpp.em"
  )
