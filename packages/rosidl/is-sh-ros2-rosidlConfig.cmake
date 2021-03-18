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
# Config file for the is-sh-ros2-rosidl utility for generating translations
# from ROS2 message specification types to eProsima xtypes
##################################################################################

cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

if(is-sh-ros2-rosidl_CONFIG_INCLUDED)
  return()
endif()
set(is-sh-ros2-rosidl_CONFIG_INCLUDED TRUE)

include(CMakeFindDependencyMacro)
find_dependency(is-core REQUIRED)

set(IS_SH_ROS2_ROSIDL_GENERATE_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/scripts/is_sh_ros2_rosidl_generate.py")
set(IS_SH_ROS2_ROSIDL_FIND_PACKAGE_INFO_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/scripts/is_sh_ros2_rosidl_find_package_info.py")

include("${CMAKE_CURRENT_LIST_DIR}/cmake/is_sh_ros2_rosidl_mix.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/is_sh_ros2_rosidl_install_extension.cmake")
