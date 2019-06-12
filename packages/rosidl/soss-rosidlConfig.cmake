# - Config file for the soss-rosidl utility for generating soss translations
#   from ROS1/ROS2 message specifications

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(soss-rosidl_CONFIG_INCLUDED)
  return()
endif()
set(soss-rosidl_CONFIG_INCLUDED TRUE)

include(CMakeFindDependencyMacro)
find_dependency(soss-core REQUIRED)

set(SOSS_ROSIDL_GENERATE_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/scripts/soss_rosidl_generate.py")
set(SOSS_ROSIDL_FIND_PACKAGE_INFO_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/scripts/soss_rosidl_find_package_info.py")

include("${CMAKE_CURRENT_LIST_DIR}/cmake/soss_rosidl_mix.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/soss_rosidl_install_extension.cmake")
