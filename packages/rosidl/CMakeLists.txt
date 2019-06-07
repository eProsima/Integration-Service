cmake_minimum_required(VERSION 3.5.0)

project(soss-rosidl)

find_package(soss-core REQUIRED)
find_package(rosidl_parser QUIET)

if(NOT rosidl_parser_FOUND)

  message(FATAL_ERROR
    "Could not find package [rosidl_parser]. Without this, it will not be "
    "possible to generate soss translations for ROS2 message types. "
    "Please source a distribution of ROS2 to build soss-rosidl."
  )

endif()

include(GNUInstallDirs)

set(soss_rosidl_share_dir "${CMAKE_INSTALL_DATADIR}/soss-rosidl")

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/soss-rosidlConfig.cmake"
  DESTINATION "${soss_rosidl_share_dir}"
  COMPONENT soss-rosidl
)

install(
  DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/cmake"
  DESTINATION "${soss_rosidl_share_dir}"
  COMPONENT soss-rosidl
)

install(
  DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/scripts"
  DESTINATION "${soss_rosidl_share_dir}"
  COMPONENT soss-rosidl
)
