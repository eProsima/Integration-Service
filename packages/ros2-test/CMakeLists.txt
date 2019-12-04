cmake_minimum_required(VERSION 3.5.0)

project(soss-ros2-test)

# Get soss dependencies
find_package(xtypes REQUIRED)
find_package(soss-core REQUIRED)
find_package(soss-mock REQUIRED)
find_package(soss-rosidl REQUIRED)
find_package(soss-ros2 REQUIRED)

# Get message dependencies
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(test_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

soss_rosidl_mix(
  PACKAGES geometry_msgs nav_msgs test_msgs
  MIDDLEWARES ros2
)

add_executable(soss-ros2-test
  main.cpp
  integration/ros2__geometry_msgs.cpp
  integration/ros2__test_msgs.cpp
  integration/ros2__test_domain.cpp
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty")

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")

include(CTest)
include(Catch)
catch_discover_tests(soss-ros2-test)

target_link_libraries(soss-ros2-test
  PRIVATE
    soss::mock
    soss::ros2
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${test_msgs_LIBRARIES}
)

target_include_directories(soss-ros2-test
  PRIVATE
    "${thirdparty_dir}/catch2/include"
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${test_msgs_INCLUDE_DIRS}
)

if($ENV{ROS_DISTRO} STREQUAL dashing)
  set(test_msgs_config_file "ros2__test_msgs_dashing.yaml")
else()
  set(test_msgs_config_file "ros2__test_msgs_crystal.yaml")
endif()

target_compile_definitions(soss-ros2-test
  PRIVATE
    "ROS2__GEOMETRY_MSGS__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/ros2__geometry_msgs.yaml\""
    "ROS2__TEST_MSGS__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/${test_msgs_config_file}\""
    "ROS2__TEST_DOMAIN__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/ros2__domain_change.yaml\""
    "ROS2__ROSIDL__BUILD_DIR=\"${CMAKE_BINARY_DIR}/soss/rosidl/ros2/lib\""
)

# Windows dll dependencies installation
if(WIN32)
  find_file(MOCKDLL NAMES "soss-mock.dll" PATHS "${soss-mock_DIR}" PATH_SUFFIXES "lib" )
  install (FILES ${MOCKDLL} DESTINATION "${CMAKE_INSTALL_PREFIX}/../bin")
endif()
