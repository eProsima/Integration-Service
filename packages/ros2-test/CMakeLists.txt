cmake_minimum_required(VERSION 3.5.0)

project(is-ros2-test)

#TODO (@jamoralp): make this file right
# Get IS dependencies
find_package(xtypes REQUIRED)
find_package(is-core REQUIRED)
find_package(is-mock REQUIRED)
find_package(is-ros2-rosidl REQUIRED)
find_package(is-ros2 REQUIRED)

# Get message dependencies
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(is_ros2_test_msg REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

is_ros2_rosidl_mix(
  PACKAGES geometry_msgs nav_msgs is_ros2_test_msg
  MIDDLEWARES ros2 #TODO (@jamoralp) fix this
)

add_executable(${PROJECT_NAME}
  main.cpp
  integration/ros2__geometry_msgs.cpp
  integration/ros2__test_msgs.cpp
  integration/ros2__test_domain.cpp
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty")

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")

include(CTest)
include(Catch)
catch_discover_tests(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    is::mock
    is::ros2
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${is_ros2_test_msg_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    "${thirdparty_dir}/catch2/include"
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${is_ros2_test_msg_INCLUDE_DIRS}
)

set(test_msgs_config_file "ros2__test_msgs.yaml")

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    "ROS2__GEOMETRY_MSGS__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/ros2__geometry_msgs.yaml\""
    "ROS2__TEST_MSGS__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/${test_msgs_config_file}\""
    "ROS2__TEST_DOMAIN__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/ros2__domain_change.yaml\""
    "ROS2__ROSIDL__BUILD_DIR=\"${CMAKE_BINARY_DIR}/is/rosidl/ros2/lib\""
)

# Windows dll dependencies installation
if(WIN32)
  find_file(MOCKDLL NAMES "is-mock.dll" PATHS "${is-mock_DIR}" PATH_SUFFIXES "lib" )
  install (FILES ${MOCKDLL} DESTINATION "${CMAKE_INSTALL_PREFIX}/../bin")
endif()
