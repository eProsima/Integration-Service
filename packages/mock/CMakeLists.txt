cmake_minimum_required(VERSION 3.5.0)

project(soss-mock)

find_package(soss-core REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "Configuring [soss-mock]")

include(GNUInstallDirs)

add_library(soss-mock SHARED
  src/SystemHandle.cpp
)

target_link_libraries(soss-mock
  PUBLIC
    soss::core
    yaml-cpp
)

target_include_directories(soss-mock
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# Install soss-mock
soss_install_middleware_plugin(
  MIDDLEWARE mock
  TARGET soss-mock
)

install(
  DIRECTORY   ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT soss-mock
)
