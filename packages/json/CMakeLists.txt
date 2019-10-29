cmake_minimum_required(VERSION 3.5.0)

project(soss-json)

find_package(fastrtps REQUIRED)
find_package(soss-core REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

message(STATUS "Configuring [soss-json]")

add_library(soss-json SHARED
  src/conversion.cpp
)

soss_generate_export_header(json)

target_link_libraries(soss-json
  PUBLIC
    soss::core
)

target_include_directories(soss-json
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

########################################################################
# Example
set(EXAMPLE_NAME ${PROJECT_NAME}-example)
add_executable(${EXAMPLE_NAME}
    example/main.cpp
)

set_target_properties(${EXAMPLE_NAME}
    PROPERTIES
        CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

target_compile_options(${EXAMPLE_NAME}
    PRIVATE
        -Werror -Wall -Wextra -Wpedantic
)

target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
        soss::core
        soss-json
)

###############################
# Install soss-json
set(soss_json_config_dir "${CMAKE_INSTALL_LIBDIR}/cmake/soss-json")

install(
  TARGETS soss-json
  EXPORT  soss-json
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  COMPONENT soss-json
)

install(
  EXPORT soss-json
  DESTINATION ${soss_json_config_dir}
  FILE soss-json-target.cmake
  NAMESPACE soss::
  COMPONENT soss-json
)

install(
  DIRECTORY   "${CMAKE_CURRENT_LIST_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT soss-json
)

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/soss-jsonConfig.cmake"
  DESTINATION "${soss_json_config_dir}"
  COMPONENT soss-json
)


# Install example
install(
    TARGETS ${EXAMPLE_NAME}
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
    COMPONENT ${PROJECT_NAME}
)

