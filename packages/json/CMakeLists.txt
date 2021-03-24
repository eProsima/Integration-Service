cmake_minimum_required(VERSION 3.5.0)

project(is-json)

find_package(xtypes REQUIRED)
find_package(is-core REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

message(STATUS "Configuring [${PROJECT_NAME}]")

add_library(${PROJECT_NAME} SHARED
  src/conversion.cpp
)

is_generate_export_header(json)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    is::core
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

########################################################################
## Example
#set(EXAMPLE_NAME ${PROJECT_NAME}-example)
#add_executable(${EXAMPLE_NAME}
#    example/main.cpp
#)
#
#set_target_properties(${EXAMPLE_NAME}
#    PROPERTIES
#        CXX_STANDARD 14
#        POSITION_INDEPENDENT_CODE ON
#)
#
#target_compile_options(${EXAMPLE_NAME}
#    PRIVATE
#        -Werror -Wall -Wextra -Wpedantic
#)
#
#target_link_libraries(${EXAMPLE_NAME}
#    PRIVATE
#        is::core
#        ${PROJECT_NAME}
#)
#
###############################
# Install ${PROJECT_NAME}
set(is_json_config_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  COMPONENT ${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}
  DESTINATION ${is_json_config_dir}
  FILE ${PROJECT_NAME}-target.cmake
  NAMESPACE is::
  COMPONENT ${PROJECT_NAME}
)

install(
  DIRECTORY   "${CMAKE_CURRENT_LIST_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT ${PROJECT_NAME}
)

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${is_json_config_dir}"
  COMPONENT ${PROJECT_NAME}
)


# Install example
#install(
#    TARGETS ${EXAMPLE_NAME}
#    DESTINATION "${CMAKE_INSTALL_BINDIR}"
#    COMPONENT ${PROJECT_NAME}
#)

