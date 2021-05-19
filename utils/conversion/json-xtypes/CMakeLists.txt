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
# Integration Service JSON to xTypes library and tests CMake project

###################################################################################
# CMake build rules for the Integration Service JSON-xTypes conversion library
###################################################################################
cmake_minimum_required(VERSION 3.5.0)

project(is-json-xtypes VERSION "3.0.0" LANGUAGES CXX)

###################################################################################
# Configure options
###################################################################################
option(BUILD_LIBRARY "Compile the Integration Service" ON)

if(NOT BUILD_LIBRARY)
    return()
endif()

###############################################################################
# Load external CMake Modules.
###############################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SANITIZERS_ROOT}/cmake)

find_package(Sanitizers QUIET)

if(SANITIZE_ADDRESS)
    message(STATUS "Preloading AddressSanitizer library could be done using \"${ASan_WRAPPER}\" to run your program.")
endif()

###################################################################################
# External dependencies for the Integration Service JSON-xTypes conversion library
###################################################################################

find_package(is-core REQUIRED)
find_package(xtypes REQUIRED)

###################################################################################
# Configure the Integration Service JSON-xTypes conversion library
###################################################################################
add_library(${PROJECT_NAME}
    SHARED
        src/conversion.cpp
    )

if(Sanitizers_FOUND)
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

include(GNUInstallDirs)
message(STATUS "Configuring [${PROJECT_NAME}]...")

is_generate_export_header(json-xtypes)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        is::core
    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

###################################################################################
# Configure the Integration Service JSON-xTypes examples
###################################################################################
set(EXAMPLE_NAME ${PROJECT_NAME}-example)

add_executable(${EXAMPLE_NAME}
    example/main.cpp
    )

set_target_properties(${EXAMPLE_NAME}
    PROPERTIES
        CXX_STANDARD
            17
        POSITION_INDEPENDENT_CODE
            ON
    )

target_compile_options(${EXAMPLE_NAME}
    PRIVATE
        -Werror -Wall -Wextra -Wpedantic
    )

target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
        is::core
        ${PROJECT_NAME}
    )

###################################################################################
# Install the Integration Service JSON-xTypes conversion library
###################################################################################
set(json_xtypes_config_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

install(
    TARGETS
        ${PROJECT_NAME}
    EXPORT
        ${PROJECT_NAME}
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}
    COMPONENT
        ${PROJECT_NAME}
)

install(
    EXPORT
        ${PROJECT_NAME}
    DESTINATION
        ${json_xtypes_config_dir}
    FILE
        ${PROJECT_NAME}-target.cmake
    NAMESPACE
        is::
    COMPONENT
        ${PROJECT_NAME}
    )

install(
    DIRECTORY
        "${CMAKE_CURRENT_LIST_DIR}/include/"
    DESTINATION
        "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT
        ${PROJECT_NAME}
    )

install(
    FILES
        "${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION
        "${json_xtypes_config_dir}"
    COMPONENT
        ${PROJECT_NAME}
    )

install(
    TARGETS
        ${EXAMPLE_NAME}
    DESTINATION
        "${CMAKE_INSTALL_BINDIR}"
    COMPONENT
        ${PROJECT_NAME}
    )

