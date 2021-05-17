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
# - Config file for the core Integration Service package

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(is-core_CONFIG_INCLUDED)
  return()
endif()
set(is-core_CONFIG_INCLUDED TRUE)

if(NOT TARGET is::is-core)
  include("${CMAKE_CURRENT_LIST_DIR}/is-core-target.cmake")
endif()

if(NOT TARGET is::core)
  add_library(is::core INTERFACE IMPORTED)
  set_target_properties(is::core PROPERTIES
    INTERFACE_LINK_LIBRARIES is::is-core
  )
endif()

set(IS_TEMPLATE_DIR "${CMAKE_CURRENT_LIST_DIR}/templates")
set(IS_MIDDLEWARE_CONFIG_TEMPLATE "${IS_TEMPLATE_DIR}/middleware-config.cmake.in")
set(IS_IDL_MIDDLEWARE_MIX_CONFIG_TEMPLATE "${IS_TEMPLATE_DIR}/is-idl-middleware-mix-config.cmake.in")
set(IS_IDL_PKG_MIX_CONFIG_TEMPLATE "${IS_TEMPLATE_DIR}/is-idl-pkg-mix-config.cmake.in")
set(IS_IDL_MIDDLEWARE_MIX_EXTENSION_TEMPLATE "${IS_TEMPLATE_DIR}/is-idl-middleware-mix-extension.cmake.in")
set(IS_GTEST_CMAKE_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake/common")
set(IS_DOXYGEN_CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/../../doxygen-config.in")

include("${CMAKE_CURRENT_LIST_DIR}/cmake/is_generate_export_header.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/is_install_middleware_plugin.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/is_mix_generator.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/is_mix_install_extension.cmake")

include(CMakeFindDependencyMacro)
find_dependency(yaml-cpp)
find_package(xtypes REQUIRED)

set(is-core_FOUND TRUE)
