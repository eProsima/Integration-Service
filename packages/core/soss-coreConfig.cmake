# - Config file for the core soss package

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(soss-core_CONFIG_INCLUDED)
  return()
endif()
set(soss-core_CONFIG_INCLUDED TRUE)

if(NOT TARGET soss::soss-core)
  include("${CMAKE_CURRENT_LIST_DIR}/soss-core-target.cmake")
endif()

if(NOT TARGET soss::core)
  add_library(soss::core INTERFACE IMPORTED)
  set_target_properties(soss::core PROPERTIES
    INTERFACE_LINK_LIBRARIES soss::soss-core
  )
endif()

set(SOSS_TEMPLATE_DIR "${CMAKE_CURRENT_LIST_DIR}/templates")
set(SOSS_MIDDLEWARE_CONFIG_TEMPLATE "${SOSS_TEMPLATE_DIR}/middleware-config.cmake.in")
set(SOSS_IDL_MIDDLEWARE_MIX_CONFIG_TEMPLATE "${SOSS_TEMPLATE_DIR}/soss-idl-middleware-mix-config.cmake.in")
set(SOSS_IDL_PKG_MIX_CONFIG_TEMPLATE "${SOSS_TEMPLATE_DIR}/soss-idl-pkg-mix-config.cmake.in")
set(SOSS_IDL_MIDDLEWARE_MIX_EXTENSION_TEMPLATE "${SOSS_TEMPLATE_DIR}/soss-idl-middleware-mix-extension.cmake.in")

include("${CMAKE_CURRENT_LIST_DIR}/cmake/soss_install_middleware_plugin.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/soss_mix_generator.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/soss_mix_install_extension.cmake")

set(soss-core_FOUND TRUE)
