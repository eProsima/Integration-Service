# - Config file for the json<-->xtypes conversion package

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(is-json-xtypes_CONFIG_INCLUDED)
  return()
endif()
set(is-json-xtypes_CONFIG_INCLUDED TRUE)

if(NOT TARGET is::json-xtypes)
  include("${CMAKE_CURRENT_LIST_DIR}/is-json-xtypes-target.cmake")
endif()

if(NOT TARGET is::json-xtypes)
  add_library(is::json-xtypes INTERFACE IMPORTED)
  set_target_properties(is::json-xtypes PROPERTIES
    INTERFACE_LINK_LIBRARIES is::is-json-xtypes
  )
endif()

set(is-json-xtypes_FOUND TRUE)
