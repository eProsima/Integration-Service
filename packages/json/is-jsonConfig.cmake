# - Config file for the json<-->is conversion package

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(is-json_CONFIG_INCLUDED)
  return()
endif()
set(is-json_CONFIG_INCLUDED TRUE)

if(NOT TARGET is::is-json)
  include("${CMAKE_CURRENT_LIST_DIR}/is-json-target.cmake")
endif()

if(NOT TARGET is::json)
  add_library(is::json INTERFACE IMPORTED)
  set_target_properties(is::json PROPERTIES
    INTERFACE_LINK_LIBRARIES is::is-json
  )
endif()

set(is-json_FOUND TRUE)
