# - Config file for the json<-->soss conversion package

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(soss-json_CONFIG_INCLUDED)
  return()
endif()
set(soss-json_CONFIG_INCLUDED TRUE)

if(NOT TARGET soss::soss-json)
  include("${CMAKE_CURRENT_LIST_DIR}/soss-json-target.cmake")
endif()

if(NOT TARGET soss::json)
  add_library(soss::json INTERFACE IMPORTED)
  set_target_properties(soss::json PROPERTIES
    INTERFACE_LINK_LIBRARIES soss::soss-json
  )
endif()

set(soss-json_FOUND TRUE)
