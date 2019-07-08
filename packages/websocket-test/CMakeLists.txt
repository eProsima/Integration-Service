cmake_minimum_required(VERSION 3.5.0)

project(soss-websocket-test)

find_package(soss-websocket REQUIRED)
find_package(soss-mock REQUIRED)
find_package(yaml-cpp)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

add_executable(soss-websocket-test
  main.cpp
  integration/websocket__roundtrip.cpp
  integration/websocket__dispatch.cpp
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty")

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")

include(CTest)
include(Catch)
catch_discover_tests(soss-websocket-test)

target_link_libraries(soss-websocket-test
  PRIVATE
    soss::mock
    yaml-cpp
)

target_include_directories(soss-websocket-test
  PRIVATE
    "${thirdparty_dir}/catch2/include"
)

foreach(resource
    websocket__roundtrip_server.yaml
    websocket__roundtrip_client.yaml
    websocket__dispatch.yaml)

  get_filename_component(resource_name ${resource} NAME_WE)
  string(TOUPPER ${resource_name} CAPS_RESOURCE_NAME)

  target_compile_definitions(soss-websocket-test
    PRIVATE
      "${CAPS_RESOURCE_NAME}__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/${resource}\""
  )

endforeach()
