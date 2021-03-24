cmake_minimum_required(VERSION 3.5.0)

project(is-websocket-test)

find_package(is-websocket REQUIRED)
find_package(is-mock REQUIRED)
find_package(is-json REQUIRED)
find_package(xtypes REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(websocketpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

add_executable(${PROJECT_NAME}
  main.cpp
  integration/websocket__roundtrip.cpp
  integration/websocket__dispatch.cpp
  integration/websocket__services.cpp
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty")

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")

include(CTest)
include(Catch)
catch_discover_tests(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    is::mock
    pthread
    OpenSSL::SSL
    is::json
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    "${thirdparty_dir}/catch2/include"
    ${WEBSOCKETPP_INCLUDE_DIR}
)

foreach(resource
    websocket__roundtrip_server.yaml
    websocket__roundtrip_client.yaml
    websocket__dispatch.yaml
    websocket__services.yaml)

  get_filename_component(resource_name ${resource} NAME_WE)
  string(TOUPPER ${resource_name} CAPS_RESOURCE_NAME)

  target_compile_definitions(${PROJECT_NAME}
    PRIVATE
      "${CAPS_RESOURCE_NAME}__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/${resource}\""
  )

endforeach()

# Windows dll dependencies installation
if(WIN32)
  find_file(JSONDLL NAMES "is-json.dll" PATHS "${is-json_DIR}" PATH_SUFFIXES "lib" )
  install (FILES ${JSONDLL} DESTINATION "${CMAKE_INSTALL_PREFIX}/../bin")
endif()
