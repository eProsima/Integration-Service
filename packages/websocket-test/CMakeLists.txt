cmake_minimum_required(VERSION 3.5.0)

project(soss-websocket-test)

find_package(soss-websocket REQUIRED)
find_package(soss-mock REQUIRED)
find_package(soss-json REQUIRED)
find_package(xtypes REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(websocketpp REQUIRED)

macro(compile_test)
    # Parse arguments
    if("${ARGV0}" STREQUAL "NAME")
        set(uniValueArgs NAME)
    else()
        set(TEST_NAME "${ARGV0}")
    endif()
    set(multiValueArgs SOURCE)
    cmake_parse_arguments(TEST "" "${uniValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${TEST_NAME} ${TEST_SOURCE})

    target_link_libraries(${TEST_NAME}
        PUBLIC
            soss::mock
            soss::websocket
            soss::json
            Threads::Threads
            OpenSSL::SSL
        PRIVATE
            libgtest
        )

    target_include_directories(${TEST_NAME}
      PRIVATE
          ${WEBSOCKETPP_INCLUDE_DIR}
    )

    target_compile_options(${TEST_NAME}
        INTERFACE
            $<$<CXX_COMPILER_ID:GNU>:-Wall>
            $<$<CXX_COMPILER_ID:GNU>:-Wextra>
        )

    set_target_properties(${TEST_NAME}
        PROPERTIES
            CXX_STANDARD 17
        )

    foreach(resource
        websocket__roundtrip_server.yaml
        websocket__roundtrip_client.yaml
        websocket__dispatch__security.yaml
        websocket__dispatch__nosecurity.yaml
        websocket__services__security.yaml
        websocket__services__nosecurity.yaml)

      get_filename_component(resource_name ${resource} NAME_WE)
      string(TOUPPER ${resource_name} CAPS_RESOURCE_NAME)

      target_compile_definitions(${TEST_NAME}
        PRIVATE
          "${CAPS_RESOURCE_NAME}__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/${resource}\""
      )

    endforeach()

    add_gtest(${TEST_NAME} SOURCES ${TEST_SOURCE})
endmacro()

include(CTest)
include(${PROJECT_SOURCE_DIR}/../core/cmake/common/gtest.cmake)
enable_testing()

compile_test(${PROJECT_NAME}_rountrip_test SOURCE integration/websocket__roundtrip.cpp)
compile_test(${PROJECT_NAME}_dispatch_test SOURCE integration/websocket__dispatch.cpp)
compile_test(${PROJECT_NAME}_services_test SOURCE integration/websocket__services.cpp)

# Windows dll dependencies installation
if(WIN32)
  find_file(JSONDLL NAMES "soss-json.dll" PATHS "${soss-json_DIR}" PATH_SUFFIXES "lib" )
  install (FILES ${JSONDLL} DESTINATION "${CMAKE_INSTALL_PREFIX}/../bin")
endif()
