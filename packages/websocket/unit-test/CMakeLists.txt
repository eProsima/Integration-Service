find_package(OpenSSL REQUIRED)

add_executable(is-websocket-unit-test
  main.cpp
  websocket__jwt.cpp
)

target_link_libraries(is-websocket-unit-test
  PRIVATE
    is-websocket
    yaml-cpp
    OpenSSL::SSL
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../../thirdparty")

target_include_directories(is-websocket-unit-test
  PRIVATE
    "${thirdparty_dir}/catch2/include"
    $<TARGET_PROPERTY:is-websocket,INTERFACE_INCLUDE_DIRECTORIES>
    OpenSSL::SSL
)

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")
include(Catch)
catch_discover_tests(is-websocket-unit-test)
