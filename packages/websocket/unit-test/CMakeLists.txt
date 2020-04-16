find_package(OpenSSL REQUIRED)

add_executable(soss-websocket-unit-test
  main.cpp
  websocket__jwt.cpp
  MsgpackSerialization.cpp
)

target_link_libraries(soss-websocket-unit-test
  PRIVATE
    soss-websocket
    yaml-cpp
    OpenSSL::SSL
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../../thirdparty")

target_include_directories(soss-websocket-unit-test
  PRIVATE
    "${thirdparty_dir}/catch2/include"
    $<TARGET_PROPERTY:soss-websocket,INTERFACE_INCLUDE_DIRECTORIES>
    OpenSSL::SSL
)

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")
include(Catch)
catch_discover_tests(soss-websocket-unit-test)
