find_package(OpenSSL REQUIRED)

include(CTest)
include(${PROJECT_SOURCE_DIR}/../core/cmake/common/gtest.cmake)
enable_testing()

add_executable(soss-websocket-unit-test
  websocket__jwt.cpp
)

target_link_libraries(soss-websocket-unit-test
  PRIVATE
    soss-websocket
    yaml-cpp
    OpenSSL::SSL
  PUBLIC
    libgtest
)

target_include_directories(soss-websocket-unit-test
  PRIVATE
    $<TARGET_PROPERTY:soss-websocket,INTERFACE_INCLUDE_DIRECTORIES>
    OpenSSL::SSL
)

add_gtest(soss-websocket-unit-test SOURCES websocket__jwt.cpp)
