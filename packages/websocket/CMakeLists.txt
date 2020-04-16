cmake_minimum_required(VERSION 3.5.0)

project(soss-websocket)

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

find_package(soss-core REQUIRED)
find_package(soss-json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(websocketpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time regex random)

if(NOT CMAKE_CXX_STANDARD)
  # TODO(MXG): Remove this block and use target_compile_features(~)
  # instead when we no longer need to support Ubuntu 16.04.
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
message(STATUS "Configuring [soss-websocket]")

add_library(soss-websocket SHARED
  src/Client.cpp
  src/Endpoint.cpp
  src/JwtValidator.cpp
  src/RosbridgeV2Encoding.cpp
  src/Server.cpp
  src/ServerConfig.cpp
  src/ServiceProvider.cpp
  src/TopicPublisher.cpp
  src/JsonSerializer.cpp
  src/BsonSerializer.cpp
  src/MsgpackSerializer.cpp
  src/MsgpackAdaptor.cpp
)

target_link_libraries(soss-websocket
  PUBLIC
    soss::core
    soss::json
    Boost::system
  PRIVATE
    OpenSSL::SSL
    Boost::date_time
    Boost::regex
    Boost::random
)

target_include_directories(soss-websocket
  PRIVATE
    OpenSSL::SSL
  PUBLIC
    ${WEBSOCKETPP_INCLUDE_DIR}
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cpp-jwt-v1.1.1/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/msgpack-3.2.1/include>
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)


###############################
# Install soss-websocket
soss_install_middleware_plugin(
  MIDDLEWARE websocket
  TARGET soss-websocket
  TYPES websocket_client websocket_server
)

include(CTest)

if(BUILD_TESTING)
  add_subdirectory(unit-test)
endif()
