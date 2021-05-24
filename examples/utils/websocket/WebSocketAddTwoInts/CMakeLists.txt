# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# WebSocket AddTwoInts server CMake project

###################################################################################
# CMake build rules for the Integration Service WebSocket AddTwoInts server example
###################################################################################
cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

project(WebSocketAddTwoInts)

if (NOT (BUILD_EXAMPLES OR BUILD_WEBSOCKET_EXAMPLES))
    return()
endif()

find_package(websocketpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

message(STATUS "Configuring ${PROJECT_NAME} example...")

add_executable(${PROJECT_NAME}
    AddTwoInts_server.cpp
    AddTwoInts_main.cpp
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
)

target_include_directories(${PROJECT_NAME}
PRIVATE
    ${WEBSOCKETPP_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Threads::Threads
        OpenSSL::SSL
)