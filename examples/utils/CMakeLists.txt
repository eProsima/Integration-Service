# Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

cmake_minimum_required(VERSION 2.8.12)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(is-examples)

if (BUILD_EXAMPLES OR BUILD_FASTDDS_EXAMPLES)
    add_subdirectory(dds/DDSAddTwoInts)
    add_subdirectory(dds/DDSHelloWorld)
endif()

if (BUILD_EXAMPLES OR BUILD_ROS1_EXAMPLES)
    if(NOT DEFINED ENV{ROS_VERSION} OR $ENV{ROS_VERSION} GREATER 1)
        message(FATAL_ERROR "Cannot BUILD_ROS1_EXAMPLES without sourcing a ROS1 overlay")
    endif()
    # The ROS1 overlay sets the environment variable CMAKE_PREFIX_PATH to ROS1 installation dir
    # Note execute_process WORKING_DIRECTORY doesn't work on the focal container
    execute_process(COMMAND catkin_make install -DBUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX=$ENV{CMAKE_PREFIX_PATH} -C ${CMAKE_CURRENT_LIST_DIR}/ros1/catkin_ws
        OUTPUT_VARIABLE CATKINRES)
    message(STATUS ${CATKINRES})
endif()

if (BUILD_EXAMPLES OR BUILD_WEBSOCKET_EXAMPLES)
    add_subdirectory(websocket/WebSocketAddTwoInts)
endif()


