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

include(CMakeParseArguments)
include(GNUInstallDirs)

#################################################
# is_ros2_rosidl_install_extension(
#   MIDDLEWARE <middleware>
#   [MESSAGE
#     [SOURCES <message-source-templates>]
#     [HEADERS <message-header-templates>] ]
#   [SERVICE
#     [SOURCES <service-source-templates>]
#     [HEADERS <service-header-templates>] ]
#   [DEPENDENCIES <middleware dependencies>]
# )
function(is_ros2_rosidl_install_extension)

  is_mix_install_extension(
    IDL_TYPE rosidl
    ${ARGN}
  )

endfunction()
