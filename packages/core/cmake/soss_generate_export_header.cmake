# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# copied from soss/packages/core/cmake/soss_install_middleware_plugin.cmake

#################################################
# soss_generate_export_header(<component_name>)
#
# component_name: The name of the component whose export header should
#                 be generated.
function(soss_generate_export_header component_name)
  include(GenerateExportHeader)

  string(TOUPPER ${component_name} UPPER_COMPONENT_NAME)

  set(binary_include_dir ${CMAKE_BINARY_DIR}/include)
  generate_export_header(soss-${component_name}
    BASE_NAME soss_${component_name}
    EXPORT_MACRO_NAME SOSS_${UPPER_COMPONENT_NAME}_API
    EXPORT_FILE_NAME ${binary_include_dir}/soss/${component_name}/export.hpp
  )

  install(
    DIRECTORY ${binary_include_dir}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT soss-${component_name}
  )

  target_include_directories(soss-${component_name}
    PUBLIC
      $<BUILD_INTERFACE:${binary_include_dir}>
  )
endfunction()
