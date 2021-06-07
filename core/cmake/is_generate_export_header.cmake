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

# copied from is/packages/core/cmake/is_install_middleware_plugin.cmake

#################################################
# is_generate_export_header(<component_name>)
#
# component_name: The name of the component whose export header should
#                 be generated.
function(is_generate_export_header component_name)
  include(GenerateExportHeader)

  string(TOUPPER ${component_name} UPPER_COMPONENT_NAME)

  set(binary_include_dir ${CMAKE_BINARY_DIR}/include)
  generate_export_header(is-${component_name}
    BASE_NAME is_${component_name}
    EXPORT_MACRO_NAME IS_${UPPER_COMPONENT_NAME}_API
    EXPORT_FILE_NAME ${binary_include_dir}/is/${component_name}/export.hpp
  )

  file(
    COPY
      ${binary_include_dir}/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/include
  )

  target_include_directories(is-${component_name}
    PUBLIC
      $<BUILD_INTERFACE:${binary_include_dir}>
  )
endfunction()
