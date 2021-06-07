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

# copied from is/packages/core/cmake/is_mix_install_extension.cmake

include(CMakeParseArguments)
include(GNUInstallDirs)

#################################################
# is_mix_install_extension(
#   IDL_TYPE   <idl_type>
#   MIDDLEWARE <middleware>
#   [MESSAGE
#     [SOURCES <message-source-templates>]
#     [HEADERS <message-header-templates>] ]
#   [SERVICE
#     [SOURCES <service-source-templates>]
#     [HEADERS <service-header-templates>] ]
#   [DEPENDENCIES <middleware dependencies>]
# )
function(is_mix_install_extension)

  cmake_parse_arguments(
    _ARG # prefix
    "" # options
    "IDL_TYPE;MIDDLEWARE" # one-value arguments
    "MESSAGE;SERVICE;DEPENDENCIES" # multi-value arguments
    ${ARGN}
  )

  cmake_parse_arguments(
    _ARG_MESSAGE
    "" # options
    "" # one-value arguments
    "SOURCES;HEADERS" # multi-value arguments
    ${_ARG_MESSAGE}
  )

  cmake_parse_arguments(
    _ARG_SERVICE
    "" # options
    "" # one-value arguments
    "SOURCES;HEADERS" # multi-value arguments
    ${_ARG_SERVICE}
  )

  set(middleware ${_ARG_MIDDLEWARE})
  foreach(transport_type MESSAGE SERVICE)
    foreach(file_type SOURCES HEADERS)
      set(${transport_type}_${file_type}_BASE)
      foreach(file ${_ARG_${transport_type}_${file_type}})
        get_filename_component(base ${file} NAME)
        list(APPEND ${transport_type}_${file_type}_BASE ${base})
      endforeach()
    endforeach()
  endforeach()

  set(config_output "${CMAKE_BINARY_DIR}/is/${_ARG_IDL_TYPE}/is-${_ARG_IDL_TYPE}-${middleware}-mixConfig.cmake")
  configure_file(
    ${IS_IDL_MIDDLEWARE_MIX_CONFIG_TEMPLATE}
    ${config_output}
    @ONLY
  )

  set(extension_output "${CMAKE_BINARY_DIR}/is/${_ARG_IDL_TYPE}/is-${_ARG_IDL_TYPE}-${middleware}-mix-extension.cmake")
  configure_file(
    ${IS_IDL_MIDDLEWARE_MIX_EXTENSION_TEMPLATE}
    ${extension_output}
    @ONLY
  )

  set(base_install_dir "${CMAKE_INSTALL_PREFIX}/../is-${middleware}/share/is-${_ARG_IDL_TYPE}-${middleware}-mix")
  file(
    COPY
      ${config_output}
      ${extension_output}
    DESTINATION
      ${base_install_dir}
  )

  set(template_install_dir "${base_install_dir}/templates")
  file(
    COPY
      ${_ARG_MESSAGE_SOURCES}
      ${_ARG_MESSAGE_HEADERS}
      ${_ARG_SERVICE_SOURCES}
      ${_ARG_SERVICE_HEADERS}
    DESTINATION
      ${template_install_dir}
  )

endfunction()
