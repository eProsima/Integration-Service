# Copyright 2018 Open Source Robotics Foundation, Inc.
#Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
# is_install_middleware_plugin(
#   MIDDLEWARE    <middleware_name>
#   TARGET        <target>
#   [NO_CONFIG]
#   [TYPES        <system_types...>]
#   [DEPENDENCIES <middleware dependencies>]
#   [EXTENSIONS   <files_of_config_extensions>]
#   [BUILD_DIR    <relative_directory>]
# )
#
# MIDDLEWARE: The name of the middleware. The resulting package will be called
# is-<middleware_name>
#
# TARGET: The target for the library that should be loaded for this middleware.
# TODO(MXG): See about supporting multiple libraries per middleware.
#
# INTERFACES: Optional list of interface targets required to be deployed on installation
#
# TYPES: Optional list of names of system types that this middleware supports.
# If this argument is not provided, then we will assume that the middleware only
# provides a system called <middleware_name>.
#
# NO_CONFIG: Option. Tell the function to not install the installation
# configuration information for this middleware plugin. This is strongly
# discouraged for any middlewares that may have downstream dependents.
#
# DEPENDENCIES: Optional list of package names that the middleware plugin
# depends on.
#
# EXTENSIONS: Optional list of files that will get included into the config-file
# that is generated. This can be used for adding calls to find_dependency or
# exposing cmake modules, or anything else. Each file will be installed to an
# "extensions" subfolder of the config directory.
#
# BUILD_DIR: The absolute path where the mix files for this middleware should be
# configured and where the library will be placed. If this argument is not
# specified, the default will be ${CMAKE_BINARY_DIR}/is/<middleware-name>/lib.
include(GNUInstallDirs)

function(is_install_middleware_plugin)

  include(CMakeParseArguments)
  cmake_parse_arguments(
    _ARG # prefix
    "NO_CONFIG" # options
    "MIDDLEWARE;TARGET;BUILD_DIR" # one-value arguments
    "INTERFACES;TYPES;DEPENDENCIES;EXTENSIONS" # multi-value arguments
    ${ARGN}
  )

  set(middleware ${_ARG_MIDDLEWARE})
  set(plugin_library_target ${_ARG_TARGET})
  string (REPLACE ";" " " interfaces "${_ARG_INTERFACES}")

  if(NOT _ARG_TYPES)
    set(system_types ${middleware})
  else()
    set(system_types ${_ARG_TYPES})
  endif()

  install(
    TARGETS ${plugin_library_target} ${interfaces}
    EXPORT  ${plugin_library_target}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT ${plugin_library_target}
  )

  if(_ARG_BUILD_DIR)
    set(mix_build_dir "${_ARG_BUILD_DIR}")
  else()
    set(mix_build_dir "${CMAKE_BINARY_DIR}/is/${middleware}/lib")
  endif()

  set_target_properties(${plugin_library_target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${mix_build_dir}
  )

  set(plugin_library_extension $<IF:$<PLATFORM_ID:Windows>,"dll","dl">)

  foreach(type ${system_types})
    set(plugin_library_mix_template "${mix_build_dir}/is/${type}/${type}.mix.gen")
    set(plugin_library_directory "../..")
    configure_file(
      "${IS_TEMPLATE_DIR}/plugin_library.mix.in"
      "${plugin_library_mix_template}"
      @ONLY
    )

    set(plugin_library_mix "${mix_build_dir}/is/${type}/${type}.mix")
    set(plugin_library_install_mix "${CMAKE_INSTALL_PREFIX}/lib/is/${type}/${type}.mix")

    file(GENERATE
      OUTPUT ${plugin_library_mix}
      INPUT  ${plugin_library_mix_template}
    )

    file(GENERATE
      OUTPUT ${plugin_library_install_mix}
      INPUT  ${plugin_library_mix_template}
    )

  endforeach()

  if(NOT _ARG_NO_CONFIG)

    set(config_install_dir ${CMAKE_INSTALL_LIBDIR}/cmake/is-${middleware})

    set(extensions)
    foreach(extension ${_ARG_EXTENSIONS})
      get_filename_component(ext_filename ${extension} NAME)
      list(APPEND extensions ${ext_filename})
    endforeach()

    export(
      TARGETS
        ${plugin_library_target}
        ${interfaces}
      FILE
        ${CMAKE_INSTALL_PREFIX}/lib/cmake/is-${middleware}/${plugin_library_target}-target.cmake
      NAMESPACE is::
    )

    include(CMakePackageConfigHelpers)
    set(config_file_input "${IS_TEMPLATE_DIR}/middleware-config.cmake.in")
    set(config_file_output "${mix_build_dir}/is-${middleware}Config.cmake")
    configure_package_config_file(
      ${config_file_input}
      ${config_file_output}
      INSTALL_DESTINATION ${config_install_dir}
    )

    file(
      COPY
        ${config_file_output}
      DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib/cmake/is-${middleware}
    )

    if(_ARG_EXTENSIONS)
      install(
        FILES ${_ARG_EXTENSIONS}
        DESTINATION ${config_install_dir}/extensions
        COMPONENT ${plugin_library_target}
      )
    endif()

  endif()

endfunction()
