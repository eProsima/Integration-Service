cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

project(is-core)

if(NOT is_CONFIGURE_AS_SUBPROJECT)

  if(NOT CMAKE_CXX_STANDARD)
    # TODO(MXG): Remove this block and use target_compile_features(~)
    # instead when we no longer need to support Ubuntu 16.04.
    set(CMAKE_CXX_STANDARD 17)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  include(GNUInstallDirs)

endif()

find_package(xtypes REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    program_options
)

###############################
# Configure is-core library
add_library(${PROJECT_NAME}
  SHARED
    src/Config.cpp
    src/FieldToString.cpp
    src/Instance.cpp
    src/MiddlewareInterfaceExtension.cpp
    src/register_system.cpp
    src/Search.cpp
    src/StringTemplate.cpp
)

# Generate the export macro header
include(cmake/is_generate_export_header.cmake)
is_generate_export_header(core)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    yaml-cpp
    xtypes
  PRIVATE
    Boost::program_options
    $<$<PLATFORM_ID:Linux>:dl>
    $<$<PLATFORM_ID:Linux>:stdc++fs>
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    #$<INSTALL_INTERFACE:${xtypes_INCLUDE_DIR}>  #propagate the xtypes headers
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    IS_LIBRARY_ARCHITECTURE="${CMAKE_LIBRARY_ARCHITECTURE}"
)

###############################
# Configure the Integration Service executable
add_executable(integration-service
  src/main.cpp
)

target_link_libraries(integration-service
  PRIVATE
    ${PROJECT_NAME}
)

###############################
# Install is-core
set(is_core_config_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  COMPONENT ${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}
  DESTINATION ${is_core_config_dir}
  FILE ${PROJECT_NAME}-target.cmake
  NAMESPACE is::
  COMPONENT ${PROJECT_NAME}
)

install(
  DIRECTORY   "${CMAKE_CURRENT_LIST_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT ${PROJECT_NAME}
)

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${is_core_config_dir}"
  COMPONENT ${PROJECT_NAME}
)

if (WIN32)
  install(
    FILES "${CMAKE_CURRENT_LIST_DIR}/windows/local_setup_windows.bat"
    DESTINATION "${CMAKE_INSTALL_PREFIX}/../"
    COMPONENT ${PROJECT_NAME}
  )
endif()

###############################
# Install Integration Service program and core library
install(
  TARGETS
  integration-service ${PROJECT_NAME}
  DESTINATION "${CMAKE_INSTALL_BINDIR}"
  COMPONENT ${PROJECT_NAME}
)

###############################
# Install core middleware extension development tools
install(
  DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/cmake"
  DESTINATION "${is_core_config_dir}"
  COMPONENT ${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/templates"
  DESTINATION "${is_core_config_dir}"
  COMPONENT ${PROJECT_NAME}
)

###############################
# Add testing subdirectory
add_subdirectory(test)
