cmake_minimum_required(VERSION 3.5.0)

project(soss-core)

if(NOT SOSS_CONFIGURE_AS_SUBPROJECT)

  if(NOT CMAKE_CXX_STANDARD)
    # TODO(MXG): Remove this block and use target_compile_features(~)
    # instead when we no longer need to support Ubuntu 16.04.
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  include(GNUInstallDirs)

endif()

find_package(fastrtps REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS
    program_options
)

###############################
# Configure soss-core library
add_library(soss-core SHARED
  src/Config.cpp
  #src/FieldToString.cpp
  src/Instance.cpp
  #src/Message.cpp
  src/MiddlewareInterfaceExtension.cpp
  src/register_system.cpp
  src/Search.cpp
  #src/StringTemplate.cpp
)

# Generate the export macro header
include(cmake/soss_generate_export_header.cmake)
soss_generate_export_header(core)

target_link_libraries(soss-core
  PUBLIC
    yaml-cpp
  PRIVATE
    Boost::program_options
    $<$<PLATFORM_ID:Linux>:dl>
    $<$<PLATFORM_ID:Linux>:stdc++fs>
)

target_include_directories(soss-core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_definitions(soss-core
  PRIVATE
    SOSS_LIBRARY_ARCHITECTURE="${CMAKE_LIBRARY_ARCHITECTURE}"
)

###############################
# Configure soss executable
add_executable(soss
  src/main.cpp
)

target_link_libraries(soss
  PRIVATE
    soss-core
)

###############################
# Install soss-core
set(soss_core_config_dir "${CMAKE_INSTALL_LIBDIR}/cmake/soss-core")

install(
  TARGETS soss-core
  EXPORT  soss-core
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  COMPONENT soss-core
)

install(
  EXPORT soss-core
  DESTINATION ${soss_core_config_dir}
  FILE soss-core-target.cmake
  NAMESPACE soss::
  COMPONENT soss-core
)

install(
  DIRECTORY   "${CMAKE_CURRENT_LIST_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT soss-core
)

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/soss-coreConfig.cmake"
  DESTINATION "${soss_core_config_dir}"
  COMPONENT soss-core
)

if (WIN32)
  install(
    FILES "${CMAKE_CURRENT_LIST_DIR}/windows/local_setup_windows.bat"
    DESTINATION "${CMAKE_INSTALL_PREFIX}/../"
    COMPONENT soss-core
  )
endif()

###############################
# Install soss program
install(
  TARGETS soss soss-core
  DESTINATION "${CMAKE_INSTALL_BINDIR}"
  COMPONENT soss-core
)

###############################
# Install core middleware extension development tools
install(
  DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/cmake"
  DESTINATION "${soss_core_config_dir}"
  COMPONENT soss-core
)

install(
  DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/templates"
  DESTINATION "${soss_core_config_dir}"
  COMPONENT soss-core
)

###############################
# Add testing subdirectory
add_subdirectory(test)
