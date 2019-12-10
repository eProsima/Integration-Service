cmake_minimum_required(VERSION 3.5.0)

project(soss-echo)

find_package(xtypes REQUIRED)
find_package(soss-core REQUIRED)
find_package(soss-json REQUIRED)
find_package(soss-mock REQUIRED)

########################################################################
# Library
add_library(${PROJECT_NAME} SHARED
    src/SystemHandle.cpp
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
       CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
       -Werror -Wall -Wextra -Wpedantic
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        soss::core
        soss::json
)

########################################################################
# Test
set(EXAMPLE_NAME ${PROJECT_NAME}-example)
add_executable(${EXAMPLE_NAME}
    example/main.cpp
)

set_target_properties(${EXAMPLE_NAME}
    PROPERTIES
        CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

target_compile_options(${EXAMPLE_NAME}
    PRIVATE
        -Werror -Wall -Wextra -Wpedantic
)

target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
        soss::mock
)

########################################################################
# Installation
soss_install_middleware_plugin(
    MIDDLEWARE echo
    TARGET ${PROJECT_NAME}
)

# Install example
install(
    TARGETS ${EXAMPLE_NAME}
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
    COMPONENT ${PROJECT_NAME}
)

install(
    FILES ${PROJECT_SOURCE_DIR}/example/config.yaml
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT ${PROJECT_NAME}
    )

