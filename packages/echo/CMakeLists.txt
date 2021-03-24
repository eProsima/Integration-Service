cmake_minimum_required(VERSION 3.5.0)

project(is-echo)

find_package(xtypes REQUIRED)
find_package(is-core REQUIRED)
find_package(is-json REQUIRED)
find_package(is-mock REQUIRED)

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
        is::core
        is::json
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
        is::mock
)

########################################################################
# Installation
is_install_middleware_plugin(
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

