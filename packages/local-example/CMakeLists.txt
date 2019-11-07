cmake_minimum_required(VERSION 3.5.0)

project(soss-local-example)

find_package(xtypes REQUIRED)
find_package(soss-core REQUIRED)
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
)

########################################################################
# Test
set(TEST_NAME ${PROJECT_NAME}-test)
add_executable(${TEST_NAME}
    test/main.cpp
)

set_target_properties(${TEST_NAME}
    PROPERTIES
        CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

target_compile_options(${TEST_NAME}
    PRIVATE
        -Werror -Wall -Wextra -Wpedantic
)

target_link_libraries(${TEST_NAME}
    PRIVATE
        soss::mock
)

########################################################################
# Installation
soss_install_middleware_plugin(
    MIDDLEWARE local-example
    TARGET ${PROJECT_NAME}
)

# Install test
install(
    TARGETS ${TEST_NAME}
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
    COMPONENT ${PROJECT_NAME}
)

install(
    FILES ${PROJECT_SOURCE_DIR}/test/test_config.yaml
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT ${PROJECT_NAME}
    )

