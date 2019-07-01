cmake_minimum_required(VERSION 3.5.0)

project(soss-xtypes-example)

find_package(soss-core REQUIRED)

########################################################################
# Library
set(SRC
    src/SystemHandle.cpp
    )

add_library(${PROJECT_NAME} SHARED ${SRC})

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        -Werror -Wall -Wextra -Wpedantic
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        include
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        soss::core
)

########################################################################
# Installation
soss_install_middleware_plugin(
    MIDDLEWARE xtypes-example
    TARGET soss-xtypes-example
)

