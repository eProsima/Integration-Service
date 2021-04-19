#####################################################################################
#                     GTest dependency as cmake external project
#####################################################################################

find_package(Threads REQUIRED)

include(ExternalProject)

find_package(GTest REQUIRED)

if (NOT GTest_FOUND)

    ExternalProject_Add(gtest
        URL https://github.com/google/googletest/archive/master.zip
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
        INSTALL_COMMAND ""
        )

    ExternalProject_Get_Property(gtest source_dir binary_dir)

endif()

add_library(libgtest IMPORTED STATIC GLOBAL)
add_dependencies(libgtest gtest)

set_target_properties(libgtest
    PROPERTIES
        "IMPORTED_LOCATION" "${binary_dir}/lib/libgtest.a"
        "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
    )

include_directories("${source_dir}/googletest/include")

macro(add_gtest)
    # Parse arguments
    if("${ARGV0}" STREQUAL "NAME")
        set(uniValueArgs NAME COMMAND)
        unset(test)
        unset(command)
    else()
        set(test "${ARGV0}")
        set(command "${test}")
    endif()
    set(multiValueArgs SOURCES ENVIRONMENTS DEPENDENCIES LABELS)
    cmake_parse_arguments(GTEST "" "${uniValueArgs}" "${multiValueArgs}" ${ARGN})

    if(GTEST_NAME)
        set(test ${GTEST_NAME})
        set(command ${GTEST_COMMAND})
    endif()

    if(WIN32)
        set(WIN_PATH "$ENV{PATH}")
        get_target_property(LINK_LIBRARIES_ ${command} LINK_LIBRARIES)
        if(NOT "${LINK_LIBRARIES_}" STREQUAL "LINK_LIBRARIES_-NOTFOUND")
            foreach(LIBRARY_LINKED ${LINK_LIBRARIES_})
                if(TARGET ${LIBRARY_LINKED})
                    # Check if is a real target or a target interface
                    get_target_property(type ${LIBRARY_LINKED} TYPE)
                    if(NOT type STREQUAL "INTERFACE_LIBRARY")
                        set(WIN_PATH "$<TARGET_FILE_DIR:${LIBRARY_LINKED}>;${WIN_PATH}")
                    endif()
                    unset(type)
                endif()
            endforeach()
        endif()
        foreach(DEP ${GTEST_DEPENDENCIES})
            set(WIN_PATH "$<TARGET_FILE_DIR:${DEP}>;${WIN_PATH}")
        endforeach()
        string(REPLACE ";" "\\;" WIN_PATH "${WIN_PATH}")

    endif()

    foreach(GTEST_SOURCE_FILE ${GTEST_SOURCES})
        file(STRINGS ${GTEST_SOURCE_FILE} GTEST_TEST_NAMES REGEX ^TEST)
        foreach(GTEST_TEST_NAME ${GTEST_TEST_NAMES})
            string(REGEX REPLACE ["\) \(,"] ";" GTEST_TEST_NAME ${GTEST_TEST_NAME})
            list(GET GTEST_TEST_NAME 1 GTEST_GROUP_NAME)
            list(GET GTEST_TEST_NAME 3 GTEST_TEST_NAME)
            add_test(NAME ${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}
                COMMAND ${command} --gtest_filter=${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}:*/${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}/*)

            # Add environment
            set(GTEST_ENVIRONMENT "")
            if(WIN32)
                set(GTEST_ENVIRONMENT "PATH=${WIN_PATH}")
            endif()

            foreach(property ${GTEST_ENVIRONMENTS})
                list(APPEND GTEST_ENVIRONMENT "${property}")
            endforeach()

            if(GTEST_ENVIRONMENT)
                set_tests_properties(${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}
                    PROPERTIES ENVIRONMENT "${GTEST_ENVIRONMENT}")
            endif()
            unset(GTEST_ENVIRONMENT)

            # Add labels
            set_property(TEST ${GTEST_GROUP_NAME}.${GTEST_TEST_NAME} PROPERTY LABELS "${GTEST_LABELS}")

        endforeach()
    endforeach()
endmacro()
