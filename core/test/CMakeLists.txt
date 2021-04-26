if(NOT BUILD_TESTING)
  return()
endif()

# TODO (@jamoralp): this definitely needs an improvement. Also, think about more unit test for the core itself.
include(CTest)
include(${PROJECT_SOURCE_DIR}/../core/cmake/common/gtest.cmake)
enable_testing()

add_executable(is-core-test
    unit/search_test.cpp
    )

target_link_libraries(is-core-test
    PRIVATE
        is-core
    PUBLIC
        $<IF:$<BOOL:${IS_GTEST_EXTERNAL_PROJECT}>,libgtest,gtest>
    )

target_include_directories(is-core-test
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/../src"
    )

add_gtest(is-core-test SOURCES unit/search_test.cpp)

set(mock_config_directory "${PROJECT_BINARY_DIR}/mock/config")
set(mock_file_name "path/to/some_file.txt")
set(mock_file_path "${mock_config_directory}/${mock_file_name}")
file(WRITE "${mock_file_path}" "intentionally blank")

target_compile_definitions(is-core-test
    PRIVATE
        "SEARCH_TEST__MOCK_CONFIG_DIRECTORY=\"${mock_config_directory}\""
        "SEARCH_TEST__MOCK_FILE_NAME=\"${mock_file_name}\""
        "SEARCH_TEST__MOCK_FILE_PATH=\"${mock_file_path}\""
)
