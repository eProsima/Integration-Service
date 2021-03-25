
if(NOT BUILD_TESTING)
  return()
endif()

include(CTest)
include(${PROJECT_SOURCE_DIR}/../core/cmake/common/gtest.cmake)
enable_testing()

add_executable(soss-core-test
  unit/search_test.cpp
)

target_link_libraries(soss-core-test
  PRIVATE
    soss-core
  PUBLIC
    libgtest
)

target_include_directories(soss-core-test
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../src"
)

add_gtest(soss-core-test SOURCES unit/search_test.cpp)

set(mock_config_directory "${PROJECT_BINARY_DIR}/mock/config")
set(mock_file_name "path/to/some_file.txt")
set(mock_file_path "${mock_config_directory}/${mock_file_name}")
file(WRITE "${mock_file_path}" "intentionally blank")

target_compile_definitions(soss-core-test
  PRIVATE
    "SEARCH_TEST__MOCK_CONFIG_DIRECTORY=\"${mock_config_directory}\""
    "SEARCH_TEST__MOCK_FILE_NAME=\"${mock_file_name}\""
    "SEARCH_TEST__MOCK_FILE_PATH=\"${mock_file_path}\""
)
