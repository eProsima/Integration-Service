
include(CTest)
if(NOT BUILD_TESTING)
  return()
endif()

add_executable(soss-core-test
  main.cpp
  unit/search_test.cpp
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../../thirdparty")

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")
include(Catch)
catch_discover_tests(soss-core-test)

target_link_libraries(soss-core-test
  PRIVATE
    soss-core
)

target_include_directories(soss-core-test
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../src"
    "${thirdparty_dir}/catch2/include"
)

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
