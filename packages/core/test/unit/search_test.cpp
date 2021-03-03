/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <Search-impl.hpp>

#include <catch2/catch.hpp>

#include <iostream>

TEST_CASE("Use config-file directory", "[search][core]")
{
    soss::Search::Implementation::set_config_file_directory(
        SEARCH_TEST__MOCK_CONFIG_DIRECTORY);

    const soss::Search no_config_dir = soss::Search("mock");
    const auto result = no_config_dir.find_file(SEARCH_TEST__MOCK_FILE_NAME);
    REQUIRE(result.empty());

    const soss::Search with_config_dir = soss::Search("mock")
            .relative_to_config();
    REQUIRE(with_config_dir.find_file(SEARCH_TEST__MOCK_FILE_NAME)
            == SEARCH_TEST__MOCK_FILE_PATH);
}
