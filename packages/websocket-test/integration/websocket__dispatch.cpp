/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <is/sh/mock/api.hpp>
#include <is/core/Instance.hpp>
#include <is/utils/Convert.hpp>

#include <catch2/catch.hpp>

#include <iostream>

namespace {
void run_test_case(
        eprosima::is::core::InstanceHandle& handle,
        const std::string& initial_topic,
        const std::string& name,
        const uint32_t number,
        const std::string& suffix)
{
    const std::string topic =
            initial_topic + "/" + name + "_" + std::to_string(number) + suffix;

    std::cout << "Testing topic [" << topic << "]" << std::endl;

    std::promise<eprosima::xtypes::DynamicData> message_promise;
    auto message_future = message_promise.get_future();
    REQUIRE(eprosima::is::sh::mock::subscribe(
                topic, [&](const eprosima::xtypes::DynamicData& incoming_message)
                {
                    message_promise.set_value(incoming_message);
                }));

    const eprosima::is::TypeRegistry& mock_types = *handle.type_registry("mock");
    eprosima::xtypes::DynamicData message(*mock_types.at("Dispatch"));

    message["name"] = name;
    message["number"] = number;

    eprosima::is::sh::mock::publish_message(initial_topic, message);

    using namespace std::chrono_literals;
    REQUIRE(message_future.wait_for(5s) == std::future_status::ready);
    const eprosima::xtypes::DynamicData result = message_future.get();
    REQUIRE(result.size() > 0);

    std::string result_name;
    result_name = result["name"].value<std::string>();
    CHECK(result_name == name);

    uint32_t result_number;
    result_number = result["number"].value<uint32_t>();
    CHECK(result_number == number);
}

} // anonymous namespace

TEST_CASE("Transmit and dispatch messages", "[websocket]")
{
    using namespace std::chrono_literals;

    eprosima::is::core::InstanceHandle handle =
            eprosima::is::run_instance(WEBSOCKET__DISPATCH__TEST_CONFIG);
    REQUIRE(handle);

    std::cout << " -- Waiting to make sure the client has time to connect"
              << std::endl;
    std::this_thread::sleep_for(5s);
    std::cout << " -- Done waiting!" << std::endl;

    run_test_case(handle, "dispatch_into_client", "apple", 1, "/topic");
    run_test_case(handle, "dispatch_into_client", "banana", 2, "/topic");
    run_test_case(handle, "dispatch_into_client", "cherry", 3, "/topic");

    run_test_case(handle, "dispatch_into_server", "avocado", 10, "");
    run_test_case(handle, "dispatch_into_server", "blueberry", 20, "");
    run_test_case(handle, "dispatch_into_server", "citrus", 30, "");

    CHECK(handle.quit().wait() == 0);

    // NOTE(MXG) It seems the error
    // `[info] asio async_shutdown error: asio.misc:2 (End of file)`
    // is normal and to be expected as far as I can tell.
}
