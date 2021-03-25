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

#include <soss/mock/api.hpp>
#include <soss/Instance.hpp>
#include <soss/utilities.hpp>

#include <gtest/gtest.h>

#include <iostream>

namespace {
void run_test_case(
        soss::InstanceHandle& handle,
        const std::string& initial_topic,
        const std::string& name,
        const uint32_t number,
        const std::string& suffix)
{
    const std::string topic =
            initial_topic + "/" + name + "_" + std::to_string(number) + suffix;

    std::cout << "Testing topic [" << topic << "]" << std::endl;

    std::promise<xtypes::DynamicData> message_promise;
    auto message_future = message_promise.get_future();
    ASSERT_TRUE(soss::mock::subscribe(
                topic, [&](const xtypes::DynamicData& incoming_message)
                {
                    message_promise.set_value(incoming_message);
                }));

    const soss::TypeRegistry& mock_types = *handle.type_registry("mock");
    xtypes::DynamicData message(*mock_types.at("Dispatch"));

    message["name"] = name;
    message["number"] = number;

    soss::mock::publish_message(initial_topic, message);

    using namespace std::chrono_literals;
    ASSERT_EQ(message_future.wait_for(5s), std::future_status::ready);
    const xtypes::DynamicData result = message_future.get();
    ASSERT_GT(result.size(), 0);

    std::string result_name;
    result_name = result["name"].value<std::string>();
    EXPECT_EQ(result_name, name);

    uint32_t result_number;
    result_number = result["number"].value<uint32_t>();
    EXPECT_EQ(result_number, number);
}

} // anonymous namespace

TEST(WebSocket, Transmit_and_dispatch_messages)
{
    using namespace std::chrono_literals;

    soss::InstanceHandle handle =
            soss::run_instance(WEBSOCKET__DISPATCH__TEST_CONFIG);
    ASSERT_TRUE(handle);

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

    EXPECT_EQ(handle.quit().wait(), 0);

    // NOTE(MXG) It seems the error
    // `[info] asio async_shutdown error: asio.misc:2 (End of file)`
    // is normal and to be expected as far as I can tell.
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}