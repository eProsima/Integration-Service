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

#include <catch2/catch.hpp>

#include <iostream>

namespace {
void run_test_case(
    const std::string& initial_topic,
    const std::string& name,
    const uint32_t number,
    const std::string& suffix)
{
  const std::string topic =
      initial_topic + "/" + name + "_" + std::to_string(number) + suffix;

  std::cout << "Testing topic [" << topic << "]" << std::endl;

  std::promise<soss::Message> message_promise;
  auto message_future = message_promise.get_future();
  REQUIRE(soss::mock::subscribe(
          topic, [&](const soss::Message& incoming_message)
  {
    message_promise.set_value(incoming_message);
  }));

  soss::Message message;
  message.type = "websocket_test/Dispatch";
  message.data["name"] = soss::Convert<std::string>::make_soss_field(name);
  message.data["number"] = soss::Convert<uint32_t>::make_soss_field(number);

  soss::mock::publish_message(initial_topic, message);

  using namespace std::chrono_literals;
  REQUIRE(message_future.wait_for(5s) == std::future_status::ready);
  const soss::Message result = message_future.get();
  REQUIRE(result.data.size() > 0);

  auto it = result.data.begin();

  std::string result_name;
  soss::Convert<std::string>::from_soss_field(it++, result_name);
  CHECK(result_name == name);

  uint32_t result_number;
  soss::Convert<uint32_t>::from_soss_field(it++, result_number);
  CHECK(result_number == number);
}
} // anonymous namespace

TEST_CASE("Transmit and dispatch messages", "[websocket]")
{
  using namespace std::chrono_literals;

  soss::InstanceHandle handle =
      soss::run_instance(WEBSOCKET__DISPATCH__TEST_CONFIG);
  REQUIRE(handle);

  std::cout << " -- Waiting to make sure the client has time to connect"
            << std::endl;
  std::this_thread::sleep_for(5s);
  std::cout << " -- Done waiting!" << std::endl;

  run_test_case("dispatch_into_client", "apple", 1, "/topic");
  run_test_case("dispatch_into_client", "banana", 2, "/topic");
  run_test_case("dispatch_into_client", "cherry", 3, "/topic");

  run_test_case("dispatch_into_server", "avocado", 10, "");
  run_test_case("dispatch_into_server", "blueberry", 20, "");
  run_test_case("dispatch_into_server", "citrus", 30, "");

  CHECK(handle.quit().wait() == 0);

  // NOTE(MXG) It seems the error
  // `[info] asio async_shutdown error: asio.misc:2 (End of file)`
  // is normal and to be expected as far as I can tell.
}
