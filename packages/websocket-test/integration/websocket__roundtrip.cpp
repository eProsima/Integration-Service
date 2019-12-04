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

TEST_CASE("Transmit and receive all test messages", "[websocket]")
{
  using namespace std::chrono_literals;

  soss::InstanceHandle server_handle =
      soss::run_instance(WEBSOCKET__ROUNDTRIP_SERVER__TEST_CONFIG);
  REQUIRE(server_handle);

  soss::InstanceHandle client_handle =
      soss::run_instance(WEBSOCKET__ROUNDTRIP_CLIENT__TEST_CONFIG);
  REQUIRE(client_handle);

  std::cout << " -- Waiting to make sure the client has time to connect"
            << std::endl;
  std::this_thread::sleep_for(5s);
  std::cout << " -- Done waiting!" << std::endl;

  std::promise<xtypes::DynamicData> client_to_server_promise;
  // Note: The public API of soss::mock can only publish/subscribe into the
  // soss. A soss::mock subscription will never receive a soss::mock publication
  // from soss::mock::publish_message(~), so any messages that this subscription
  // receives will have come from the server_handle.
  REQUIRE(soss::mock::subscribe(
            "client_to_server",
            [&](const xtypes::DynamicData& message)
  {
    client_to_server_promise.set_value(message);
  }));

  const soss::TypeRegistry& mock_types = *client_handle.type_registry("mock");
  xtypes::DynamicData msg_to_server(*mock_types.at("ClientToServer"));

  const float apple = 2.3f;
  msg_to_server["apple"] = apple;
  soss::mock::publish_message("client_to_server", msg_to_server);

  auto client_to_server_future = client_to_server_promise.get_future();
  REQUIRE(client_to_server_future.wait_for(5s) == std::future_status::ready);
  const xtypes::DynamicData client_to_server_result = client_to_server_future.get();
  REQUIRE(client_to_server_result.size() > 0);

  float apple_result = client_to_server_result["apple"];
  CHECK(apple_result == apple);


  std::promise<xtypes::DynamicData> server_to_client_promise;
  REQUIRE(soss::mock::subscribe(
            "server_to_client",
            [&](const xtypes::DynamicData& message)
  {
    server_to_client_promise.set_value(message);
  }));

  const soss::TypeRegistry& server_types = *server_handle.type_registry("mock");
  xtypes::DynamicData msg_to_client(*mock_types.at("ServerToClient"));

  const std::string banana = "here is a banana";
  msg_to_client["banana"] = banana;
  soss::mock::publish_message("server_to_client", msg_to_client);

  auto server_to_client_future = server_to_client_promise.get_future();
  REQUIRE(server_to_client_future.wait_for(5s) == std::future_status::ready);
  const xtypes::DynamicData server_to_client_result = server_to_client_future.get();
  REQUIRE(server_to_client_result.size() > 0);

  std::string banana_result;
  banana_result = server_to_client_result["banana"].value<std::string>();
  CHECK(banana_result == banana);

  CHECK(client_handle.quit().wait() == 0);
  CHECK(server_handle.quit().wait() == 0);

  // NOTE(MXG) It seems the error
  // `[info] asio async_shutdown error: asio.misc:2 (End of file)`
  // is normal and to be expected as far as I can tell.
}
