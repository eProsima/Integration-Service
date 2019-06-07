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

  const YAML::Node server_config_node =
      YAML::LoadFile(WEBSOCKET__ROUNDTRIP_SERVER__TEST_CONFIG);
  soss::InstanceHandle server_handle = soss::run_instance(server_config_node);
  REQUIRE(server_handle);

  const YAML::Node client_config_node =
      YAML::LoadFile(WEBSOCKET__ROUNDTRIP_CLIENT__TEST_CONFIG);
  soss::InstanceHandle client_handle = soss::run_instance(client_config_node);
  REQUIRE(client_handle);

  std::cout << " -- Waiting to make sure the client has time to connect"
            << std::endl;
  std::this_thread::sleep_for(5s);
  std::cout << " -- Done waiting!" << std::endl;

  std::promise<soss::Message> client_to_server_promise;
  auto client_to_server_future = client_to_server_promise.get_future();
  // Note: The public API of soss::mock can only publish/subscribe into the
  // soss. A soss::mock subscription will never receive a soss::mock publication
  // from soss::mock::publish_message(~), so any messages that this subscription
  // receives will have come from the server_handle.
  REQUIRE(soss::mock::subscribe(
            "client_to_server",
            [&](const soss::Message& message)
  {
    client_to_server_promise.set_value(message);
  }));

  soss::Message msg_to_server;
  msg_to_server.type = "websocket_test/ClientToServer";
  const float apple = 2.3f;
  msg_to_server.data["apple"] = soss::Convert<float>::make_soss_field(apple);
  soss::mock::publish_message("client_to_server", msg_to_server);

  REQUIRE(client_to_server_future.wait_for(5s) == std::future_status::ready);
  const soss::Message client_to_server_result = client_to_server_future.get();
  REQUIRE(client_to_server_result.data.size() > 0);

  float apple_result;
  soss::Convert<float>::from_soss_field(
        client_to_server_result.data.begin(), apple_result);
  CHECK(apple_result == apple);


  std::promise<soss::Message> server_to_client_promise;
  auto server_to_client_future = server_to_client_promise.get_future();
  REQUIRE(soss::mock::subscribe(
            "server_to_client",
            [&](const soss::Message& message)
  {
    server_to_client_promise.set_value(message);
  }));

  soss::Message msg_to_client;
  msg_to_client.type = "websocket_test/ServerToClient";

  const std::string banana = "here is a banana";
  msg_to_client.data["banana"] =
      soss::Convert<std::string>::make_soss_field(banana);
  soss::mock::publish_message("server_to_client", msg_to_client);

  REQUIRE(server_to_client_future.wait_for(5s) == std::future_status::ready);
  const soss::Message server_to_client_result = server_to_client_future.get();
  REQUIRE(server_to_client_result.data.size() > 0);

  std::string banana_result;
  soss::Convert<std::string>::from_soss_field(
        server_to_client_result.data.begin(), banana_result);
  CHECK(banana_result == banana);

  CHECK(client_handle.quit().wait() == 0);
  CHECK(server_handle.quit().wait() == 0);

  // NOTE(MXG) It seems the error
  // `[info] asio async_shutdown error: asio.misc:2 (End of file)`
  // is normal and to be expected as far as I can tell.
}
