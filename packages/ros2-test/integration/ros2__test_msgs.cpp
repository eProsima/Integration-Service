/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <soss/mock/api.hpp>
#include <soss/Instance.hpp>
#include <soss/utilities.hpp>

#include <nav_msgs/srv/get_plan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <test_msgs/msg/basic_types.hpp>
#include <test_msgs/msg/arrays.hpp>

#include <yaml-cpp/yaml.h>

#include <catch2/catch.hpp>

#include <random>

using Catch::Matchers::WithinAbs;

#ifdef RCLCPP__QOS_HPP_
test_msgs::msg::BasicTypes generate_random_basic_types(const std::size_t seed)
{
  using test_msgs::msg::BasicTypes;

  std::mt19937 rng;
  // Use consistent seeds for deterministic test results
  rng.seed(64 + seed);

  BasicTypes basic_types;

  // uniform_int_distribution is only compatible with short, int, long, long long,
  // unsigned short, unsigned int, unsigned long, or unsigned long long. For smaller
  // values use int16_t (short).
  basic_types.bool_value = std::uniform_int_distribution<int16_t>(0, 2)(rng);
  basic_types.byte_value = static_cast<BasicTypes::_byte_value_type>(std::uniform_int_distribution<uint16_t>()(rng));
  basic_types.char_value = static_cast<BasicTypes::_char_value_type>(std::uniform_int_distribution<uint16_t>()(rng));
  basic_types.int8_value =
          static_cast<BasicTypes::_int8_value_type>(
              std::uniform_int_distribution<BasicTypes::_int16_value_type>()(rng));
  basic_types.int16_value = std::uniform_int_distribution<BasicTypes::_int16_value_type>()(rng);
  basic_types.int32_value = std::uniform_int_distribution<BasicTypes::_int32_value_type>()(rng);
  basic_types.int64_value = std::uniform_int_distribution<BasicTypes::_int64_value_type>()(rng);
  basic_types.uint8_value = static_cast<BasicTypes::_uint8_value_type>(
                                std::uniform_int_distribution<BasicTypes::_uint16_value_type>()(rng));
  basic_types.uint16_value = std::uniform_int_distribution<BasicTypes::_uint16_value_type>()(rng);
  basic_types.uint32_value = std::uniform_int_distribution<BasicTypes::_uint32_value_type>()(rng);
  basic_types.uint64_value = std::uniform_int_distribution<BasicTypes::_uint64_value_type>()(rng);
  basic_types.float32_value = std::uniform_real_distribution<BasicTypes::_float32_value_type>()(rng);
  basic_types.float64_value = std::uniform_real_distribution<BasicTypes::_float64_value_type>()(rng);

  return basic_types;
}

soss::Message generate_random_basic_types_msg(const std::size_t seed)
{
  using namespace test_msgs::msg;

  std::mt19937 rng;
  // Use consistent seeds for deterministic test results
  rng.seed(64 + seed);

  const auto basic_types = generate_random_basic_types(seed);
  soss::Message msg;
#define SOSS_SET_FIELD(field_name) \
  msg.data[#field_name] = soss::Convert<BasicTypes::_ ## field_name ## _type>::make_soss_field(basic_types.field_name);

  SOSS_SET_FIELD(bool_value);
  SOSS_SET_FIELD(byte_value);
  SOSS_SET_FIELD(char_value);
  SOSS_SET_FIELD(int8_value);
  SOSS_SET_FIELD(int16_value);
  SOSS_SET_FIELD(int32_value);
  SOSS_SET_FIELD(int64_value);
  SOSS_SET_FIELD(uint8_value);
  SOSS_SET_FIELD(uint16_value);
  SOSS_SET_FIELD(uint32_value);
  SOSS_SET_FIELD(uint64_value);
  SOSS_SET_FIELD(float32_value);
  SOSS_SET_FIELD(float64_value);

  return msg;
}

test_msgs::msg::Arrays generate_random_array(std::size_t N)
{
  REQUIRE(N <= 3);
  test_msgs::msg::Arrays msg;
  for(std::size_t i=0; i < N; ++i)
    msg.basic_types_values[i] = generate_random_basic_types(i);

  return msg;
}

TEST_CASE("Transmit and receive all test messages", "[ros2]")
{
  using namespace std::chrono_literals;

  YAML::Node config_node = YAML::LoadFile(ROS2__TEST_MSGS__TEST_CONFIG);

  soss::InstanceHandle handle = soss::run_instance(
        config_node, {ROS2__ROSIDL__BUILD_DIR});

  REQUIRE(handle);

  rclcpp::Node::SharedPtr ros2 = std::make_shared<rclcpp::Node>("ros2_test");
  rclcpp::executors::SingleThreadedExecutor executor;

  REQUIRE( rclcpp::ok() );

  SECTION("Send a bounded array of a message containing basic_types and see that it arrives correctly ")
  {
    using namespace test_msgs::msg;

    const auto publisher =
        ros2->create_publisher<Arrays>("transmit_arrays", rclcpp::SystemDefaultsQoS());
    REQUIRE(publisher);

    std::promise<soss::Message> msg_promise;
    std::future<soss::Message> msg_future = msg_promise.get_future();
    bool is_msg_received = false;
    std::mutex mock_sub_mutex;
    auto mock_sub = [&](const soss::Message& msg)
    {
      std::unique_lock<std::mutex> lock(mock_sub_mutex);
      if(is_msg_received)
        return;

      is_msg_received = true;
      msg_promise.set_value(msg);
    };
    REQUIRE(soss::mock::subscribe("transmit_arrays", mock_sub));

    const std::size_t N = 3;
    Arrays ros2_msg = generate_random_array(N);

    publisher->publish(ros2_msg);

    executor.spin_node_some(ros2);

    // Keep spinning while we wait for the promise to be delivered. Try cycle
    // this for no more than a few seconds. If it's not finished by that time,
    // then something is probably broken with the test or with soss, and we
    // should quit instead of waiting for the future and potentially hanging
    // forever.
    auto start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);
      if(msg_future.wait_for(100ms) == std::future_status::ready)
        break;

      publisher->publish(ros2_msg);
    }

    REQUIRE(msg_future.wait_for(0s) == std::future_status::ready);
    soss::Message received_msg = msg_future.get();

    CHECK(received_msg.type == "test_msgs/Arrays");

    std::vector<soss::Message>* data =
        received_msg.data["basic_types_values"]
        .cast<std::vector<soss::Message>>();
    REQUIRE(data);
    REQUIRE(data->size() == N);

    for(std::size_t i = 0; i < N; ++i)
    {
      const soss::Message& soss_basic_types = (*data)[i];
      const BasicTypes& ros2_basic_types = ros2_msg.basic_types_values[i];

      #define SOSS_REQUIRE_AND_COMPARE(field_name) { \
        const auto field_it = soss_basic_types.data.find(#field_name); \
        REQUIRE(field_it != soss_basic_types.data.end()); \
        BasicTypes::_ ## field_name ## _type field; \
        soss::Convert<BasicTypes::_ ## field_name ## _type>::from_soss_field(field_it, field); \
        CHECK(field == ros2_basic_types.field_name); }

      SOSS_REQUIRE_AND_COMPARE(bool_value);
      SOSS_REQUIRE_AND_COMPARE(byte_value);
      SOSS_REQUIRE_AND_COMPARE(char_value);
      SOSS_REQUIRE_AND_COMPARE(int8_value);
      SOSS_REQUIRE_AND_COMPARE(int16_value);
      SOSS_REQUIRE_AND_COMPARE(int32_value);
      SOSS_REQUIRE_AND_COMPARE(int64_value);
      SOSS_REQUIRE_AND_COMPARE(uint8_value);
      SOSS_REQUIRE_AND_COMPARE(uint16_value);
      SOSS_REQUIRE_AND_COMPARE(uint32_value);
      SOSS_REQUIRE_AND_COMPARE(uint64_value);
      SOSS_REQUIRE_AND_COMPARE(float32_value);
      SOSS_REQUIRE_AND_COMPARE(float64_value);
    }

    const std::size_t M = ros2_msg.basic_types_values.max_size() + 2;
    data->resize(M);
    // Add entries beyond the limits of the bounded vector so that we can make
    // sure the message conversion is robust to overflow.
    for(std::size_t i = N; i < M; ++i)
      (*data)[i] = generate_random_basic_types_msg(445+i);

    bool promise_sent = false;
    std::promise<Arrays> array_promise;
    auto array_future = array_promise.get_future();
    std::mutex echo_sub_mutex;
    auto echo_sub = [&](Arrays::UniquePtr msg)
    {
      std::unique_lock<std::mutex> lock(echo_sub_mutex);
      // promises will throw an exception if set_value(~) is called more than
      // once, so we'll guard against that.
      if(promise_sent)
        return;

      promise_sent = true;
      array_promise.set_value(*msg);
    };

    const auto subscriber = ros2->create_subscription<Arrays>(
          "echo_arrays", rclcpp::SystemDefaultsQoS(), echo_sub);

    // Keep spinning and publishing while we wait for the promise to be
    // delivered. Try cycle this for no more than a few seconds. If it's not
    // finished by that time, then something is probably broken with the test or
    // with soss, and we should quit instead of waiting for the future and
    // potentially hanging forever.
    start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);

      soss::mock::publish_message("echo_arrays", received_msg);

      executor.spin_node_some(ros2);
      if(array_future.wait_for(100ms) == std::future_status::ready)
        break;
    }

    REQUIRE(array_future.wait_for(0s) == std::future_status::ready);
    Arrays received_array = array_future.get();

    // Check that the arrays match for the first N entries
    for(std::size_t i=0; i < N; ++i)
      CHECK(received_array.basic_types_values[i] == ros2_msg.basic_types_values[i]);

    CHECK(received_array.basic_types_values.size()
          == ros2_msg.basic_types_values.max_size());
  }

  // Quit and wait for no more than a minute. We don't want the test to get
  // hung here indefinitely in the case of an error.
  handle.quit().wait_for(1min);

  // Require that it's no longer running. If it is still running, then it is
  // probably stuck, and we should forcefully quit.
  REQUIRE(!handle.running());
  REQUIRE(handle.wait() == 0);
}
#else

#include <test_msgs/msg/bounded_array_nested.hpp>

test_msgs::msg::Primitives generate_random_primitives(const std::size_t seed)
{
  using namespace test_msgs::msg;

  std::mt19937 rng;
  // Use consistent seeds for deterministic test results
  rng.seed(64+seed);

  Primitives primitives;
  primitives.bool_value = std::uniform_int_distribution<int8_t>(0, 2)(rng);
  primitives.byte_value = std::uniform_int_distribution<Primitives::_byte_value_type>()(rng);
  primitives.char_value = std::uniform_int_distribution<Primitives::_char_value_type>()(rng);
  primitives.int8_value = std::uniform_int_distribution<Primitives::_int8_value_type>()(rng);
  primitives.int16_value = std::uniform_int_distribution<Primitives::_int16_value_type>()(rng);
  primitives.int32_value = std::uniform_int_distribution<Primitives::_int32_value_type>()(rng);
  primitives.int64_value = std::uniform_int_distribution<Primitives::_int64_value_type>()(rng);
  primitives.uint8_value = std::uniform_int_distribution<Primitives::_uint8_value_type>()(rng);
  primitives.uint16_value = std::uniform_int_distribution<Primitives::_uint16_value_type>()(rng);
  primitives.uint32_value = std::uniform_int_distribution<Primitives::_uint32_value_type>()(rng);
  primitives.uint64_value = std::uniform_int_distribution<Primitives::_uint64_value_type>()(rng);
  primitives.float32_value = std::uniform_real_distribution<Primitives::_float32_value_type>()(rng);
  primitives.float64_value = std::uniform_real_distribution<Primitives::_float64_value_type>()(rng);
  primitives.string_value = std::to_string(std::uniform_int_distribution<int32_t>(0, 100)(rng));

  return primitives;
}

soss::Message generate_random_primitives_msg(const std::size_t seed)
{
  using namespace test_msgs::msg;

  const auto primitives = generate_random_primitives(seed);
  soss::Message msg;
#define SOSS_SET_FIELD(field_name) \
  msg.data[#field_name] = soss::Convert<Primitives::_ ## field_name ## _type>::make_soss_field(primitives.field_name);

  SOSS_SET_FIELD(bool_value);
  SOSS_SET_FIELD(byte_value);
  SOSS_SET_FIELD(char_value);
  SOSS_SET_FIELD(int8_value);
  SOSS_SET_FIELD(int16_value);
  SOSS_SET_FIELD(int32_value);
  SOSS_SET_FIELD(int64_value);
  SOSS_SET_FIELD(uint8_value);
  SOSS_SET_FIELD(uint16_value);
  SOSS_SET_FIELD(uint32_value);
  SOSS_SET_FIELD(uint64_value);
  SOSS_SET_FIELD(float32_value);
  SOSS_SET_FIELD(float64_value);
  SOSS_SET_FIELD(string_value);

  return msg;
}

test_msgs::msg::BoundedArrayNested generate_random_array(std::size_t N)
{
  test_msgs::msg::BoundedArrayNested msg;
  msg.primitive_values.resize(N);
  for(std::size_t i=0; i < N; ++i)
    msg.primitive_values[i] = generate_random_primitives(i);

  return msg;
}

TEST_CASE("Transmit and receive all test messages", "[ros2]")
{
  using namespace std::chrono_literals;

  const double tolerance = 1e-8;

  YAML::Node config_node = YAML::LoadFile(ROS2__TEST_MSGS__TEST_CONFIG);

  soss::InstanceHandle handle = soss::run_instance(
        config_node, {ROS2__ROSIDL__BUILD_DIR});

  REQUIRE(handle);

  rclcpp::Node::SharedPtr ros2 = std::make_shared<rclcpp::Node>("ros2_test");
  rclcpp::executors::SingleThreadedExecutor executor;

  REQUIRE( rclcpp::ok() );

  SECTION("Send a bounded array of a message containing primitives and see that it arrives correctly ")
  {
    using namespace test_msgs::msg;

    const auto publisher =
        ros2->create_publisher<BoundedArrayNested>("transmit_primitives");
    REQUIRE(publisher);

    std::promise<soss::Message> msg_promise;
    std::future<soss::Message> msg_future = msg_promise.get_future();
    bool is_msg_received = false;
    std::mutex mock_sub_mutex;
    auto mock_sub = [&](const soss::Message& msg)
    {
      std::unique_lock<std::mutex> lock(mock_sub_mutex);
      if(is_msg_received)
        return;

      is_msg_received = true;
      msg_promise.set_value(msg);
    };
    REQUIRE(soss::mock::subscribe("transmit_primitives", mock_sub));

    const std::size_t N = 3;
    BoundedArrayNested ros2_msg = generate_random_array(N);

    publisher->publish(ros2_msg);

    executor.spin_node_some(ros2);

    // Keep spinning while we wait for the promise to be delivered. Try cycle
    // this for no more than a few seconds. If it's not finished by that time,
    // then something is probably broken with the test or with soss, and we
    // should quit instead of waiting for the future and potentially hanging
    // forever.
    auto start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);
      if(msg_future.wait_for(100ms) == std::future_status::ready)
        break;

      publisher->publish(ros2_msg);
    }

    REQUIRE(msg_future.wait_for(0s) == std::future_status::ready);
    soss::Message received_msg = msg_future.get();

    CHECK(received_msg.type == "test_msgs/BoundedArrayNested");

    std::vector<soss::Message>* data =
        received_msg.data["primitive_values"]
        .cast<std::vector<soss::Message>>();
    REQUIRE(data);
    REQUIRE(data->size() == N);

    for(std::size_t i = 0; i < N; ++i)
    {
      const soss::Message& soss_primitives = (*data)[i];
      const Primitives& ros2_primitives = ros2_msg.primitive_values[i];

      #define SOSS_REQUIRE_AND_COMPARE(field_name) { \
        const auto field_it = soss_primitives.data.find(#field_name); \
        REQUIRE(field_it != soss_primitives.data.end()); \
        Primitives::_ ## field_name ## _type field; \
        soss::Convert<Primitives::_ ## field_name ## _type>::from_soss_field(field_it, field); \
        CHECK(field == ros2_primitives.field_name); }

      SOSS_REQUIRE_AND_COMPARE(bool_value);
      SOSS_REQUIRE_AND_COMPARE(byte_value);
      SOSS_REQUIRE_AND_COMPARE(char_value);
      SOSS_REQUIRE_AND_COMPARE(int8_value);
      SOSS_REQUIRE_AND_COMPARE(int16_value);
      SOSS_REQUIRE_AND_COMPARE(int32_value);
      SOSS_REQUIRE_AND_COMPARE(int64_value);
      SOSS_REQUIRE_AND_COMPARE(uint8_value);
      SOSS_REQUIRE_AND_COMPARE(uint16_value);
      SOSS_REQUIRE_AND_COMPARE(uint32_value);
      SOSS_REQUIRE_AND_COMPARE(uint64_value);
      SOSS_REQUIRE_AND_COMPARE(string_value);
      SOSS_REQUIRE_AND_COMPARE(float32_value);
      SOSS_REQUIRE_AND_COMPARE(float64_value);
    }

    const std::size_t M = ros2_msg.primitive_values.max_size()+2;
    data->resize(M);
    // Add entries beyond the limits of the bounded vector so that we can make
    // sure the message conversion is robust to overflow.
    for(std::size_t i=N; i < M; ++i)
      (*data)[i] = generate_random_primitives_msg(445+i);

    bool promise_sent = false;
    std::promise<BoundedArrayNested> array_promise;
    auto array_future = array_promise.get_future();
    std::mutex echo_sub_mutex;
    auto echo_sub = [&](BoundedArrayNested::UniquePtr msg)
    {
      std::unique_lock<std::mutex> lock(echo_sub_mutex);
      // promises will throw an exception if set_value(~) is called more than
      // once, so we'll guard against that.
      if(promise_sent)
        return;

      promise_sent = true;
      array_promise.set_value(*msg);
    };

    const auto subscriber = ros2->create_subscription<BoundedArrayNested>(
          "echo_primitives", echo_sub);

    // Keep spinning and publishing while we wait for the promise to be
    // delivered. Try cycle this for no more than a few seconds. If it's not
    // finished by that time, then something is probably broken with the test or
    // with soss, and we should quit instead of waiting for the future and
    // potentially hanging forever.
    start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);

      soss::mock::publish_message("echo_primitives", received_msg);

      executor.spin_node_some(ros2);
      if(array_future.wait_for(100ms) == std::future_status::ready)
        break;
    }

    REQUIRE(array_future.wait_for(0s) == std::future_status::ready);
    BoundedArrayNested received_array = array_future.get();

    // Check that the arrays match for the first N entries
    for(std::size_t i=0; i < N; ++i)
      CHECK(received_array.primitive_values[i] == ros2_msg.primitive_values[i]);

    CHECK(received_array.primitive_values.size()
          == ros2_msg.primitive_values.max_size());
  }

  // Quit and wait for no more than a minute. We don't want the test to get
  // hung here indefinitely in the case of an error.
  handle.quit().wait_for(1min);

  // Require that it's no longer running. If it is still running, then it is
  // probably stuck, and we should forcefully quit.
  REQUIRE(!handle.running());
  REQUIRE(handle.wait() == 0);
}
#endif // RCLCPP__QOS_HPP_
