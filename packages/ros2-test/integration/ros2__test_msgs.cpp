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

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <random>

#include <soss_ros2_test_msg/msg/bounded_array_nested.hpp>
#define ROS2_TYPE_NAME "soss_ros2_test_msg/BoundedArrayNested"

template<typename Ros2Primitives>
Ros2Primitives generate_random_primitives(
        const std::size_t seed)
{
    std::mt19937 rng;
    // Use consistent seeds for deterministic test results
    rng.seed(64 + seed);

    Ros2Primitives primitives;

    // uniform_int_distribution is only compatible with short, int, long, long long,
    // unsigned short, unsigned int, unsigned long, or unsigned long long. So for smaller
    // values use int16_t (short)
    primitives.bool_value = std::uniform_int_distribution<int16_t>(0, 2)(rng);
    primitives.byte_value = std::uniform_int_distribution<uint16_t>()(rng);
    primitives.char_value = std::uniform_int_distribution<uint16_t>()(rng);
    primitives.int8_value = std::uniform_int_distribution<typename Ros2Primitives::_int16_value_type>()(rng);
    primitives.int16_value = std::uniform_int_distribution<typename Ros2Primitives::_int16_value_type>()(rng);
    primitives.int32_value = std::uniform_int_distribution<typename Ros2Primitives::_int32_value_type>()(rng);
    primitives.int64_value = std::uniform_int_distribution<typename Ros2Primitives::_int64_value_type>()(rng);
    primitives.uint8_value = std::uniform_int_distribution<typename Ros2Primitives::_uint16_value_type>()(rng);
    primitives.uint16_value = std::uniform_int_distribution<typename Ros2Primitives::_uint16_value_type>()(rng);
    primitives.uint32_value = std::uniform_int_distribution<typename Ros2Primitives::_uint32_value_type>()(rng);
    primitives.uint64_value = std::uniform_int_distribution<typename Ros2Primitives::_uint64_value_type>()(rng);
    primitives.float32_value = std::uniform_real_distribution<typename Ros2Primitives::_float32_value_type>()(rng);
    primitives.float64_value = std::uniform_real_distribution<typename Ros2Primitives::_float64_value_type>()(rng);
    primitives.string_value = std::to_string(std::uniform_int_distribution<int32_t>(0, 100)(rng));

    return primitives;
}

template<typename Ros2Primitives>
xtypes::DynamicData generate_random_primitives_msg(
        const std::size_t seed,
        const xtypes::DynamicType& type)
{
    const Ros2Primitives ros2_primitives = generate_random_primitives<Ros2Primitives>(seed);
    xtypes::DynamicData xtypes_primitives(type);
#define SOSS_SET_FIELD(field_name) \
    soss::Convert<typename Ros2Primitives::_ ## field_name ## _type>::to_xtype_field(ros2_primitives.field_name, \
            xtypes_primitives[#field_name]); \

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

    return xtypes_primitives;
}

template<typename Ros2Array, typename Ros2Primitives>
Ros2Array generate_random_array(
        std::size_t N)
{
    Ros2Array msg;
    msg.primitive_values.resize(N);
    for (std::size_t i = 0; i < N; ++i)
    {
        msg.primitive_values[i] = generate_random_primitives<Ros2Primitives>(i);
    }

    return msg;
}

template<typename Ros2Array, typename Ros2Primitives>
void test(
        rclcpp::Node::SharedPtr& ros2,
        rclcpp::executors::SingleThreadedExecutor& executor)
{
    using namespace std::chrono_literals;

    const auto publisher =
#ifndef RCLCPP__QOS_HPP_
            ros2->create_publisher<Ros2Array>("transmit");
#else
            ros2->create_publisher<Ros2Array>("transmit", rclcpp::SystemDefaultsQoS());
#endif // RCLCPP__QOS_HPP_
    ASSERT_TRUE(publisher);

    std::promise<xtypes::DynamicData> msg_promise;
    std::future<xtypes::DynamicData> msg_future = msg_promise.get_future();
    bool is_msg_received = false;
    std::mutex mock_sub_mutex;
    auto mock_sub = [&](const xtypes::DynamicData& msg)
            {
                std::unique_lock<std::mutex> lock(mock_sub_mutex);
                if (is_msg_received)
                {
                    return;
                }

                is_msg_received = true;
                msg_promise.set_value(msg);
            };
    ASSERT_TRUE(soss::mock::subscribe("transmit", mock_sub));

    const std::size_t N = 3;
    Ros2Array ros2_msg = generate_random_array<Ros2Array, Ros2Primitives>(N);

    publisher->publish(ros2_msg);

    executor.spin_node_some(ros2);

    // Keep spinning while we wait for the promise to be delivered. Try cycle
    // this for no more than a few seconds. If it's not finished by that time,
    // then something is probably broken with the test or with soss, and we
    // should quit instead of waiting for the future and potentially hanging
    // forever.
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_node_some(ros2);
        if (msg_future.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }

        publisher->publish(ros2_msg);
    }

    ASSERT_EQ(msg_future.wait_for(0s), std::future_status::ready);
    xtypes::DynamicData received_msg = msg_future.get();

    EXPECT_EQ(received_msg.type().name(), ROS2_TYPE_NAME);


    xtypes::WritableDynamicDataRef seq = received_msg["primitive_values"];
    ASSERT_EQ(seq.size(), N);

    for (std::size_t i = 0; i < N; ++i)
    {
        xtypes::ReadableDynamicDataRef xtypes_primitives = seq[i];
        const Ros2Primitives& ros2_primitives = ros2_msg.primitive_values[i];

    #define SOSS_REQUIRE_AND_COMPARE(field_name) { \
        typename Ros2Primitives::_ ## field_name ## _type ros2_field; \
        soss::Convert<typename Ros2Primitives::_ ## field_name ## _type>::from_xtype_field( \
            xtypes_primitives[#field_name], ros2_field); \
        EXPECT_EQ(ros2_field, ros2_primitives.field_name); \
    }

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

    const std::size_t M = ros2_msg.primitive_values.max_size();
    seq.resize(M);
    // Add entries.
    for (std::size_t i = N; i < M; ++i)
    {
        seq[i] = generate_random_primitives_msg<Ros2Primitives>(445 + i, seq[0].type());
    }

    bool promise_sent = false;
    std::promise<Ros2Array> array_promise;
    auto array_future = array_promise.get_future();
    std::mutex echo_sub_mutex;
    auto echo_sub = [&](typename Ros2Array::UniquePtr msg)
            {
                std::unique_lock<std::mutex> lock(echo_sub_mutex);
                // promises will throw an exception if set_value(~) is called more than
                // once, so we'll guard against that.
                if (promise_sent)
                {
                    return;
                }

                promise_sent = true;
                array_promise.set_value(*msg);
            };

    const auto subscriber =
#ifndef RCLCPP__QOS_HPP_
            ros2->create_subscription<Ros2Array>("echo", echo_sub);
#else
            ros2->create_subscription<Ros2Array>("echo", rclcpp::SystemDefaultsQoS(), echo_sub);
#endif // RCLCPP__QOS_HPP_
    ASSERT_TRUE(subscriber);

    // Keep spinning and publishing while we wait for the promise to be
    // delivered. Try cycle this for no more than a few seconds. If it's not
    // finished by that time, then something is probably broken with the test or
    // with soss, and we should quit instead of waiting for the future and
    // potentially hanging forever.
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_node_some(ros2);

        soss::mock::publish_message("echo", received_msg);

        executor.spin_node_some(ros2);
        if (array_future.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }
    }

    ASSERT_EQ(array_future.wait_for(0s), std::future_status::ready);
    Ros2Array received_array = array_future.get();

    // Check that the arrays match for the first N entries
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_EQ(received_array.primitive_values[i], ros2_msg.primitive_values[i]);
    }

    EXPECT_EQ(received_array.primitive_values.size(), ros2_msg.primitive_values.max_size());
}

TEST(ROS2, Transmit_and_receive_all_test_messages)
{
    using namespace std::chrono_literals;

    YAML::Node config_node = YAML::LoadFile(ROS2__TEST_MSGS__TEST_CONFIG);

    soss::InstanceHandle handle = soss::run_instance(
        config_node, {ROS2__ROSIDL__BUILD_DIR});

    ASSERT_TRUE(handle);

    rclcpp::Node::SharedPtr ros2 = std::make_shared<rclcpp::Node>("ros2_test");
    rclcpp::executors::SingleThreadedExecutor executor;

    ASSERT_TRUE( rclcpp::ok() );

    test<soss_ros2_test_msg::msg::BoundedArrayNested, soss_ros2_test_msg::msg::Primitives>(ros2, executor);

    // Quit and wait for no more than a minute. We don't want the test to get
    // hung here indefinitely in the case of an error.
    handle.quit().wait_for(1min);

    // Require that it's no longer running. If it is still running, then it is
    // probably stuck, and we should forcefully quit.
    ASSERT_TRUE(!handle.running());
    ASSERT_EQ(handle.wait(), 0);
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
