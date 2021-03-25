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
#include <rclcpp/node_options.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <rcl/logging.h>

#include <soss/Instance.hpp>
#include <soss/utilities.hpp>

#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>
#include <chrono>

#include <gtest/gtest.h>

constexpr const char* DOMAIN_ID_1 = "5";
constexpr const char* DOMAIN_ID_2 = "10";

#ifdef WIN32
#define SETENV(id, value, b) \
    {std::ostringstream _aux_d; \
     _aux_d << id << "=" << value; \
     _putenv(_aux_d.str().c_str());}
#define UNSETENV(id, retValue) SETENV(id, "", false)
#else
#define SETENV(id, value, b) setenv(id, value, b)
#define UNSETENV(id) unsetenv(id)
#endif // ifdef WIN32

TEST(ROS2, Change_ROS2_Domain_id)
{
    char const* const argv[1] = {"soss"};
    if (!rclcpp::ok())
    {
        rclcpp::init(1, argv);
    }
    ASSERT_TRUE(rclcpp::ok());

    // Create the nodes in separate context, since in ROS2 Foxy each context uses
    // an unique DDS participant and thus creating the two nodes in the same context
    // would result in them having the same DOMAIN_ID.
    SETENV("ROS_DOMAIN_ID", DOMAIN_ID_1, true);

    rclcpp::InitOptions init_options_1;
    if (rcl_logging_rosout_enabled())
    {
        init_options_1.auto_initialize_logging(false);
    }

    const char* const argv_1[1] = {"soss_context_1"};
    auto context_1 = std::make_shared<rclcpp::Context>();
    context_1->init(1, argv_1, init_options_1);

    rclcpp::NodeOptions node_ops_1;
    node_ops_1.context(context_1);

    // This needs to be called so that NodeOptions::node_options_ pointer gets filled.
    auto rcl_node_ops_1 = node_ops_1.get_rcl_node_options();

    auto node_1 = std::make_shared<rclcpp::Node>("node_1", node_ops_1);

    UNSETENV("ROS_DOMAIN_ID");

    SETENV("ROS_DOMAIN_ID", DOMAIN_ID_2, true);

    rclcpp::InitOptions init_options_2;
    if (rcl_logging_rosout_enabled())
    {
        init_options_2.auto_initialize_logging(false);
    }

    const char* const argv_2[1] = {"soss_context_2"};
    auto context_2 = std::make_shared<rclcpp::Context>();
    context_2->init(1, argv_2);

    rclcpp::NodeOptions node_ops_2;
    node_ops_2.context(context_2);

    // This needs to be called so that NodeOptions::node_options_ pointer gets filled.
    auto rcl_node_ops_2 = node_ops_2.get_rcl_node_options();

    auto node_2 = std::make_shared<rclcpp::Node>("node_2", node_ops_2);

    UNSETENV("ROS_DOMAIN_ID");

    std::cout << "[soss-ros2-test] Domain ID for 'node1': "
              << rcl_node_ops_1->domain_id << std::endl;
    std::cout << "[soss-ros2-test] Domain ID for 'node2': "
              << rcl_node_ops_2->domain_id << std::endl;

    const std::string topic_name("string_topic");

#ifdef RCLCPP__QOS_HPP_
    const auto publisher =
            node_1->create_publisher<std_msgs::msg::String>(topic_name, rclcpp::SystemDefaultsQoS());
#else
    const auto publisher =
            node_1->create_publisher<std_msgs::msg::String>(topic_name);
#endif // ifdef RCLCPP__QOS_HPP_

    std::promise<std_msgs::msg::String> msg_promise;
    std::future<std_msgs::msg::String> msg_future = msg_promise.get_future();
    std::mutex node2_sub_mutex;
    auto node2_sub = [&](std_msgs::msg::String::UniquePtr msg)
            {
                std::unique_lock<std::mutex> lock(node2_sub_mutex);
                msg_promise.set_value(*msg);
            };

#ifdef RCLCPP__QOS_HPP_
    const auto subscriber = node_2->create_subscription<std_msgs::msg::String>(
        topic_name, rclcpp::SystemDefaultsQoS(), node2_sub);
#else
    const auto subscriber = node_2->create_subscription<std_msgs::msg::String>(
        topic_name, node2_sub);
#endif // ifdef RCLCPP__QOS_HPP_

    std_msgs::msg::String pub_msg;
    pub_msg.set__data("Hello node");

    rclcpp::executors::SingleThreadedExecutor executor;
    using namespace std::chrono_literals;

    auto rclcpp_delay = 500ms;
    publisher->publish(pub_msg);
    executor.spin_node_some(node_1);
    std::this_thread::sleep_for(rclcpp_delay);
    executor.spin_node_some(node_2);

    // In different domains the message should not be received
    ASSERT_NE(msg_future.wait_for(0s), std::future_status::ready);

    // Run soss in order to make the communication possible.
    YAML::Node config_node = YAML::LoadFile(ROS2__TEST_DOMAIN__TEST_CONFIG);

    soss::InstanceHandle handle = soss::run_instance(
        config_node, { ROS2__ROSIDL__BUILD_DIR });

    ASSERT_TRUE(handle);

    // Wait for soss to start properly before publishing.
    std::this_thread::sleep_for(1s);
    publisher->publish(pub_msg);
    executor.spin_node_some(node_1);
    std::this_thread::sleep_for(rclcpp_delay);
    executor.spin_node_some(node_2);

    ASSERT_EQ(msg_future.wait_for(0s), std::future_status::ready);

    std_msgs::msg::String received_msg = msg_future.get();

    ASSERT_EQ(pub_msg, received_msg);
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
