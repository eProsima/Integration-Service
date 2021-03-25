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
#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <random>

// TODO (@jamoralp): re-think or refactor these tests.

geometry_msgs::msg::PoseStamped generate_random_pose(
        const int sec = 0)
{
    std::mt19937 rng;
    // Use a fixed seed for deterministic test results
    rng.seed(39);
    std::uniform_real_distribution<double> dist(-100.0, 100.0);

    geometry_msgs::msg::PoseStamped ros2_pose;

    ros2_pose.pose.position.x = dist(rng);
    ros2_pose.pose.position.y = dist(rng);
    ros2_pose.pose.position.z = dist(rng);

    ros2_pose.pose.orientation.w = 1.0;
    ros2_pose.pose.orientation.x = 0.0;
    ros2_pose.pose.orientation.y = 0.0;
    ros2_pose.pose.orientation.z = 0.0;

    ros2_pose.header.frame_id = "map";
    ros2_pose.header.stamp.sec = sec;

    return ros2_pose;
}

void transform_pose_msg(
        const geometry_msgs::msg::PoseStamped& p,
        xtypes::WritableDynamicDataRef to)
{
    to["header"]["stamp"]["sec"] = p.header.stamp.sec;
    to["header"]["stamp"]["nanosec"] = p.header.stamp.nanosec;
    to["header"]["frame_id"] = "map";
    to["pose"]["position"]["x"] = p.pose.position.x;
    to["pose"]["position"]["y"] = p.pose.position.y;
    to["pose"]["position"]["z"] = p.pose.position.z;
    to["pose"]["orientation"]["x"] = p.pose.orientation.x;
    to["pose"]["orientation"]["y"] = p.pose.orientation.y;
    to["pose"]["orientation"]["z"] = p.pose.orientation.z;
    to["pose"]["orientation"]["w"] = p.pose.orientation.w;
}

xtypes::DynamicData generate_plan_request_msg(
        const xtypes::DynamicType& request_type,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const float tolerance = 1e-3f)
{
    xtypes::DynamicData message(request_type);
    transform_pose_msg(goal, message["goal"]);
    transform_pose_msg(start, message["start"]);
    message["tolerance"] = tolerance;

    return message;
}

std::string print_header(
        const std_msgs::msg::Header& header)
{
    return "[stamp: " + std::to_string(header.stamp.sec) + " | "
           + std::to_string(header.stamp.nanosec) + "] [frame_id: "
           + std::to_string(header.stamp.sec) + "]";
}

void compare_plans(
        const nav_msgs::srv::GetPlan_Response::_plan_type& plan_a,
        const nav_msgs::srv::GetPlan_Response::_plan_type& plan_b)
{
    const bool header_matches = (plan_a.header == plan_b.header);
    EXPECT_TRUE(header_matches);
    if (!header_matches)
    {
        std::cout << "Header A: " << print_header(plan_a.header)
                  << "\nHeader B: " << print_header(plan_b.header)
                  << std::endl;
    }

    ASSERT_EQ(plan_a.poses.size(), plan_b.poses.size());
    for (std::size_t i = 0; i < plan_a.poses.size(); ++i)
    {
        const bool pose_matches = (plan_a.poses[i] == plan_b.poses[i]);
        EXPECT_TRUE(pose_matches);
        if (!pose_matches)
        {
            std::cout << "Poses at index [" << i << "] did not match: ";
            for (const auto& pose : {plan_a.poses[i], plan_b.poses[i]})
            {
                std::cout << "\n -- " << print_header(pose.header)
                          << " [p: " << pose.pose.position.x
                          << ", " << pose.pose.position.y
                          << ", " << pose.pose.position.z
                          << "] [q: " << pose.pose.orientation.w
                          << ", " << pose.pose.orientation.x
                          << ", " << pose.pose.orientation.y
                          << ", " << pose.pose.orientation.z
                          << "]" << std::endl;
            }
        }
    }
}

TEST(ROS2, Publish_subscribe_between_ros2_and_mock)
{
    using namespace std::chrono_literals;

    const double tolerance = 1e-8;

    YAML::Node config_node = YAML::LoadFile(ROS2__GEOMETRY_MSGS__TEST_CONFIG);

    soss::InstanceHandle handle = soss::run_instance(
        config_node, {ROS2__ROSIDL__BUILD_DIR});

    ASSERT_TRUE(handle);

    rclcpp::Node::SharedPtr ros2 = std::make_shared<rclcpp::Node>("ros2_test");
    rclcpp::executors::SingleThreadedExecutor executor;

    ASSERT_TRUE( rclcpp::ok() );

    executor.add_node(ros2);

    const auto publisher =
#ifndef RCLCPP__QOS_HPP_
            ros2->create_publisher<geometry_msgs::msg::Pose>("transmit_pose");
#else
            ros2->create_publisher<geometry_msgs::msg::Pose>(
        "transmit_pose", rclcpp::SystemDefaultsQoS());
#endif // RCLCPP__QOS_HPP_
    ASSERT_TRUE(publisher);

    std::promise<xtypes::DynamicData> msg_promise;
    std::future<xtypes::DynamicData> msg_future = msg_promise.get_future();
    std::mutex mock_sub_mutex;
    bool mock_sub_value_received = false;
    auto mock_sub = [&](const xtypes::DynamicData& msg)
            {
                std::unique_lock<std::mutex> lock(mock_sub_mutex);
                if (mock_sub_value_received)
                {
                    return;
                }

                mock_sub_value_received = true;
                msg_promise.set_value(msg);
            };
    ASSERT_TRUE(soss::mock::subscribe("transmit_pose", mock_sub));

    geometry_msgs::msg::Pose ros2_pose = generate_random_pose().pose;

    publisher->publish(ros2_pose);

    executor.spin_some();

    // Wait no longer than a few seconds for the message to arrive. If it's not
    // ready by that time, then something is probably broken with the test, and
    // we should quit instead of waiting for the future and potentially hanging
    // forever.
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_some();
        if (msg_future.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }

        publisher->publish(ros2_pose);
    }

    ASSERT_EQ(msg_future.wait_for(0s), std::future_status::ready);
    xtypes::DynamicData received_msg = msg_future.get();

    EXPECT_EQ(received_msg.type().name(), "geometry_msgs/Pose");

    xtypes::ReadableDynamicDataRef position = received_msg["position"];
    xtypes::ReadableDynamicDataRef orientation = received_msg["orientation"];

    #define TEST_POSITION_OF( u ) \
    { \
        const double u = position[#u]; \
        ASSERT_NEAR(u, ros2_pose.position.u, tolerance); \
    }

    TEST_POSITION_OF(x);
    TEST_POSITION_OF(y);
    TEST_POSITION_OF(z);

    bool promise_sent = false;
    std::promise<geometry_msgs::msg::Pose> pose_promise;
    auto pose_future = pose_promise.get_future();
    std::mutex echo_mutex;
    auto echo_sub = [&](geometry_msgs::msg::Pose::UniquePtr msg)
            {
                std::unique_lock<std::mutex> lock(echo_mutex);

                // promises will throw an exception if set_value(~) is called more than
                // once, so we'll guard against that.
                if (promise_sent)
                {
                    return;
                }

                promise_sent = true;
                pose_promise.set_value(*msg);
            };

#ifndef RCLCPP__QOS_HPP_
    const auto subscriber = ros2->create_subscription<geometry_msgs::msg::Pose>(
        "echo_pose", echo_sub);
#else
    const auto subscriber = ros2->create_subscription<geometry_msgs::msg::Pose>(
        "echo_pose", rclcpp::SystemDefaultsQoS(), echo_sub);
#endif // RCLCPP__QOS_HPP_
    ASSERT_TRUE(subscriber);

    // Keep spinning and publishing while we wait for the promise to be
    // delivered. Try to cycle this for no more than a few seconds. If it's not
    // finished by that time, then something is probably broken with the test or
    // with soss, and we should quit instead of waiting for the future and
    // potentially hanging forever.
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_some();

        soss::mock::publish_message("echo_pose", received_msg);

        executor.spin_some();
        if (pose_future.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }
    }

    ASSERT_EQ(pose_future.wait_for(0s), std::future_status::ready);
    geometry_msgs::msg::Pose received_pose = pose_future.get();

    EXPECT_EQ(ros2_pose, received_pose);

    // Destroy ros2 instance node
    executor.remove_node(ros2);
    ros2.reset();

    // Quit and wait for no more than a minute. We don't want the test to get
    // hung here indefinitely in the case of an error.
    handle.quit().wait_for(1min);

    // Require that it's no longer running. If it is still running, then it is
    // probably stuck, and we should forcefully quit.
    ASSERT_TRUE(!handle.running());
    ASSERT_TRUE(handle.wait() == 0);
}

TEST(ROS2, Request_reply_between_ros2_and_mock)
{
    using namespace std::chrono_literals;

    const double tolerance = 1e-8;

    YAML::Node config_node = YAML::LoadFile(ROS2__GEOMETRY_MSGS__TEST_CONFIG);

    soss::InstanceHandle handle = soss::run_instance(
        config_node, {ROS2__ROSIDL__BUILD_DIR});

    ASSERT_TRUE(handle);

    rclcpp::Node::SharedPtr ros2 = std::make_shared<rclcpp::Node>("ros2_test");
    rclcpp::executors::SingleThreadedExecutor executor;

    ASSERT_TRUE( rclcpp::ok() );

    executor.add_node(ros2);

    // Get request type from ros2 middleware
    const soss::TypeRegistry& ros2_types = *handle.type_registry("ros2");
    const xtypes::DynamicType& request_type = *ros2_types.at("nav_msgs/GetPlan:request");

    // Create a plan
    nav_msgs::srv::GetPlan_Response plan_response;
    plan_response.plan.header.stamp.sec = 284;
    plan_response.plan.header.stamp.nanosec = 285;
    plan_response.plan.header.frame_id = "arbitrary_frame_string";
    for (int i = 0; i < 5; ++i)
    {
        plan_response.plan.poses.emplace_back(generate_random_pose(i));
    }

    std::promise<geometry_msgs::msg::PoseStamped> promised_start;
    auto future_start = promised_start.get_future();
    std::promise<geometry_msgs::msg::PoseStamped> promised_goal;
    auto future_goal = promised_goal.get_future();

    nav_msgs::srv::GetPlan::Request plan_request;

    bool service_called = false;
    std::mutex service_mutex;
    const auto ros2_plan_service = [&](
        const std::shared_ptr<rmw_request_id_t>, //request_header
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        const std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
            {
                std::unique_lock<std::mutex> lock(service_mutex);
                *response = plan_response;

                if (service_called)
                {
                    return;
                }

                service_called = true;
                plan_request = *request;
                promised_start.set_value(request->start);
                promised_goal.set_value(request->goal);
            };

    const auto ros2_serv = ros2->create_service<nav_msgs::srv::GetPlan>(
        "get_plan", ros2_plan_service);

    executor.spin_some();

    xtypes::DynamicData request_msg = generate_plan_request_msg(
        request_type,
        plan_response.plan.poses.front(),
        plan_response.plan.poses.back());

    auto future_response_msg = soss::mock::request(
        "get_plan", request_msg);

    // Make sure that we got the expected request message
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_some();
        // Note: future_goal gets set after future_start, so waiting on
        // future_goal alone is sufficient for waiting on both.
        if (future_goal.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }
    }

    ASSERT_EQ(future_start.wait_for(0s), std::future_status::ready);
    ASSERT_EQ(future_goal.wait_for(0s), std::future_status::ready);
    auto requested_start = future_start.get();
    EXPECT_EQ(requested_start, plan_response.plan.poses.front());
    auto requested_goal = future_goal.get();
    EXPECT_EQ(requested_goal, plan_response.plan.poses.back());

    start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_some();
        if (future_response_msg.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }
    }
    ASSERT_EQ(future_response_msg.wait_for(0s), std::future_status::ready);

    const xtypes::DynamicData response_msg = future_response_msg.get();

    // TODO(MXG): We could copy the request message that gets passed to here and
    // compare it against the original request message that was sent. This would
    // require implementing comparison operators for the soss::Message class.
    std::mutex serve_mutex;
    soss::mock::serve("echo_plan", [&](const xtypes::DynamicData&)
            {
                std::unique_lock<std::mutex> lock(serve_mutex);
                return response_msg;
            });

    const auto client =
            ros2->create_client<nav_msgs::srv::GetPlan>("echo_plan");
    ASSERT_TRUE(client->wait_for_service(10s));

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    *request = plan_request;
    auto future_response = client->async_send_request(request);

    // Keep spinning while we wait for the promise to be delivered. Try to cycle
    // this for no more than a few seconds. If it's not finished by that time,
    // then something is probably broken with the test or with soss, and we
    // should quit instead of waiting for the future and potentially hanging
    // forever.
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_some();
        if (future_response.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }
    }

    ASSERT_EQ(future_response.wait_for(0s), std::future_status::ready);
    const auto response = future_response.get();
    EXPECT_EQ(*response, plan_response);
    compare_plans(response->plan, plan_response.plan);

    // Destroy ros2 instance node
    executor.remove_node(ros2);
    ros2.reset();

    // Quit and wait for no more than a minute. We don't want the test to get
    // hung here indefinitely in the case of an error.
    handle.quit().wait_for(1min);

    // Require that it's no longer running. If it is still running, then it is
    // probably stuck, and we should forcefully quit.
    ASSERT_TRUE(!handle.running());
    ASSERT_TRUE(handle.wait() == 0);
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}