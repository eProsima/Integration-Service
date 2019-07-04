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

#include <catch2/catch.hpp>

#include <random>

using Catch::Matchers::WithinAbs;

geometry_msgs::msg::PoseStamped generate_random_pose(const int sec = 0)
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

soss::Message generate_pose_msg(
    const geometry_msgs::msg::PoseStamped& p)
{
  soss::Message stamp;
  stamp.type = "builtin_interfaces/Time";
  stamp.data["sec"] = soss::Convert<int32_t>::make_soss_field(p.header.stamp.sec);
  stamp.data["nanosec"] = soss::Convert<uint32_t>::make_soss_field(p.header.stamp.nanosec);

  soss::Message header;
  header.type = "std_msgs/Header";
  header.data["stamp"] = soss::make_field<soss::Message>(stamp);
  header.data["frame_id"] = soss::Convert<std::string>::make_soss_field("map");

  soss::Message position;
  position.type = "geometry_msgs/Point";
  position.data["x"] = soss::Convert<double>::make_soss_field(p.pose.position.x);
  position.data["y"] = soss::Convert<double>::make_soss_field(p.pose.position.y);
  position.data["z"] = soss::Convert<double>::make_soss_field(p.pose.position.z);

  soss::Message orientation;
  orientation.type = "geometry_msgs/Quaternion";
  orientation.data["x"] = soss::Convert<double>::make_soss_field(p.pose.orientation.x);
  orientation.data["y"] = soss::Convert<double>::make_soss_field(p.pose.orientation.y);
  orientation.data["z"] = soss::Convert<double>::make_soss_field(p.pose.orientation.z);
  orientation.data["w"] = soss::Convert<double>::make_soss_field(p.pose.orientation.w);

  soss::Message pose;
  pose.type = "geometry_msgs/Pose";
  pose.data["position"] = soss::make_field<soss::Message>(position);
  pose.data["orientation"] = soss::make_field<soss::Message>(orientation);

  soss::Message msg;
  msg.type = "geometry_msgs/PoseStamped";
  msg.data["header"] = soss::make_field<soss::Message>(header);
  msg.data["pose"] = soss::make_field<soss::Message>(pose);

  return msg;
}

soss::Message generate_plan_request_msg(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const float tolerance = 1e-3f)
{
  soss::Message msg;
  msg.type = "nav_msgs/GetPlan:request";
  msg.data["goal"] = soss::make_field<soss::Message>(generate_pose_msg(goal));
  msg.data["start"] = soss::make_field<soss::Message>(generate_pose_msg(start));
  msg.data["tolerance"] = soss::Convert<float>::make_soss_field(tolerance);

  return msg;
}

std::string print_header(const std_msgs::msg::Header& header)
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
  CHECK(header_matches);
  if(!header_matches)
  {
    std::cout << "Header A: " << print_header(plan_a.header)
              << "\nHeader B: " << print_header(plan_b.header)
              << std::endl;
  }

  REQUIRE(plan_a.poses.size() == plan_b.poses.size());
  for(std::size_t i=0; i < plan_a.poses.size(); ++i)
  {
    const bool pose_matches = (plan_a.poses[i] == plan_b.poses[i]);
    CHECK(pose_matches);
    if(!pose_matches)
    {
      std::cout << "Poses at index [" << i << "] did not match: ";
      for(const auto& pose : {plan_a.poses[i], plan_b.poses[i]})
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

TEST_CASE("Talk between ros2 and the mock middleware", "[ros2]")
{
  using namespace std::chrono_literals;

  const double tolerance = 1e-8;

  YAML::Node config_node = YAML::LoadFile(ROS2__GEOMETRY_MSGS__TEST_CONFIG);

  soss::InstanceHandle handle = soss::run_instance(
        config_node, {ROS2__ROSIDL__BUILD_DIR});

  REQUIRE(handle);

  rclcpp::Node::SharedPtr ros2 = std::make_shared<rclcpp::Node>("ros2_test");
  rclcpp::executors::SingleThreadedExecutor executor;

  REQUIRE( rclcpp::ok() );

  SECTION("Publish a pose and get it echoed back")
  {
#ifndef RCLCPP__QOS_HPP_
    const auto publisher =
        ros2->create_publisher<geometry_msgs::msg::Pose>("transmit_pose");
#else
    const auto publisher =
        ros2->create_publisher<geometry_msgs::msg::Pose>(
          "transmit_pose", rclcpp::SystemDefaultsQoS());
#endif // RCLCPP__QOS_HPP_
    REQUIRE(publisher);

    std::promise<soss::Message> msg_promise;
    std::future<soss::Message> msg_future = msg_promise.get_future();
    std::mutex mock_sub_mutex;
    bool mock_sub_value_received = false;
    auto mock_sub = [&](const soss::Message& msg)
    {
      std::unique_lock<std::mutex> lock(mock_sub_mutex);
      if(mock_sub_value_received)
        return;

      mock_sub_value_received = true;
      msg_promise.set_value(msg);
    };
    REQUIRE(soss::mock::subscribe("transmit_pose", mock_sub));

    geometry_msgs::msg::Pose ros2_pose = generate_random_pose().pose;

    publisher->publish(ros2_pose);

    executor.spin_node_some(ros2);

    // Wait no longer than a few seconds for the message to arrive. If it's not
    // ready by that time, then something is probably broken with the test, and
    // we should quit instead of waiting for the future and potentially hanging
    // forever.
    auto start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);
      if(msg_future.wait_for(100ms) == std::future_status::ready)
        break;

      publisher->publish(ros2_pose);
    }

    REQUIRE(msg_future.wait_for(0s) == std::future_status::ready);
    soss::Message received_msg = msg_future.get();

    CHECK(received_msg.type == "geometry_msgs/Pose");

    soss::Message* position =
        received_msg.data["position"].cast<soss::Message>();
    REQUIRE(position);

    soss::Message* orientation =
        received_msg.data["orientation"].cast<soss::Message>();
    REQUIRE(orientation);

    #define TEST_POSITION_OF( u ) \
    { \
      const double* u = position->data[#u].cast<double>(); \
      REQUIRE(u); \
      CHECK_THAT(*u, WithinAbs(ros2_pose.position.u, tolerance)); \
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
      if(promise_sent)
        return;

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

    // Keep spinning and publishing while we wait for the promise to be
    // delivered. Try to cycle this for no more than a few seconds. If it's not
    // finished by that time, then something is probably broken with the test or
    // with soss, and we should quit instead of waiting for the future and
    // potentially hanging forever.
    start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);

      soss::mock::publish_message("echo_pose", received_msg);

      executor.spin_node_some(ros2);
      if(pose_future.wait_for(100ms) == std::future_status::ready)
        break;
    }

    REQUIRE(pose_future.wait_for(0s) == std::future_status::ready);
    geometry_msgs::msg::Pose received_pose = pose_future.get();

    CHECK(ros2_pose == received_pose);
  }
  SECTION("Request a plan and get it echoed back")
  {
    // Create a plan
    nav_msgs::srv::GetPlan_Response plan_response;
    plan_response.plan.header.stamp.sec = 284;
    plan_response.plan.header.stamp.nanosec = 285;
    plan_response.plan.header.frame_id = "arbitrary_frame_string";
    for(int i=0 ; i < 5; ++i)
      plan_response.plan.poses.push_back(generate_random_pose(i));

    std::promise<geometry_msgs::msg::PoseStamped> promised_start;
    auto future_start = promised_start.get_future();
    std::promise<geometry_msgs::msg::PoseStamped> promised_goal;
    auto future_goal = promised_goal.get_future();

    nav_msgs::srv::GetPlan::Request plan_request;

    bool service_called = false;
    std::mutex service_mutex;
    const auto ros2_plan_service = [&](
        const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        const std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
      std::unique_lock<std::mutex> lock(service_mutex);
      *response = plan_response;

      if(service_called)
        return;

      service_called = true;
      plan_request = *request;
      promised_start.set_value(request->start);
      promised_goal.set_value(request->goal);
    };

    const auto ros2_serv = ros2->create_service<nav_msgs::srv::GetPlan>(
          "get_plan", ros2_plan_service);

    executor.spin_node_some(ros2);

    soss::Message request_msg = generate_plan_request_msg(
          plan_response.plan.poses.front(),
          plan_response.plan.poses.back());

    auto future_response_msg = soss::mock::request(
          "get_plan", request_msg, 100ms);

    // Make sure that we got the expected request message
    auto start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);
      // Note: future_goal gets set after future_start, so waiting on
      // future_goal alone is sufficient for waiting on both.
      if(future_goal.wait_for(100ms) == std::future_status::ready)
        break;
    }

    REQUIRE(future_start.wait_for(0s) == std::future_status::ready);
    REQUIRE(future_goal.wait_for(0s) == std::future_status::ready);
    auto requested_start = future_start.get();
    CHECK(requested_start == plan_response.plan.poses.front());
    auto requested_goal = future_goal.get();
    CHECK(requested_goal == plan_response.plan.poses.back());

    start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);
      if(future_response_msg.wait_for(100ms) == std::future_status::ready)
        break;
    }
    REQUIRE(future_response_msg.wait_for(0s) == std::future_status::ready);
    const soss::Message response_msg = future_response_msg.get();

    // TODO(MXG): We could copy the request message that gets passed to here and
    // compare it against the original request message that was sent. This would
    // require implementing comparison operators for the soss::Message class.
    std::mutex serve_mutex;
    soss::mock::serve("echo_plan", [&](const soss::Message&)
    {
      std::unique_lock<std::mutex> lock(serve_mutex);
      return response_msg;
    });

    const auto client =
        ros2->create_client<nav_msgs::srv::GetPlan>("echo_plan");
    REQUIRE(client->wait_for_service(10s));

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    *request = plan_request;
    auto future_response = client->async_send_request(request);

    // Keep spinning while we wait for the promise to be delivered. Try to cycle
    // this for no more than a few seconds. If it's not finished by that time,
    // then something is probably broken with the test or with soss, and we
    // should quit instead of waiting for the future and potentially hanging
    // forever.
    start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < 30s)
    {
      executor.spin_node_some(ros2);
      if(future_response.wait_for(100ms) == std::future_status::ready)
        break;
    }

    REQUIRE(future_response.wait_for(0s) == std::future_status::ready);
    const auto response = future_response.get();
    CHECK(*response == plan_response);
    compare_plans(response->plan, plan_response.plan);
  }

  // Quit and wait for no more than a minute. We don't want the test to get
  // hung here indefinitely in the case of an error.
  handle.quit().wait_for(1min);

  // Require that it's no longer running. If it is still running, then it is
  // probably stuck, and we should forcefully quit.
  REQUIRE(!handle.running());
  REQUIRE(handle.wait() == 0);
}
