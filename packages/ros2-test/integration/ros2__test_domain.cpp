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

#include <soss/Instance.hpp>
#include <soss/utilities.hpp>

#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>
#include <chrono>

#include <catch2/catch.hpp>

using Catch::Matchers::WithinAbs;

#ifdef WIN32
#define SETENV(id,value,b) \
	{std::ostringstream _aux_d; \
	_aux_d << id << "=" << value; \
	_putenv(_aux_d.str().c_str());}
#define UNSETENV(id,retValue) SETENV(id, "", false)
#else
#define SETENV(id,value,b) setenv(id, value, b)
#define UNSETENV(id) unsetenv(id)
#endif

TEST_CASE("Change ROS2 Domain id test case", "[ros2]")
{
  const int argc = 1;
  const char* argv[argc];
  argv[0] = "soss";
  if (rclcpp::is_initialized())
  {
    std::cout << "Skipping rclcpp init. rcl already initialized." << std::endl;
  }
  else
  {
    std::cout << "Initializing rclcpp." << std::endl;
    rclcpp::init(argc, argv);
  }

  REQUIRE(rclcpp::ok());

  // Change domain to 5 and create node.
  SETENV("ROS_DOMAIN_ID", "5", true);

  std::string name_1 = "Node_1";
  std::string ns_1 = "";
  rclcpp::Node::SharedPtr node_1 = std::make_shared<rclcpp::Node>(name_1, ns_1);

  // Change domain to 10 and create node
  SETENV("ROS_DOMAIN_ID", "10", true);

  std::string name_2 = "Node_2";
  std::string ns_2 = "";
  rclcpp::Node::SharedPtr node_2 = std::make_shared<rclcpp::Node>(name_2, ns_2);


  // Create publisher in domain 5
  const auto publisher =
    node_1->create_publisher<std_msgs::msg::String>("string_topic");


  std::promise<std_msgs::msg::String> msg_promise;
  std::future<std_msgs::msg::String> msg_future = msg_promise.get_future();
  //std::mutex node2_sub_mutex;
  auto node2_sub = [&](std_msgs::msg::String::UniquePtr msg)
  {
    //std::unique_lock<std::mutex> lock(node2_sub_mutex);
    msg_promise.set_value(*msg);

  };

  const auto subscriber = node_2->create_subscription<std_msgs::msg::String>(
    "string_topic", node2_sub);

  std_msgs::msg::String pub_msg;
  pub_msg.set__data("Hello node");

  
  rclcpp::executors::SingleThreadedExecutor executor;
  using namespace std::chrono_literals;

  auto start_time = std::chrono::steady_clock::now();
  // Three secs trying to receive msg.
  while (std::chrono::steady_clock::now() - start_time < 3s)
  {
    publisher->publish(pub_msg);
    executor.spin_node_some(node_1);
    executor.spin_node_some(node_2);
    // In different domains the message should not be received
    if (msg_future.wait_for(100ms) == std::future_status::ready)
      break;
  }

  REQUIRE(msg_future.wait_for(0s) != std::future_status::ready);

  // Run soss in order to make the communication possible.
  YAML::Node config_node = YAML::LoadFile(ROS2__TEST_DOMAIN__TEST_CONFIG);

  soss::InstanceHandle handle = soss::run_instance(
    config_node, { ROS2__ROSIDL__BUILD_DIR });

  REQUIRE(handle);

  // Three secs trying to receive msg.
  start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 3s)
  {
    publisher->publish(pub_msg);
    executor.spin_node_some(node_1);
    executor.spin_node_some(node_2);
    // Now with soss active the communication should work.
    if (msg_future.wait_for(100ms) == std::future_status::ready)
      break;
  }

  REQUIRE(msg_future.wait_for(0s) == std::future_status::ready);
  std_msgs::msg::String received_msg = msg_future.get();
  CHECK(pub_msg == received_msg);

}
