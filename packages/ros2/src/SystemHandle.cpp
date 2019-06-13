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

#include "SystemHandle.hpp"
#include "MetaPublisher.hpp"

#include <soss/ros2/Factory.hpp>

#include <soss/Mix.hpp>
#include <soss/Search.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace soss {
namespace ros2 {

namespace {

rmw_qos_profile_t parse_rmw_qos_configuration(
    const YAML::Node& /*configuration*/)
{
  // TODO(MXG): Parse the configuration node
  // This is currently considered low-priority
  return rmw_qos_profile_default;
}

}

//==============================================================================
SystemHandle::SystemHandle()
{
  // Do nothing
}

//==============================================================================
void print_missing_mix_file(
    const std::string& msg_or_srv,
    const std::string& type,
    const std::vector<std::string>& /*checked_paths*/)
{
  std::string error = "soss-ros2 could not find .mix file for " + msg_or_srv
      + " type: " + type +
      "\n -- Make sure that you have generated the soss-ros2 extension for "
      "that message type by calling "
      "soss_rosidl_mix(PACKAGES <package> MIDDLEWARES ros2) "
      "in your build system!\n";

//  // TODO(MXG): Introduce a way for users to request a "debug", especially from
//  // the command line. When debug mode is active, this should be printed out.
//  error += " -- Checked locations (these files did not exist or could not be accessed):\n";
//  for(const auto& p : checked_paths)
//    error += "     - " + p + "\n";

  std::cerr << error;
}

//==============================================================================
bool SystemHandle::configure(
    const RequiredTypes& types,
    const YAML::Node& configuration)
{
  const int argc = 1;
  const char* argv[argc];
  bool success = true;
  argv[0] = "soss";

  if(!rclcpp::is_initialized())
  {
    rclcpp::init(argc, argv);
  }

  std::string ns = "";
  if(const YAML::Node namespace_node = configuration["namespace"])
  {
    ns = namespace_node.as<std::string>("");
  }

  std::string name = "soss_ros2";
  if(const YAML::Node name_node = configuration["node_name"])
  {
    name = name_node.as<std::string>("");
  }

  if(const YAML::Node domain_node = configuration["domain"])
  {
    std::string previous_domain;
    if(getenv("ROS_DOMAIN_ID") != nullptr)
    {
      previous_domain = getenv("ROS_DOMAIN_ID");
    }

    std::string domain = domain_node.as<std::string>();
    success += setenv("ROS_DOMAIN_ID", domain.c_str(), true);

    _node = std::make_shared<rclcpp::Node>(name, ns);

    if(previous_domain.empty())
    {
        success += unsetenv("ROS_DOMAIN_ID");
    }
    else
    {
        success += setenv("ROS_DOMAIN_ID", previous_domain.c_str(), true);
    }
  }
  else
  {
    _node = std::make_shared<rclcpp::Node>(name, ns);
  }

  // TODO(MXG): Allow the type of executor to be specified by the configuration
  _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

  soss::Search search("ros2");
  for(const std::string& type : types.messages)
  {
    std::vector<std::string> checked_paths;
    const std::string msg_mix_path =
        search.find_message_mix(type, &checked_paths);

    if(msg_mix_path.empty())
    {
      print_missing_mix_file("message", type, checked_paths);
      success = false;
      continue;
    }

    if(!Mix::from_file(msg_mix_path).load())
    {
      std::cerr << "soss-ros2 failed to load extension for message type ["
                << type << "] using mix file: " << msg_mix_path << std::endl;
      success = false;
    }
  }

  for(const std::string& type : types.services)
  {
    std::vector<std::string> checked_paths;
    const std::string srv_mix_path =
        search.find_service_mix(type, &checked_paths);

    if(srv_mix_path.empty())
    {
      print_missing_mix_file("service", type, checked_paths);
      success = false;
      continue;
    }

    if(!Mix::from_file(srv_mix_path).load())
    {
      std::cerr << "soss-ros2 failed to load extension for service type ["
                << type << "] using mix file: " << srv_mix_path << std::endl;
      success = false;
    }
  }

  return success;
}

//==============================================================================
bool SystemHandle::okay() const
{
  if(_node)
    return rclcpp::ok();

  return false;
}

//==============================================================================
bool SystemHandle::spin_once()
{
  _executor->spin_node_once(_node, std::chrono::milliseconds(100));
  return rclcpp::ok();
}

//==============================================================================
SystemHandle::~SystemHandle()
{
  _subscriptions.clear();
  _client_proxies.clear();

  rclcpp::shutdown();
}

//==============================================================================
bool SystemHandle::subscribe(
    const std::string& topic_name,
    const std::string& message_type,
    SubscriptionCallback callback,
    const YAML::Node& configuration)
{
  auto subscription = Factory::instance().create_subscription(
        message_type, *_node, topic_name, std::move(callback),
        parse_rmw_qos_configuration(configuration));

  if(!subscription)
    return false;

  _subscriptions.emplace_back(std::move(subscription));
  return true;
}

//==============================================================================
std::shared_ptr<TopicPublisher> SystemHandle::advertise(
    const std::string& topic_name,
    const std::string& message_type,
    const YAML::Node& configuration)
{
  if(topic_name.find('{') != std::string::npos)
  {
    // If the topic name contains a curly brace, we must assume that it needs
    // runtime substitutions.
    return make_meta_publisher(
          message_type, *_node, topic_name,
          parse_rmw_qos_configuration(configuration),
          configuration);
  }

  return Factory::instance().create_publisher(
        message_type, *_node, topic_name,
        parse_rmw_qos_configuration(configuration));
}

//==============================================================================
bool SystemHandle::create_client_proxy(
    const std::string& service_name,
    const std::string& service_type,
    RequestCallback callback,
    const YAML::Node& configuration)
{
  auto client_proxy = Factory::instance().create_client_proxy(
        service_type, *_node, service_name, callback,
        parse_rmw_qos_configuration(configuration));

  if(!client_proxy)
    return false;

  _client_proxies.emplace_back(std::move(client_proxy));
  return true;
}

//==============================================================================
std::shared_ptr<ServiceProvider> SystemHandle::create_service_proxy(
    const std::string& service_name,
    const std::string& service_type,
    const YAML::Node& configuration)
{
  return Factory::instance().create_server_proxy(
        service_type, *_node, service_name,
        parse_rmw_qos_configuration(configuration));
}

} // namespace ros2
} // namespace soss

//==============================================================================
SOSS_REGISTER_SYSTEM("ros2", soss::ros2::SystemHandle)
