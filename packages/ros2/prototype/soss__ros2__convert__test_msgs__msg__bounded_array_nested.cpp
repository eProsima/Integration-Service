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

// Include the API header for this message type
#include "soss__ros2__convert__test_msgs__msg__bounded_array_nested.hpp"

// Include the Factory header so we can add this message type to the Factory
#include <soss/ros2/Factory.hpp>

// Include the Node API so we can subscribe and advertise
#include <rclcpp/node.hpp>

namespace soss {
namespace ros2 {
namespace convert__test_msgs__msg__bounded_array_nested {

//==============================================================================
class Subscription final
{
public:

  Subscription(
      rclcpp::Node& node,
      TopicSubscriberSystem::SubscriptionCallback callback,
      const std::string& topic_name,
      const rmw_qos_profile_t& qos_profile)
    : _topic(topic_name),
      _callback(std::move(callback))
  {
    _message = initialize();

    _subscription = node.create_subscription<Ros2_Msg>(
          topic_name,
          [=](Ros2_Msg::UniquePtr msg) { this->subscription_callback(*msg); },
          qos_profile);
  }

private:

  void subscription_callback(const Ros2_Msg& msg)
  {
    convert_to_soss(msg, _message);
    _callback(_message);
  }

  // Hang onto the topic name, because it's needed by the SOSS callback
  const std::string _topic;

  // Save the SOSS callback that we were given by the soss-ros2 plugin
  TopicSubscriberSystem::SubscriptionCallback _callback;

  // Save a pre-initialized copy of the message so that we don't need to
  // allocate and deallocate more than necessary
  soss::Message _message;

  // Hang onto the subscription handle to make sure the connection to the topic
  // stays alive
  rclcpp::Subscription<Ros2_Msg>::SharedPtr _subscription;

};

//==============================================================================
std::shared_ptr<void> subscribe(
    rclcpp::Node& node,
    const std::string& topic_name,
    TopicSubscriberSystem::SubscriptionCallback callback,
    const rmw_qos_profile_t& qos_profile)
{
  return std::make_shared<Subscription>(
        node, std::move(callback), topic_name, qos_profile);
}

namespace {

SubscriptionFactoryRegistrar register_subscriber(g_msg_name, &subscribe);

} // anonymous namespace

//==============================================================================
class Publisher final : public virtual soss::TopicPublisher
{
public:

  Publisher(
      rclcpp::Node& node,
      const std::string& topic_name,
      const rmw_qos_profile_t& qos_profile)
  {
    _publisher = node.create_publisher<Ros2_Msg>(
          topic_name, qos_profile);
  }

  bool publish(const soss::Message& message) override
  {
    Ros2_Msg ros2_msg;
    convert_to_ros2(message, ros2_msg);

    _publisher->publish(ros2_msg);
    return true;
  }

private:

  rclcpp::Publisher<Ros2_Msg>::SharedPtr _publisher;

};

//==============================================================================
std::shared_ptr<soss::TopicPublisher> make_publisher(
    rclcpp::Node& node,
    const std::string& topic_name,
    const rmw_qos_profile_t& qos_profile)
{
  return std::make_shared<Publisher>(node, topic_name, qos_profile);
}

namespace {

PublisherFactoryRegistrar register_publisher(g_msg_name, &make_publisher);

}

} // namespace convert__test_msgs__msg__bounded_array_nested
} // namespace ros2
} // namespace soss
