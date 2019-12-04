// generated from soss/packages/ros2/resource/soss__ros2__message.cpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating soss/rosidl/ros2/<package>/src/msg/convert__msg__<msg>.cpp files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg' or 'srv'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################

@{
camelcase_msg_type = spec.base_type.type
underscore_msg_type = get_header_filename_from_msg_name(camelcase_msg_type)

namespace_parts = [
    'convert', spec.base_type.pkg_name, 'msg', underscore_msg_type]
namespace_variable = '__'.join(namespace_parts)

conversion_dependency = 'soss/rosidl/ros2/{}/msg/convert__msg__{}.hpp'.format(
    spec.base_type.pkg_name, camelcase_msg_type)
}@

// Include the API header for this message type
#include <@(conversion_dependency)>
// Include the Factory header so we can add this message type to the Factory
#include <soss/ros2/Factory.hpp>

// Include the Node API so we can subscribe and advertise
#include <rclcpp/node.hpp>

namespace soss {
namespace ros2 {
namespace @(namespace_variable) {

//==============================================================================
namespace {
TypeFactoryRegistrar register_type(g_msg_name, &type);
} // anonymous namespace

//==============================================================================
class Subscription final
{
public:

  Subscription(
      rclcpp::Node& node,
      TopicSubscriberSystem::SubscriptionCallback callback,
      const std::string& topic_name,
      const xtypes::DynamicType& message_type,
      const rmw_qos_profile_t& qos_profile)
    : _callback(std::move(callback))
    , _message_type(message_type)
  {

#ifndef RCLCPP__QOS_HPP_
    _subscription = node.create_subscription<Ros2_Msg>(
          topic_name,
          [=](Ros2_Msg::UniquePtr msg) { this->subscription_callback(*msg); },
          qos_profile);
#else
    _subscription = node.create_subscription<Ros2_Msg>(
          topic_name,
          rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)),
          [=](Ros2_Msg::UniquePtr msg) { this->subscription_callback(*msg); });
#endif
  }

private:

  void subscription_callback(const Ros2_Msg& msg)
  {
    xtypes::DynamicData data(_message_type);
    convert_to_xtype(msg, data);
    _callback(data);
  }

  // Save the SOSS callback that we were given by the soss-ros2 plugin
  TopicSubscriberSystem::SubscriptionCallback _callback;

  const xtypes::DynamicType& _message_type;

  // Hang onto the subscription handle to make sure the connection to the topic
  // stays alive
  rclcpp::Subscription<Ros2_Msg>::SharedPtr _subscription;

};

//==============================================================================
std::shared_ptr<void> subscribe(
    rclcpp::Node& node,
    const std::string& topic_name,
    const xtypes::DynamicType& message_type,
    TopicSubscriberSystem::SubscriptionCallback callback,
    const rmw_qos_profile_t& qos_profile)
{
  return std::make_shared<Subscription>(
        node, std::move(callback), topic_name, message_type, qos_profile);
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
#ifndef RCLCPP__QOS_HPP_
    // If the rclcpp/qos.hpp header does not exist, then we assume that we
    // are in crystal
    _publisher = node.create_publisher<Ros2_Msg>(topic_name, qos_profile);
#else
    _publisher = node.create_publisher<Ros2_Msg>(
          topic_name,
          rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)));
#endif
  }

  bool publish(const xtypes::DynamicData& message) override
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

} // namespace @(namespace_variable)
} // namespace ros2
} // namespace soss
