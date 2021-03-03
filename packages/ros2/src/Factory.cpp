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

#include <soss/ros2/Factory.hpp>

#include <unordered_map>

namespace soss {
namespace ros2 {

//==============================================================================
class Factory::Implementation
{
public:

    //============================================================================
    void register_type_factory(
            const std::string& message_type,
            TypeFactory type_factory)
    {
        _type_factories[message_type] = std::move(type_factory);
    }

    //============================================================================
    xtypes::DynamicType::Ptr create_type(
            const std::string& message_type)
    {
        auto it = _type_factories.find(message_type);
        if (it == _type_factories.end())
        {
            return xtypes::DynamicType::Ptr();
        }

        return it->second();
    }

    //============================================================================
    void register_subscription_factory(
            const std::string& message_type,
            SubscriptionFactory subscriber_factory)
    {
        _subscription_factories[message_type] = std::move(subscriber_factory);
    }

    //============================================================================
    std::shared_ptr<void> create_subscription(
            const xtypes::DynamicType& message_type,
            rclcpp::Node& node,
            const std::string& topic_name,
            TopicSubscriberSystem::SubscriptionCallback callback,
            const rmw_qos_profile_t& qos_profile)
    {
        auto it = _subscription_factories.find(message_type.name());
        if (it == _subscription_factories.end())
        {
            return nullptr;
        }

        return it->second(node, topic_name, message_type, std::move(callback), qos_profile);
    }

    //============================================================================
    void register_publisher_factory(
            const std::string& message_type,
            PublisherFactory publisher_factory)
    {
        _publisher_factories[message_type] = std::move(publisher_factory);
    }

    //============================================================================
    std::shared_ptr<TopicPublisher> create_publisher(
            const xtypes::DynamicType& message_type,
            rclcpp::Node& node,
            const std::string& topic_name,
            const rmw_qos_profile_t& qos_profile)
    {
        auto it = _publisher_factories.find(message_type.name());
        if (it == _publisher_factories.end())
        {
            return nullptr;
        }

        return it->second(node, topic_name, qos_profile);
    }

    //============================================================================
    void register_client_proxy_factory(
            const std::string& service_type,
            ServiceClientFactory client_proxy_factory)
    {
        _client_proxy_factories[service_type] = std::move(client_proxy_factory);
    }

    //============================================================================
    std::shared_ptr<ServiceClient> create_client_proxy(
            const std::string& service_type,
            rclcpp::Node& node,
            const std::string& service_name,
            const ServiceClientSystem::RequestCallback& callback,
            const rmw_qos_profile_t& qos_profile)
    {
        auto it = _client_proxy_factories.find(service_type);
        if (it == _client_proxy_factories.end())
        {
            return nullptr;
        }

        return it->second(node, service_name, callback, qos_profile);
    }

    //============================================================================
    void register_server_proxy_factory(
            const std::string& service_type,
            ServiceProviderFactory server_proxy_factory)
    {
        _server_proxy_factories[service_type] = server_proxy_factory;
    }

    //============================================================================
    std::shared_ptr<ServiceProvider> create_server_proxy(
            const std::string& service_type,
            rclcpp::Node& node,
            const std::string& service_name,
            const rmw_qos_profile_t& qos_profile)
    {
        auto it = _server_proxy_factories.find(service_type);
        if (it == _server_proxy_factories.end())
        {
            return nullptr;
        }

        return it->second(node, service_name, qos_profile);
    }

private:

    std::unordered_map<std::string, TypeFactory> _type_factories;
    std::unordered_map<std::string, SubscriptionFactory> _subscription_factories;
    std::unordered_map<std::string, PublisherFactory> _publisher_factories;
    std::unordered_map<std::string, ServiceClientFactory> _client_proxy_factories;
    std::unordered_map<std::string, ServiceProviderFactory> _server_proxy_factories;

};

//==============================================================================
Factory& Factory::instance()
{
    static Factory factory;
    return factory;
}

//==============================================================================
void Factory::register_type_factory(
        const std::string& message_type,
        TypeFactory type_factory)
{
    _pimpl->register_type_factory(
        message_type, std::move(type_factory));
}

//==============================================================================
xtypes::DynamicType::Ptr Factory::create_type(
        const std::string& message_type)
{
    return _pimpl->create_type(message_type);
}

//==============================================================================
void Factory::register_subscription_factory(
        const std::string& message_type,
        SubscriptionFactory subscriber_factory)
{
    _pimpl->register_subscription_factory(
        message_type, std::move(subscriber_factory));
}

//==============================================================================
std::shared_ptr<void> Factory::create_subscription(
        const xtypes::DynamicType& message_type,
        rclcpp::Node& node,
        const std::string& topic_name,
        TopicSubscriberSystem::SubscriptionCallback callback,
        const rmw_qos_profile_t& qos_profile)
{
    return _pimpl->create_subscription(
        message_type, node, topic_name, std::move(callback), qos_profile);
}

//==============================================================================
void Factory::register_publisher_factory(
        const std::string& message_type,
        PublisherFactory publisher_factory)
{
    _pimpl->register_publisher_factory(
        message_type, std::move(publisher_factory));
}

//==============================================================================
std::shared_ptr<TopicPublisher> Factory::create_publisher(
        const xtypes::DynamicType& message_type,
        rclcpp::Node& node,
        const std::string& topic_name,
        const rmw_qos_profile_t& qos_profile)
{
    return _pimpl->create_publisher(
        message_type, node, topic_name, qos_profile);
}

//==============================================================================
void Factory::register_client_proxy_factory(
        const std::string& service_type,
        ServiceClientFactory client_proxy_factory)
{
    _pimpl->register_client_proxy_factory(
        service_type, std::move(client_proxy_factory));
}

//==============================================================================
std::shared_ptr<ServiceClient> Factory::create_client_proxy(
        const std::string& service_type,
        rclcpp::Node& node,
        const std::string& service_name,
        const ServiceClientSystem::RequestCallback& callback,
        const rmw_qos_profile_t& qos_profile)
{
    return _pimpl->create_client_proxy(
        service_type, node, service_name, callback, qos_profile);
}

//==============================================================================
void Factory::register_server_proxy_factory(
        const std::string& service_type,
        ServiceProviderFactory server_proxy_factory)
{
    _pimpl->register_server_proxy_factory(
        service_type, std::move(server_proxy_factory));
}

//==============================================================================
std::shared_ptr<ServiceProvider> Factory::create_server_proxy(
        const std::string& service_type,
        rclcpp::Node& node,
        const std::string& service_name,
        const rmw_qos_profile_t& qos_profile)
{
    return _pimpl->create_server_proxy(
        service_type, node, service_name, qos_profile);
}

//==============================================================================
Factory::Factory()
    : _pimpl(new Implementation)
{
    // Do nothing
}

} // namespace ros2
} // namespace soss
