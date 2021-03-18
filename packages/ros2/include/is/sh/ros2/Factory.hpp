/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _IS_SH_ROS2_FACTORY_HPP_
#define _IS_SH_ROS2_FACTORY_HPP_

#include <is/systemhandle/SystemHandle.hpp>

#include <is/sh-ros2/export.hpp> // TODO (@jamoralp): convert this to is/sh/ros2

#include <rclcpp/node.hpp>

#include <functional>
#include <memory>

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

// TODO(@jamoralp) Doxygen docs
//==============================================================================
class IS_SH_ROS2_API Factory
{
public:

    /// \brief Factory is a singleton class in order to make it accessible to the
    /// plugin libraries that define subscriber/publisher/client/server code for
    /// each message and service type. These plugin libraries will register their
    /// factories with this singleton so that
    static Factory& instance();

    /// \brief Signature for subscription factories
    using TypeFactory =
            std::function<eprosima::xtypes::DynamicType::Ptr()>;

    /// \brief Register a subscription factory
    void register_type_factory(
            const std::string& message_type,
            TypeFactory&& type_factory);

    /// \brief Create a subscription using the factory for the relevant message
    /// type
    eprosima::xtypes::DynamicType::Ptr create_type(
            const std::string& message_type);


    /// \brief Signature for subscription factories
    using SubscriptionFactory =
            std::function<std::shared_ptr<void>(
                        rclcpp::Node& node,
                        const std::string& topic_name,
                        const eprosima::xtypes::DynamicType& message_type,
                        TopicSubscriberSystem::SubscriptionCallback && callback,
                        const rmw_qos_profile_t& qos_profile)>;

    /// \brief Register a subscription factory
    void register_subscription_factory(
            const std::string& message_type,
            SubscriptionFactory&& subscriber_factory);

    /// \brief Create a subscription using the factory for the relevant message
    /// type
    std::shared_ptr<void> create_subscription(
            const eprosima::xtypes::DynamicType& message_type,
            rclcpp::Node& node,
            const std::string& topic_name,
            TopicSubscriberSystem::SubscriptionCallback&& callback,
            const rmw_qos_profile_t& qos_profile);


    /// \brief Signature for publisher factories
    using PublisherFactory =
            std::function<std::shared_ptr<TopicPublisher>(
                        rclcpp::Node& node,
                        const std::string& topic_name,
                        const rmw_qos_profile_t& qos_profile)>;

    /// \brief Register a publisher factory
    void register_publisher_factory(
            const std::string& message_type,
            PublisherFactory&& publisher_factory);

    /// \brief Create a publisher using the factory for the relevant message type
    std::shared_ptr<TopicPublisher> create_publisher(
            const eprosima::xtypes::DynamicType& message_type,
            rclcpp::Node& node,
            const std::string& topic_name,
            const rmw_qos_profile_t& qos_profile);


    /// \brief Signature for client proxy factories
    using ServiceClientFactory =
            std::function<std::shared_ptr<ServiceClient>(
                        rclcpp::Node& node,
                        const std::string& service_name,
                        const ServiceClientSystem::RequestCallback& callback,
                        const rmw_qos_profile_t& qos_profile)>;

    /// \brief Register a client proxy factory
    void register_client_proxy_factory(
            const std::string& service_type,
            ServiceClientFactory&& client_proxy_factory);

    /// \brief Create a client proxy using the factory for the relevant service
    /// type
    std::shared_ptr<ServiceClient> create_client_proxy(
            const std::string& service_type,
            rclcpp::Node& node,
            const std::string& service_name,
            const ServiceClientSystem::RequestCallback& callback,
            const rmw_qos_profile_t& qos_profile);


    /// \brief Signature for server proxy factories
    using ServiceProviderFactory =
            std::function<std::shared_ptr<ServiceProvider>(
                        rclcpp::Node& node,
                        const std::string& service_name,
                        const rmw_qos_profile_t& qos_profile)>;

    /// \brief Register a server proxy factory
    void register_server_proxy_factory(
            const std::string& service_type,
            ServiceProviderFactory&& server_proxy_factory);

    /// \brief Create a server proxy using the factory for the relevant service
    /// type
    std::shared_ptr<ServiceProvider> create_server_proxy(
            const std::string& service_type,
            rclcpp::Node& node,
            const std::string& service_name,
            const rmw_qos_profile_t& qos_profile);

private:

    /// \brief The private constructor ensures that this class can only be
    /// constructed using the instance() function.
    Factory() = default;

    class Implementation;
    /// \brief Pointer to the implementation of the Factory class.
    std::unique_ptr<Implementation> _pimpl;

};

//==============================================================================
template<typename FactoryType, void(Factory::* register_func)(const std::string&, FactoryType &&)>
struct FactoryRegistrar
{
    FactoryRegistrar(
            const std::string& type,
            FactoryType&& factory)
    {
        (Factory::instance().*register_func)(type, std::move(factory));
    }

};

//==============================================================================
using TypeFactoryRegistrar =
        FactoryRegistrar<Factory::TypeFactory, &Factory::register_type_factory>;

//==============================================================================
using SubscriptionFactoryRegistrar =
        FactoryRegistrar<Factory::SubscriptionFactory, &Factory::register_subscription_factory>;

//==============================================================================
using PublisherFactoryRegistrar =
        FactoryRegistrar<Factory::PublisherFactory, &Factory::register_publisher_factory>;

//==============================================================================
using ServiceClientFactoryRegistrar =
        FactoryRegistrar<Factory::ServiceClientFactory, &Factory::register_client_proxy_factory>;

//==============================================================================
using ServiceProviderFactoryRegistrar =
        FactoryRegistrar<Factory::ServiceProviderFactory, &Factory::register_server_proxy_factory>;

} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima

#endif //  _IS_SH_ROS2_FACTORY_HPP_
