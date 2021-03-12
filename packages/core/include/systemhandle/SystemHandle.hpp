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

#ifndef _IS_CORE_SYSTEMHANDLE_HPP_
#define _IS_CORE_SYSTEMHANDLE_HPP_

#include <is/core/systemhandle/SystemHandleFactory.hpp>
#include <is/core/Message.hpp>

#include <yaml-cpp/yaml.h>

#include <set>
#include <string>
#include <vector>
#include <functional>

namespace eprosima {
namespace is {
namespace core {

using TypeRegistry = std::map<std::string, xtypes::DynamicType::Ptr>;

/**
 * @brief Call this macro in a .cpp file of your middleware's plugin library,
 *        so that the *Integration Service* can find your SystemHandle implementation
 *         when your plugin library gets dynamically loaded. For example:
 *
 *        `IS_REGISTER_SYSTEM("my_middleware", my::middleware::SystemHandle)`
 *
 *        The first argument should be a string representing the name of the
 *        middleware. This should match the name in the `system:` dictionary of your
 *        soss configuration file. Each middleware should have a unique name.
 *
 *        The second argument should be the literal type (not a string) of the class
 *        that implements SystemHandle in your plugin library.
 */
#define IS_REGISTER_SYSTEM(middleware_name_str, SystemType) \
    DETAIL_IS_REGISTER_SYSTEM(middleware_name_str, SystemType)

/**
 * @class SystemHandle
 *        It is the base interface class for all middleware systems.
 *
 *        All middleware systems that want to interact with the *Integration Service*
 *        must implement, at least, this interface.
 *
 *        Depending on the type of middleware, it should also implement
 *        the derived classes, using multiple virtual inheritance:
 *
 *        - TopicSubscriberSystem: provides with subscribing capabilities.
 *        - TopicPublisherSystem: provides with publishing capabilities.
 *        - ServiceClientSystem: allows to manage middleware service clients.
 *        - ServiceProviderSystem: allows to manage middleware service servers.
 *
 *        A SystemHandle implementing the four interfaces described above is
 *        called a FullSystem, and it is usually the base class used for implementing
 *        a middleware plugin for the *Integration Service*.
 */
class SystemHandle
{
public:

    /**
     * @brief Default constructor.
     */
    SystemHandle() = default;

    /**
     * @brief SystemHandle shall not be copy constructible.
     */
    SystemHandle(
            const SystemHandle& /*other*/) = delete;

    /**
     * @brief SystemHandle shall not be copy assignable.
     */
    SystemHandle& operator =(
            const SystemHandle& /*other*/) = delete;

    /**
     * @brief SystemHandle shall not be move constructible.
     */
    SystemHandle(
            SystemHandle&&) = delete;

    /**
     * @brief SystemHandle shall not be move assignable.
     */
    SystemHandle& operator =(
            SystemHandle&&) = delete;

    /**
     * @brief Destructor.
     */
    virtual ~SystemHandle() = default;

    /**
     * @brief Configure the *Integration Service* handle for this middleware's system.
     *
     * @param[in] types The set of types (messages and services)
     *            that this middleware needs to support.
     *
     *            The SystemHandle must register this types into the TypeRegistry,
     *            using for that the storage class SystemHandleInfo.
     *
     * @param[in] configuration The configuration specific to this SystemHandle,
     *            as described in the user-provided `YAML` input file.
     *
     *            See the specific SystemHandle implementation documentation for
     *            a list of accepted configuration parameters for each middleware.
     *
     * @param[in] type_registry The set of type definitions that this middleware
     *            is able to support.
     *
     * @returns `true` if the configuration process was successful, or `false` otherwise.
     */
    virtual bool configure(
            const RequiredTypes& types,
            const YAML::Node& configuration,
            TypeRegistry& type_registry) = 0;

    /**
     * @brief Method that allows to check if a SystemHandle is correctly working.
     *
     * @returns `true` if the SystemHandle is under normal behaviour, or `false` otherwise.
     */
    virtual bool okay() const = 0;

    /**
     * @brief Boolean operator overload. Implicit conversion, same as `okay()`.
     *
     * @returns `true` if the SystemHandle is under normal behaviour, or `false` otherwise.
     */
    inline operator bool() const
    {
        return okay();
    }
    /**
     * @brief Tell the SystemHandle to spin once, e.g. read through its subscriptions.
     *
     * @returns `true` if the SystemHandle is still working; `false` otherwise.
     */
    virtual bool spin_once() = 0;
};

/**
 * @class TopicSubscriberSystem
 *        Extends the SystemHandle class with subscription capabilities.
 */
class TopicSubscriberSystem : public virtual SystemHandle
{
public:

    using SubscriptionCallback = std::function<void (const xtypes::DynamicData& message)>;

    /**
     * @brief Constructor.
     */
    TopicSubscriberSystem() = default;

    /**
     * @brief Destructor.
     */
    virtual ~TopicSubscriberSystem() = default;

    /**
     * @brief Have this SystemHandle instance subscribed to a topic.
     *
     * @param[in] topic_name Name of the topic to get subscribed to.
     *
     * @param[in] message_type Message type that this topic should expect to receive.
     *
     * @param[in] callback The callback that should be triggered when a message comes in.
     *
     * @param[in] configuration A `YAML` node containing any middleware-specific
     *            configuration information for this subscription. This may be an empty node.
     *
     * @returns `true` if subscription was successfully established, or `false` otherwise.
     */
    virtual bool subscribe(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            SubscriptionCallback callback,
            const YAML::Node& configuration) = 0;
};

/**
 * @class TopicPublisher
 *        This is the abstract interface for objects that can act as
 *        publisher proxies.
 *
 *        These objects will be created by the *Integration Service* as a
 *        bridge between the common data representation (`eprosima::xtypes`) and the
 *        *user subscription application*, when we want to publish data from one
 *        middleware to another.
 *
 *        These objects should be generated by the TopicPublisherSystem
 *        `advertise()` method.
 */
class TopicPublisher
{
public:

    /**
     * @brief Constructor.
     */
    TopicPublisher() = default;

    /**
     * @brief Destructor.
     */
    virtual ~TopicPublisher() = default;

    /**
     * @brief Publish to a topic.
     *
     * @param[in] topic_name Name of the topic to publish to.
     *
     * @param[in] message DynamicData that is being published.
     *
     * @returns `true` if the data was correctly published, or `false` otherwise.
     */
    virtual bool publish(
            const xtypes::DynamicData& message) = 0;

};

/**
 * @class TopicPublisherSystem
 *        This class extends the SystemHandle class with publishing capabilities.
 */
class TopicPublisherSystem : public virtual SystemHandle
{
public:

    /**
     * @brief Constructor.
     */
    TopicPublisherSystem() = default;

    /**
     * @brief Destructor.
     */
    virtual ~TopicPublisherSystem() = default;

    /**
     * @brief Advertise the ability to publish to a topic.
     *
     * @param[in] topic_name Name of the topic to advertise.
     *
     * @param[in] message_type Message type that this entity will publish.
     *
     * @param[in] configuration A `YAML` node containing any
     *            middleware-specific configuration information
     *            for this publisher. This may be an empty node.
     *
     * @returns `true` if the advertisement was successful, or `false` otherwise.
     */
    virtual std::shared_ptr<TopicPublisher> advertise(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            const YAML::Node& configuration) = 0;
};

/**
 * @class TopicSystem.
 *        It is the conjunction of TopicPublisherSystem and TopicSubscriberSystem.
 *        Allows to create a middleware library for *Integration Service*
 *        fully compatible with the publish/subscribe paradigm.
 */
class TopicSystem
    : public virtual TopicPublisherSystem
    , public virtual TopicSubscriberSystem
{
public:

    /**
     * @brief Constructor.
     */
    TopicSystem() = default;

    /**
     * @brief Destructor.
     */
    virtual ~TopicSystem() = default;
};

/**
 * @class ServiceClient
 *        This is the abstract interface for objects that can act as client proxies.
 *
 *        This class is different from ServiceClientSystem, because ServiceClientSystem
 *        is the interface for SystemHandle libraries that are able to support
 *        client proxies, whereas ServiceClient is the interface for the
 *        client proxy objects themselves.
 *
 *        This class, when overriden by the specific middleware implementation, will
 *        tipically contain a `middleware::server` object, so that `receive_response`
 *        can fetch the response sent by the *user server application* (usually, implemented
 *        using a different middleware) and serve this response to the *target user client
 *        application*, which will receive the final response by means of the internal server
 *        created by this ServiceClient.
 */
class ServiceClient
{
public:

    /**
     * @brief Constructor.
     */
    class ServiceClient() = default;

    /**
     * @brief Destructor.
     */
    virtual ~ServiceClient() = default;

    /**
     * @brief Receive the response of a service request.
     *
     * @attention Services are assumed to all be asynchronous (non-blocking), so
     *            this function may be called by multiple threads at once.
     *
     *            ServiceClient implementers must make sure that they can handle
     *            multiple simultaneous calls to this function.
     *
     * @param[in] call_handle The handle that was given to the call by this ServiceClient.
     *            The usage of the handle is determined by the ServiceClient
     *            implementation.
     *
     *            Typically, `receive_response` will cast this handle into
     *            a useful object type that contains information on where to send the
     *            service response message.
     *
     * @param[in] response The message that represents the response from the service.
     */
    virtual void receive_response(
            std::shared_ptr<void> call_handle,
            const xtypes::DynamicData& response) = 0;
};

/**
 * @class ServiceClientSystem
 *        This class extends the SystemHandle class with service client handling capabilities.
 */
class ServiceClientSystem : public virtual SystemHandle
{
public:

    /**
     * @brief Signature of the callback that gets triggered when a client has made a request.
     */
    using RequestCallback =
            std::function<void (
                        const xtypes::DynamicData& request,
                        ServiceClient& client,
                        std::shared_ptr<void> call_handle)>;

    /**
     * @brief Constructor.
     */
    class ServiceClientSystem() = default;

    /**
     * @brief Destructor.
     */
    virtual ~ServiceClientSystem() = default;

    /**
     * @brief Create a proxy for a client application.
     *
     * @param[in] service_name Name of the service for this client proxy to listen to.
     *
     * @param[in] service_type Service request and reply type to expect.
     *
     * @param[in] callback The callback that should be used when a request comes in
     *            from the middleware.
     *
     * @param[in] configuration A `YAML` node containing any middleware-specific
     *            configuration information for this service client.
     *            This may be an empty node.
     *
     * @returns `true` if a client proxy could be created, or `false` otherwise.
     */
    virtual bool create_client_proxy(
            const std::string& service_name,
            const xtypes::DynamicType& service_type,
            RequestCallback callback,
            const YAML::Node& configuration)
    {
        (void)service_name, (void)service_type, (void)callback, (void)configuration;
        return false;
    }

    /**
     * @brief Create a proxy for a client application.
     *
     * @param[in] service_name Name of the service for this client to listen to.
     *
     * @param[in] request_type Type of service request to expect.
     *
     * @param[in] reply_type Type of service reply to expect.
     *
     * @param[in] callback The callback that should be used when a request comes in
     *            from the middleware.
     *
     * @param[in] configuration A `YAML` node containing any middleware-specific
     *            configuration information for this service client.
     *            This may be an empty node.
     *
     * @returns `true` if a client proxy could be created, or `false` otherwise.
     */
    virtual bool create_client_proxy(
            const std::string& service_name,
            const xtypes::DynamicType& request_type,
            const xtypes::DynamicType& reply_type,
            RequestCallback callback,
            const YAML::Node& configuration)
    {
        (void)reply_type;
        return create_client_proxy(service_name, request_type, callback, configuration);
    }

};

/**
 * @class ServiceProvider
 *        This is the abstract interface for objects that can act as service server proxies.
 *
 *        This class is different from ServiceProviderSystem, because ServiceProviderSystem
 *        is the interface for SystemHandle libraries that are able to support
 *        service server proxies, whereas ServiceProvider is the interface for the
 *        service server proxy objects themselves.
 *
 *        This class, when overriden by the specific middleware implementation, will
 *        tipically contain a `middleware::client` object that will actually send the
 *        request to the *user server application*.
 *        After processing the request by means of the `call_service` method,
 *        thans to the associated ServiceClient entity, `receive_response`
 *        will be called, to serve the response to the *user client application* (tipically,
 *        implemented using a different middleware, which justifies the use of the
 *        *Integration Service* to interconnect them).
 */
class ServiceProvider
{
public:

    /**
     * @brief Constructor.
     */
    class ServiceProvider() = default;

    /**
     * @brief Destructor.
     */
    virtual ~ServiceProvider() = default;

    // TODO (@jamoralp): can we come up with a way to avoid calling receive_response
    // from the call_service method? This would make the system handle implementation more intuitive and easier.

    /**
     * @brief Call a service.
     *
     * @attention It is important that this function:
     *
     *            1. Is **non-blocking**.
     *            2. Calls `client.receive_response()` when the service finishes.
     *
     * @param[in] request Request message for the service.
     *
     * @param[in,out] client The proxy for the client that is making the request.
     *
     * @param[in] call_handle A handle for the call.
     *
     *            The usage of this handle is determined by the ServiceClient implementation.
     *            The ServiceProvider should not attempt to cast or modify it in any way;
     *            it should only be passed back to the ServiceClient later on,
     *            when `receive_response()` is called.
     *
     */
    virtual void call_service(
            const xtypes::DynamicData& request,
            ServiceClient& client,
            std::shared_ptr<void> call_handle) = 0;
};

/**
 * @class ServiceProviderSystem
 *        This class extends the SystemHandle class with service server handling capabilities.
 */
class ServiceProviderSystem : public virtual SystemHandle
{
public:

    /**
     * @brief Constructor.
     */
    class ServiceProviderSystem() = default;

    /**
     * @brief Destructor.
     */
    virtual ~ServiceProviderSystem() = default;

    /**
     * @brief Create a proxy for a service server.
     *
     * @param[in] service_name Name of the service to offer.
     *
     * @param[in] service_type Type of service being offered.
     *
     * @param[in] configuration A `YAML` node containing any middleware-specific
     *            configuration information for this service provider.
     *            This may be an empty node.
     *
     * @returns `true` if the middleware's SystemHandle can offer this service,
     *          or `false` otherwise.
     */
    virtual std::shared_ptr<ServiceProvider> create_service_proxy(
            const std::string& service_name,
            const xtypes::DynamicType& service_type,
            const YAML::Node& configuration)
    {
        (void)service_name, (void)service_type, (void)configuration;
        return nullptr;
    }

    /**
     * @brief Create a proxy for a service server.
     *
     * @param[in] service_name Name of the service to offer.
     *
     * @param[in] request_type Type of service request being offered.
     *
     * @param[in] reply_type Type of service reply being offered.
     *
     * @param[in] configuration A `YAML` node containing any middleware-specific
     *            configuration information for this service provider.
     *            This may be an empty node.
     *
     * @returns `true` if the middleware's SystemHandle can offer this service,
     *          or `false` otherwise.
     */
    virtual std::shared_ptr<ServiceProvider> create_service_proxy(
            const std::string& service_name,
            const xtypes::DynamicType& request_type,
            const xtypes::DynamicType& reply_type,
            const YAML::Node& configuration)
    {
        (void)request_type;
        return create_service_proxy(service_name, reply_type, configuration);
    }

};

/**
 * @class ServiceSystem.
 *        It is the conjunction of ServiceProviderSystem and ServiceClientSystem.
 *        Allows to create a middleware library for *Integration Service*
 *        fully compatible with the request/reply paradigm.
 */
class ServiceSystem
    : public virtual ServiceClientSystem
    , public virtual ServiceProviderSystem
{
public:

    /**
     * @brief Constructor.
     */
    ServiceSystem() = default;

    /**
     * @brief Destructor.
     */
    virtual ~ServiceSystem() = default;
};

/**
 * @class FullSystem
 *        It is the conjunction of ServiceSystem and TopicSystem.
 *        It allows to define a whole middleware, in terms of both publish/subscribe
 *        and request/reply paradigms.
 *
 *        Usually, most middleware plugins for the *Integration Service* will inherit
 *        from this class.
 */
class FullSystem
    : public virtual TopicSystem
    , public virtual ServiceSystem
{
public:

    /**
     * @brief Constructor.
     */
    FullSystem() = default;

    /**
     * @brief Destructor.
     */
    virtual ~FullSystem() = default;
};

} //  namespace core
} //  namespace is
} //  namespace eprosima

#endif //  _IS_CORE_SYSTEMHANDLE_HPP_
