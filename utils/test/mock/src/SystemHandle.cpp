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

#include <is/sh/mock/api.hpp>

#include <is/systemhandle/SystemHandle.hpp>

#include <iostream>
#include <thread>

// TODO(@jamoralp): Document doxygen
// TODO(@jamoralp): Add logger
namespace eprosima {
namespace is {
namespace sh {
namespace mock {

class MockServiceClient;

namespace {

//==============================================================================
class Implementation
{
public:

    static Implementation& get()
    {
        static Implementation impl;
        return impl;
    }

    // Note: This is a map from topic name to message types. This mock middleware
    // supports multiple message types per topic.
    using Channels = std::map<std::string, std::set<std::string> >;
    Channels subscriptions;
    Channels publishers;
    Channels clients;
    Channels services;

    std::map<std::string, TopicSubscriberSystem::SubscriptionCallback*> is_subscription_callbacks;

    std::map<std::string, ServiceClientSystem::RequestCallback*> is_request_callbacks;

    std::map<std::string, std::vector<MockSubscriptionCallback> > mock_subscriptions;
    std::map<std::string, MockServiceCallback> mock_services;

    // We deposit references to clients into this vector to guarantee that their
    // promises don't get broken and the user has a chance to read them. This is a
    // terribly memory inefficient approach, and it's essentially a memory leak,
    // but the mock middleware is only meant for testing purposes, so this isn't
    // really a concern.
    std::vector<std::shared_ptr<MockServiceClient> > mock_clients;

private:

    Implementation() = default;

};

Implementation& impl()
{
    return Implementation::get();
}

} // anonymous namespace

//==============================================================================
class Publisher : public virtual TopicPublisher
{
public:

    Publisher(
            const std::string& topic)
        : _topic(topic)
    {
        // Does nothing
    }

    bool publish(
            const eprosima::xtypes::DynamicData& message) override
    {
        // Tests that we have advertised this topic
        const auto pub_it = impl().publishers.find(_topic);
        if (pub_it == impl().publishers.end() || pub_it->second.empty())
        {
            // We'll go ahead and throw an exception here, because this situation
            // should be considered a bug in Integration Service
            throw std::runtime_error(
                      "Integration Service attempted to publish to a topic/message pair "
                      "that it didn't advertise");
        }


        const auto it = impl().mock_subscriptions.find(_topic);
        if (it == impl().mock_subscriptions.end())
        {
            return true;
        }

        for (const auto& callback : it->second)
        {
            callback(message);
        }

        return true;
    }

    const std::string _topic;

};

//==============================================================================
class Server : public virtual ServiceProvider
{
public:

    Server(
            const std::string& service)
        : _service(service)
    {
        // Do nothing
    }

    void call_service(
            const eprosima::xtypes::DynamicData& request,
            ServiceClient& client,
            std::shared_ptr<void> call_handle) override
    {
        bool only_service = impl().mock_services.count(_service) > 0;
        const auto it =
                (only_service
                ? impl().mock_services.find(_service)
                : impl().mock_services.find(_service + "_" + request.type().name()));

        if (it == impl().mock_services.end())
        {
            throw std::runtime_error(
                      "mock middleware was never given the requested service: "
                      + _service);
        }

        eprosima::xtypes::DynamicData response = it->second(request);
        client.receive_response(call_handle, response);
    }

    const std::string _service;
};

//==============================================================================
class SystemHandle : public virtual FullSystem
{
public:

    bool configure(
            const core::RequiredTypes&,
            const YAML::Node&,
            TypeRegistry&) override
    {
        return true;
    }

    bool okay() const override
    {
        // We're always okay
        return true;
    }

    bool spin_once() override
    {
        // We're always spinning. We'll put a sleep here so that this thread doesn't
        // do too heavy of a dead spin.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }

    bool subscribe(
            const std::string& topic_name,
            const eprosima::xtypes::DynamicType& message_type,
            TopicSubscriberSystem::SubscriptionCallback* callback,
            const YAML::Node& /*configuration*/) override
    {
        impl().subscriptions[topic_name].insert(message_type.name());
        impl().is_subscription_callbacks[topic_name] = callback;
        return true;
    }

    bool is_internal_message(
            void* /*filter_message*/)
    {
        return false;
    }

    std::shared_ptr<TopicPublisher> advertise(
            const std::string& topic_name,
            const eprosima::xtypes::DynamicType& message_type,
            const YAML::Node& /*configuration*/) override
    {
        impl().publishers[topic_name].insert(message_type.name());
        return std::make_shared<Publisher>(topic_name);
    }

    bool create_client_proxy(
            const std::string& service_name,
            const eprosima::xtypes::DynamicType& service_type,
            RequestCallback* callback,
            const YAML::Node& /*configuration*/) override
    {
        impl().clients[service_name].insert(service_type.name());
        impl().is_request_callbacks[service_name] = callback;
        return true;
    }

    std::shared_ptr<ServiceProvider> create_service_proxy(
            const std::string& service_name,
            const eprosima::xtypes::DynamicType& service_type,
            const YAML::Node& /*configuration*/) override
    {
        impl().services[service_name].insert(service_type.name());
        return std::make_shared<Server>(service_name);
    }

};

//==============================================================================
bool publish_message(
        const std::string& topic,
        const eprosima::xtypes::DynamicData& msg)
{
    const auto it = impl().subscriptions.find(topic);
    if (it == impl().subscriptions.end() ||
            it->second.find(msg.type().name()) == it->second.end())
    {
        return false;
    }

    const auto cb = impl().is_subscription_callbacks.find(topic);
    if (cb == impl().is_subscription_callbacks.end())
    {
        return false;
    }

    (*cb->second)(msg, nullptr);

    return true;
}

//==============================================================================
bool subscribe(
        const std::string& topic,
        MockSubscriptionCallback callback)
{
    const auto it = impl().publishers.find(topic);
    if (it == impl().publishers.end())
    {
        return false;
    }

    impl().mock_subscriptions[topic].emplace_back(std::move(callback));
    return true;
}

//==============================================================================
class MockServiceClient
    : public virtual ServiceClient,
    public std::enable_shared_from_this<MockServiceClient>
{
public:

    MockServiceClient()
        : response_received(false)
        , quit(false)
    {
        // Does nothing
    }

    std::shared_future<xtypes::DynamicData> request(
            const std::string& topic,
            const eprosima::xtypes::DynamicData& request_msg,
            std::chrono::nanoseconds retry)
    {
        const auto it = impl().is_request_callbacks.find(topic);
        if (it == impl().is_request_callbacks.end())
        {
            throw std::runtime_error(
                      "a callback could not be found for the requested service: "
                      + topic);
        }

        (*it->second)(request_msg, *this, shared_from_this());

        auto future = promise.get_future().share();

        if (retry != std::chrono::nanoseconds(0))
        {
            // If a non-zero retry is specified, then we will launch a thread that
            // keeps retrying the service request as often as specified, until the
            // result is received.
            retry_thread = std::thread([=]()
                            {
                                while (future.wait_for(retry) != std::future_status::ready)
                                {
                                    if (this->quit)
                                    {
                                        break;
                                    }

                                    (*it->second)(request_msg, *this, shared_from_this());
                                }
                            });
        }

        return future;
    }

    void receive_response(
            std::shared_ptr<void> call_handle,
            const eprosima::xtypes::DynamicData& response) override
    {
        std::unique_lock<std::mutex> lock(this->mutex);
        if (response_received)
        {
            return;
        }

        response_received = true;

        promise.set_value(response);
        (void)call_handle;
    }

    std::promise<xtypes::DynamicData> promise;
    std::thread retry_thread;
    bool response_received;
    std::atomic_bool quit;
    std::mutex mutex;

    ~MockServiceClient() override
    {
        quit = true;
        if (retry_thread.joinable())
        {
            retry_thread.join();
        }
    }

};

//==============================================================================
std::shared_future<xtypes::DynamicData> request(
        const std::string& topic,
        const eprosima::xtypes::DynamicData& request_msg,
        std::chrono::nanoseconds retry)
{
    const auto it = impl().clients.find(topic);
    if (it == impl().clients.end())
    {
        throw std::runtime_error(
                  "you have requested a service from mock middleware "
                  "that it is not providing: " + topic);
    }

    auto client = std::make_shared<MockServiceClient>();
    impl().mock_clients.push_back(client);

    return client->request(topic, request_msg, retry);
}

//==============================================================================
void serve(
        const std::string& topic,
        MockServiceCallback callback,
        const std::string& type)
{
    const auto it = impl().services.find(topic);
    if (it == impl().services.end())
    {
        throw std::runtime_error(
                  "you are attempting to serve something from mock middleware "
                  "that it is not providing: " + topic);
    }

    const auto sit =
            impl().mock_services.insert(
        std::make_pair(
            (type.empty() ? topic : topic + "_" + type),
            callback));
    if (!sit.second)
    {
        throw std::runtime_error(
                  "you are attempting to serve [" + topic + "], but it is already "
                  "being served!");
    }
}

} //  namespace mock
} //  namespace sh
} //  namespace is
} //  namespace eprosima

//==============================================================================
IS_REGISTER_SYSTEM("mock", eprosima::is::sh::mock::SystemHandle)
