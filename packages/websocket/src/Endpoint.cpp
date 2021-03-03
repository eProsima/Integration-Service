/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Endpoint.hpp"

#include <cstdlib>

namespace soss {
namespace websocket {

//==============================================================================
struct CallHandle
{
    std::string service_name;
    std::string service_type;
    std::string id;
    std::shared_ptr<void> connection_handle;
};

//==============================================================================
inline std::shared_ptr<CallHandle> make_call_handle(
        std::string service_name,
        std::string service_type,
        std::string id,
        std::shared_ptr<void> connection_handle)
{
    return std::make_shared<CallHandle>(
        CallHandle{std::move(service_name),
                   std::move(service_type),
                   std::move(id),
                   std::move(connection_handle)});
}

//==============================================================================
Endpoint::Endpoint()
    : _next_service_call_id(1)
{
    // Do nothing
}

//==============================================================================
bool Endpoint::configure(
        const RequiredTypes& types,
        const YAML::Node& configuration,
        TypeRegistry& type_registry)
{
    if (const YAML::Node encode_node = configuration[YamlEncodingKey])
    {
        const std::string encoding_str = [&]() -> std::string
                {
                    std::string encoding = encode_node.as<std::string>("");
                    std::transform(encoding.begin(), encoding.end(),
                            encoding.begin(), ::tolower);
                    return encoding;
                }
                    ();

        if (encoding_str == YamlEncoding_Rosbridge_v2_0)
        {
            _encoding = make_rosbridge_v2_0();
        }
        else
        {
            std::cerr << "[soss::websocket::SystemHnadle::configure] Unknown "
                      << "encoding type was requested: [" << _encoding
                      << "]" << std::endl;
            return false;
        }
    }
    else
    {
        _encoding = make_rosbridge_v2_0();
    }

    if (!_encoding)
    {
        std::cerr << "[soss::websocket::SystemHandle::configure] Reached a line ["
                  << __LINE__ << "] that should be impossible. Please report this "
                  << "bug!" << std::endl;
        return false;
    }

    for (const std::string& type : types.messages)
    {
        if (!type.empty())
        {
            _encoding->add_type(*type_registry.at(type), type);
        }
    }

    for (const std::string& type : types.services)
    {
        if (!type.empty())
        {
            _encoding->add_type(*type_registry.at(type), type);
        }
    }

    _endpoint = configure_endpoint(types, configuration);

    return static_cast<bool>(_endpoint);
}

//==============================================================================
bool Endpoint::subscribe(
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        SubscriptionCallback callback,
        const YAML::Node& configuration)
{
    _startup_messages.emplace_back(
        _encoding->encode_subscribe_msg(
            topic_name, message_type.name(), "", configuration));

    TopicSubscribeInfo& info = _topic_subscribe_info[topic_name];
    info.type = message_type.name();
    info.callback = callback;

    return true;
}

//==============================================================================
std::shared_ptr<TopicPublisher> Endpoint::advertise(
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        const YAML::Node& configuration)
{
    return make_topic_publisher(
        topic_name, message_type, "", configuration, *this);
}

//==============================================================================
bool Endpoint::create_client_proxy(
        const std::string& service_name,
        const xtypes::DynamicType& service_type,
        RequestCallback callback,
        const YAML::Node& /*configuration*/)
{
    ClientProxyInfo& info = _client_proxy_info[service_name];
    info.type = service_type.name();
    info.callback = callback;

    return true;
}

//==============================================================================
std::shared_ptr<ServiceProvider> Endpoint::create_service_proxy(
        const std::string& service_name,
        const xtypes::DynamicType& service_type,
        const YAML::Node& configuration)
{
    ServiceProviderInfo& info = _service_provider_info[service_name];
    info.type = service_type.name();
    info.configuration = configuration;

    return make_service_provider(service_name, *this);
}

//==============================================================================
void Endpoint::startup_advertisement(
        const std::string& topic,
        const xtypes::DynamicType& message_type,
        const std::string& id,
        const YAML::Node& configuration)
{
    TopicPublishInfo& info = _topic_publish_info[topic];
    info.type = message_type.name();

    _startup_messages.emplace_back(
        _encoding->encode_advertise_msg(
            topic, message_type.name(), id, configuration));
}

//==============================================================================
bool Endpoint::publish(
        const std::string& topic,
        const xtypes::DynamicData& message)
{
    const TopicPublishInfo& info = _topic_publish_info.at(topic);

    // If no one is listening, then don't bother publishing
    if (info.listeners.empty())
    {
        return true;
    }

    for (const auto& v_handle : info.listeners)
    {
        auto connection_handle = _endpoint->get_con_from_hdl(v_handle.first);

        auto ec = connection_handle->send(
            _encoding->encode_publication_msg(topic, info.type, "", message));

        if (ec)
        {
            std::cerr << "[soss::websocket::Endpoint] Failed to send publication on "
                      << "topic [" << topic << "]: " << ec.message() << std::endl;
        }
    }

    return true;
}

//==============================================================================
void Endpoint::call_service(
        const std::string& service,
        const xtypes::DynamicData& request,
        ServiceClient& client,
        std::shared_ptr<void> call_handle)
{
    const std::size_t id = _next_service_call_id++;
    const std::string id_str = std::to_string(id);
    _service_request_info[id_str] = {&client, std::move(call_handle)};

    ServiceProviderInfo& provider_info = _service_provider_info.at(service);

    const std::string payload = _encoding->encode_call_service_msg(
        service, provider_info.type, request,
        id_str, provider_info.configuration);

    _endpoint->get_con_from_hdl(provider_info.connection_handle)->send(payload);
}

//==============================================================================
void Endpoint::receive_response(
        std::shared_ptr<void> v_call_handle,
        const xtypes::DynamicData& response)
{
    const auto& call_handle =
            *static_cast<const CallHandle*>(v_call_handle.get());

    auto connection_handle = _endpoint->get_con_from_hdl(
        call_handle.connection_handle);

    connection_handle->send(
        _encoding->encode_service_response_msg(
            call_handle.service_name,
            call_handle.service_type,
            call_handle.id,
            response, true));
}

//==============================================================================
void Endpoint::receive_topic_advertisement_ws(
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        const std::string& /*id*/,
        std::shared_ptr<void> connection_handle)
{
    auto it = _topic_subscribe_info.find(topic_name);
    if (it != _topic_subscribe_info.end())
    {
        TopicSubscribeInfo& info = it->second;
        if (message_type.name() != info.type)
        {
            info.blacklist.insert(connection_handle);
            std::cerr << "[soss::websocket] A remote connection advertised a topic "
                      << "we want to subscribe to [" << topic_name << "] but with "
                      << "the wrong message type [" << message_type.name() << "]. The "
                      << "expected type is [" << info.type << "]. Messages from "
                      << "this connection will be ignored." << std::endl;
        }
        else
        {
            info.blacklist.erase(connection_handle);
        }
    }
}

//==============================================================================
void Endpoint::receive_topic_unadvertisement_ws(
        const std::string& /*topic_name*/,
        const std::string& /*id*/,
        std::shared_ptr<void> /*connection_handle*/)
{
    // TODO(MXG): Do anything here?
}

//==============================================================================
void Endpoint::receive_publication_ws(
        const std::string& topic_name,
        const xtypes::DynamicData& message,
        std::shared_ptr<void> connection_handle)
{
    auto it = _topic_subscribe_info.find(topic_name);
    if (it == _topic_subscribe_info.end())
    {
        return;
    }

    TopicSubscribeInfo& info = it->second;
    if (info.blacklist.count(connection_handle) > 0)
    {
        return;
    }

    info.callback(message);
}

//==============================================================================
void Endpoint::receive_subscribe_request_ws(
        const std::string& topic_name,
        const xtypes::DynamicType* message_type,
        const std::string& id,
        std::shared_ptr<void> connection_handle)
{
    auto insertion = _topic_publish_info.insert(
        std::make_pair(topic_name, TopicPublishInfo{}));
    const bool inserted = insertion.second;
    TopicPublishInfo& info = insertion.first->second;

    if (inserted)
    {
        std::cerr << "[soss::websocket] Received subscription request for a "
                  << "topic that we are not currently advertising ["
                  << topic_name << "]" << std::endl;
    }
    else
    {
        if (message_type != nullptr && message_type->name() != info.type)
        {
            std::cerr << "[soss::websocket] Received subscription request for topic ["
                      << topic_name << "], but the requested message type ["
                      << message_type->name() << "] does not match the one we are publishing "
                      << "[" << info.type << "]" << std::endl;
            return;
        }
    }

    info.listeners[connection_handle].insert(id);
}

//==============================================================================
void Endpoint::receive_unsubscribe_request_ws(
        const std::string& topic_name,
        const std::string& id,
        std::shared_ptr<void> connection_handle)
{
    auto it = _topic_publish_info.find(topic_name);
    if (it == _topic_publish_info.end())
    {
        std::cerr << "[soss::websocket] Received an unsubscription request for a "
                  << "topic that we are not advertising [" << topic_name << "]"
                  << std::endl;
        return;
    }

    TopicPublishInfo& info = it->second;
    auto lit = info.listeners.find(connection_handle);

    if (lit == info.listeners.end())
    {
        return;
    }

    if (id.empty())
    {
        // If id is empty, then we should erase this connection as a listener
        // entirely.
        info.listeners.erase(lit);
        return;
    }

    std::unordered_set<std::string>& listeners = lit->second;
    listeners.erase(id);

    if (listeners.empty())
    {
        // If no more unique ids are listening from this connection, then
        // erase it entirely.
        info.listeners.erase(lit);
    }
}

//==============================================================================
void Endpoint::receive_service_request_ws(
        const std::string& service_name,
        const xtypes::DynamicData& request,
        const std::string& id,
        std::shared_ptr<void> connection_handle)
{

    auto it = _client_proxy_info.find(service_name);
    if (it == _client_proxy_info.end())
    {
        std::cerr << "[soss::websocket] Received a service request for a service "
                  << "[" << service_name << "] that we are not providing!"
                  << std::endl;
        return;
    }

    ClientProxyInfo& info = it->second;
    info.callback(request, *this,
            make_call_handle(service_name, info.type,
            id, connection_handle));
}

//==============================================================================
void Endpoint::receive_service_advertisement_ws(
        const std::string& service_name,
        const xtypes::DynamicType& service_type,
        std::shared_ptr<void> connection_handle)
{
    _service_provider_info[service_name] =
            ServiceProviderInfo{service_type.name(), connection_handle, YAML::Node{}};
}

//==============================================================================
void Endpoint::receive_service_unadvertisement_ws(
        const std::string& service_name,
        const xtypes::DynamicType* /*service_type*/,
        std::shared_ptr<void> connection_handle)
{
    auto it = _service_provider_info.find(service_name);
    if (it == _service_provider_info.end())
    {
        return;
    }

    if (it->second.connection_handle == connection_handle)
    {
        _service_provider_info.erase(it);
    }
}

//==============================================================================
void Endpoint::receive_service_response_ws(
        const std::string& /*service_name*/,
        const xtypes::DynamicData& response,
        const std::string& id,
        std::shared_ptr<void> /*connection_handle*/)
{
    auto it = _service_request_info.find(id);
    if (it == _service_request_info.end())
    {
        std::cerr << "[soss::websocket] A remote connection provided a service "
                  << "response with an unrecognized id [" << id << "]"
                  << std::endl;
        return;
    }

    // TODO(MXG): We could use the service_name and connection_handle info to
    // verify that the service response is coming from the source that we were
    // expecting.
    ServiceRequestInfo& info = it->second;
    info.client->receive_response(info.call_handle, response);

    _service_request_info.erase(it);
}

//==============================================================================
const Encoding& Endpoint::get_encoding() const
{
    return *_encoding;
}

//==============================================================================
void Endpoint::notify_connection_opened(
        const WsCppConnectionPtr& connection_handle)
{
    for (const std::string& msg : _startup_messages)
    {
        connection_handle->send(msg);
    }
}

//==============================================================================
void Endpoint::notify_connection_closed(
        const std::shared_ptr<void>& connection_handle)
{
    for (auto& entry : _topic_subscribe_info)
    {
        entry.second.blacklist.erase(connection_handle);
    }

    for (auto& entry : _topic_publish_info)
    {
        entry.second.listeners.erase(connection_handle);
    }

    std::vector<std::string> lost_services;
    lost_services.reserve(_service_provider_info.size());
    for (auto& entry : _service_provider_info)
    {
        if (entry.second.connection_handle == connection_handle)
        {
            lost_services.push_back(entry.first);
        }
    }

    for (const std::string& s : lost_services)
    {
        _service_provider_info.erase(s);
    }

    // NOTE(MXG): We'll leave _service_request_info alone, because it's feasible
    // that the service response might arrive later after the other side has
    // reconnected. The downside is this could allow lost services to accumulate.
}

//==============================================================================
int32_t parse_port(
        const YAML::Node& configuration)
{
    if (const YAML::Node port_node = configuration[YamlPortKey])
    {
        try
        {
            return port_node.as<int>();
        }
        catch (const YAML::InvalidNode& v)
        {
            std::cerr << "[soss::websocket::SystemHandle::configure] Could not "
                      << "parse an integer value for the port setting ["
                      << port_node.as<std::string>("") << "]: "
                      << v.what() << std::endl;
        }
    }
    else
    {
        std::cerr << "[soss::websocket::SystemHandle::configure] You must specify "
                  << "a port setting in your soss-websocket configuration!"
                  << std::endl;
    }

    return -1;
}

} // namespace websocket
} // namespace soss
