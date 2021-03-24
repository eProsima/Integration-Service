/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Endpoint.hpp"

#include <is/core/runtime/StringTemplate.hpp>

namespace eprosima {
namespace is {
namespace sh {
namespace websocket {

//==============================================================================
class TopicPublisher : public is::TopicPublisher
{
public:

    TopicPublisher(
            const std::string& topic,
            const eprosima::xtypes::DynamicType& message_type,
            const std::string& id,
            const YAML::Node& configuration,
            Endpoint& endpoint)
        : _topic(topic)
        , _endpoint(endpoint)
    {
        _endpoint.startup_advertisement(topic, message_type, id, configuration);
    }

    bool publish(
            const eprosima::xtypes::DynamicData& message)
    {
        return _endpoint.publish(_topic, message);
    }

private:

    const std::string _topic;
    Endpoint& _endpoint;

};

namespace {
//==============================================================================
std::string make_detail_string(
        const std::string& topic_name,
        const eprosima::xtypes::DynamicType& message_type)
{
    return
        "[Middleware: Websocket, topic template: "
        + topic_name + ", message type: " + message_type.name() + "]";
}

} // anonymous namespace

//==============================================================================
class MetaTopicPublisher : public is::TopicPublisher
{
public:

    MetaTopicPublisher(
            is::core::StringTemplate&& string_template,
            const eprosima::xtypes::DynamicType& message_type,
            const std::string& id,
            const YAML::Node& configuration,
            Endpoint& endpoint)
        : _string_template(std::move(string_template))
        , _message_type(message_type)
        , _id(id)
        , _config(configuration)
        , _endpoint(endpoint)
    {
        // Do nothing
    }

    bool publish(
            const eprosima::xtypes::DynamicData& message)
    {
        const std::string topic = _string_template.compute_string(message);
        const bool inserted = _advertised_topics.insert(topic).second;

        if (inserted)
        {
            _endpoint.startup_advertisement(topic, *_message_type, _id, _config);
            _endpoint.runtime_advertisement(topic, *_message_type, _id, _config);
        }

        return _endpoint.publish(topic, message);
    }

private:

    const is::core::StringTemplate _string_template;
    const eprosima::xtypes::DynamicType::Ptr _message_type;
    const std::string _id;
    const YAML::Node _config;
    std::unordered_set<std::string> _advertised_topics;
    Endpoint& _endpoint;

};

//==============================================================================
std::shared_ptr<is::TopicPublisher> make_topic_publisher(
        const std::string& topic,
        const eprosima::xtypes::DynamicType& message_type,
        const std::string& id,
        const YAML::Node& configuration,
        Endpoint& endpoint)
{
    if (topic.find('{') != std::string::npos)
    {
        return std::make_shared<websocket::MetaTopicPublisher>(
            is::core::StringTemplate(topic, make_detail_string(topic, message_type)),
            message_type, id, configuration, endpoint);
    }

    return std::make_shared<websocket::TopicPublisher>(
        topic, message_type, id, configuration, endpoint);
}

} // namespace websocket
} // namespace sh
} // namespace is
} // namespace eprosima
