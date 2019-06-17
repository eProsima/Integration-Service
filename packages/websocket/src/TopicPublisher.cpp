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

#include <soss/StringTemplate.hpp>

namespace soss {
namespace websocket {

//==============================================================================
class TopicPublisher : public soss::TopicPublisher
{
public:

  TopicPublisher(
      const std::string& topic,
      const std::string& message_type,
      const std::string& id,
      const YAML::Node& configuration,
      Endpoint& endpoint)
    : _topic(topic),
      _endpoint(endpoint)
  {
    _endpoint.startup_advertisement(topic, message_type, id, configuration);
  }

  bool publish(const soss::Message& message)
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
    const std::string& message_type)
{
  return
      "[Middleware: Websocket, topic template: "
      + topic_name + ", message type: " + message_type + "]";
}
} // anonymous namespace

//==============================================================================
class MetaTopicPublisher : public soss::TopicPublisher
{
public:

  MetaTopicPublisher(
      soss::StringTemplate&& string_template,
      const std::string& message_type,
      const std::string& id,
      const YAML::Node& configuration,
      Endpoint& endpoint)
    : _string_template(std::move(string_template)),
      _message_type(message_type),
      _id(id),
      _configuration(configuration),
      _endpoint(endpoint)
  {
    // Do nothing
  }

  bool publish(const soss::Message& message)
  {
    const std::string& topic = _string_template.compute_string(message);
    _endpoint.startup_advertisement(topic, _message_type, _id, _configuration);
    _endpoint.runtime_advertisement(topic, _message_type, _id, _configuration);

    return _endpoint.publish(topic, message);
  }

private:

  const soss::StringTemplate _string_template;
  const std::string _message_type;
  const std::string _id;
  const YAML::Node _configuration;
  Endpoint& _endpoint;

};

//==============================================================================
std::shared_ptr<soss::TopicPublisher> make_topic_publisher(
    const std::string& topic,
    const std::string& message_type,
    const std::string& id,
    const YAML::Node& configuration,
    Endpoint& endpoint)
{
  if(topic.find('{') != std::string::npos)
  {
    return std::make_shared<websocket::MetaTopicPublisher>(
          soss::StringTemplate(topic, make_detail_string(topic, message_type)),
          message_type, id, configuration, endpoint);
  }

  return std::make_shared<websocket::TopicPublisher>(
        topic, message_type, id, configuration, endpoint);
}

} // namespace websocket
} // namespace soss
