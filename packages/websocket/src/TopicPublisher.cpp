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

namespace soss {
namespace websocket {

//==============================================================================
class TopicPublisher : public soss::TopicPublisher
{
public:

  TopicPublisher(
      const std::string& topic,
      Endpoint& endpoint)
    : _topic(topic),
      _endpoint(endpoint)
  {
    // Do nothing
  }

  bool publish(const soss::Message& message)
  {
    return _endpoint.publish(_topic, message);
  }

private:

  const std::string _topic;
  Endpoint& _endpoint;

};

//==============================================================================
std::shared_ptr<soss::TopicPublisher> make_topic_publisher(
    const std::string& topic, Endpoint& endpoint)
{
  return std::make_shared<websocket::TopicPublisher>(topic, endpoint);
}

} // namespace websocket
} // namespace soss
