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

#include "MetaPublisher.hpp"

#include <soss/ros2/Factory.hpp>

#include <soss/StringTemplate.hpp>

namespace soss {
namespace ros2 {

//==============================================================================
class MetaPublisher : public soss::TopicPublisher
{
public:

  MetaPublisher(
      TopicTemplate&& topic_template,
      const std::string& message_type,
      const YAML::Node& configuration)
    : _topic_template(std::move(topic_template)),
      _message_type(message_type),
      _configuration(configuration)
  {
    // Do nothing
  }

  bool publish(const soss::Message& message) override
  {

  }


private:

  const TopicTemplate _topic_template;
  const std::string _message_type;
  const YAML::Node _configuration;

  using TopicPublisherPtr = std::shared_ptr<TopicPublisher>;
  using PublisherMap = std::unordered_map<std::string, TopicPublisherPtr>;
  PublisherMap _publishers;

};



} // namespace ros2
} // namespace soss
