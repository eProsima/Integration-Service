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

#include <soss/StringTemplate.hpp>

#include <soss/ros2/Factory.hpp>


namespace soss {
namespace ros2 {

//==============================================================================
class MetaPublisher : public soss::TopicPublisher
{
public:

    MetaPublisher(
            const StringTemplate&& topic_template,
            const xtypes::DynamicType& message_type,
            rclcpp::Node& node,
            const rmw_qos_profile_t& qos_profile,
            const YAML::Node& /*unused*/)
        : _topic_template(std::move(topic_template))
        , _message_type(message_type)
        , _node(node)
        , _qos_profile(qos_profile)
    {
        // Do nothing
    }

    bool publish(
            const xtypes::DynamicData& message) override final
    {
        const std::string topic_name = _topic_template.compute_string(message);

        const auto insertion = _publishers.insert(
            std::make_pair(std::move(topic_name), nullptr));
        const bool inserted = insertion.second;
        TopicPublisherPtr& publisher = insertion.first->second;

        if (inserted)
        {
            publisher = Factory::instance().create_publisher(
                _message_type, _node, topic_name, _qos_profile);
        }

        return publisher->publish(message);
    }

private:

    const StringTemplate _topic_template;
    const xtypes::DynamicType& _message_type;
    rclcpp::Node& _node;
    const rmw_qos_profile_t _qos_profile;

    using TopicPublisherPtr = std::shared_ptr<TopicPublisher>;
    using PublisherMap = std::unordered_map<std::string, TopicPublisherPtr>;
    PublisherMap _publishers;

};

namespace {
//==============================================================================
std::string make_detail_string(
        const std::string& topic_name,
        const std::string& message_type)
{
    return
        "[Middleware: ROS2, topic template: "
        + topic_name + ", message type: " + message_type + "]";
}

} // anonymous namespace

//==============================================================================
std::shared_ptr<soss::TopicPublisher> make_meta_publisher(
        const xtypes::DynamicType& message_type,
        rclcpp::Node& node,
        const std::string& topic_name,
        const rmw_qos_profile_t& qos_profile,
        const YAML::Node& configuration)
{
    return std::make_shared<MetaPublisher>(
        StringTemplate(topic_name, make_detail_string(topic_name, message_type.name())),
        message_type, node, qos_profile, configuration);
}

} // namespace ros2
} // namespace soss
