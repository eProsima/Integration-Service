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

#ifndef SOSS__ROS2__INTERNAL__SYSTEMHANDLE_HPP
#define SOSS__ROS2__INTERNAL__SYSTEMHANDLE_HPP

#include <soss/SystemHandle.hpp>
#include <rclcpp/rclcpp.hpp>

#include <soss/ros2/Factory.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

#include <vector>

namespace soss {
namespace ros2 {

//==============================================================================
class SystemHandle : public virtual FullSystem
{
public:

    // Construct the system handle
    SystemHandle();

    // Documentation inherited
    bool configure(
            const RequiredTypes& types,
            const YAML::Node& configuration,
            TypeRegistry& type_registry) override;

    // Documentation inherited
    bool okay() const override;

    // Documentation inherited
    bool spin_once() override;

    // Documentation inherited
    ~SystemHandle() override;

    // Documentation inherited
    bool subscribe(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            SubscriptionCallback callback,
            const YAML::Node& configuration) override;

    // Documentation inherited
    std::shared_ptr<TopicPublisher> advertise(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            const YAML::Node& configuration) override;

    // Documentation inherited
    bool create_client_proxy(
            const std::string& service_name,
            const xtypes::DynamicType& service_type,
            RequestCallback callback,
            const YAML::Node& configuration) override;

    // Documentation inherited
    bool create_client_proxy(
            const std::string& service_name,
            const xtypes::DynamicType&,
            const xtypes::DynamicType& reply_type,
            RequestCallback callback,
            const YAML::Node& configuration) override
    {
        return create_client_proxy(service_name, reply_type, callback, configuration);
    }

    // Documentation inherited
    std::shared_ptr<ServiceProvider> create_service_proxy(
            const std::string& service_name,
            const xtypes::DynamicType& service_type,
            const YAML::Node& configuration) override;

    // Documentation inherited
    std::shared_ptr<ServiceProvider> create_service_proxy(
            const std::string& service_name,
            const xtypes::DynamicType& request_type,
            const xtypes::DynamicType&,
            const YAML::Node& configuration) override
    {
        return create_service_proxy(service_name, request_type, configuration);
    }

private:

    std::shared_ptr<rclcpp::Context> _context;
    std::shared_ptr<rclcpp::NodeOptions> _node_options;
    std::shared_ptr<rclcpp::Node> _node;
#ifdef SOSS_ROS2__ROSIDL_GENERATOR_CPP
    std::unique_ptr<rclcpp::executor::Executor> _executor;
#else
    std::unique_ptr<rclcpp::Executor> _executor;
#endif // SOSS_ROS2_ROSIDL_GENERATOR_CPP

    rclcpp::ExecutorOptions _executor_options;

    std::vector<std::shared_ptr<void> > _subscriptions;
    std::vector<std::shared_ptr<ServiceClient> > _client_proxies;
};


} // namespace ros2
} // namespace soss


#endif // SOSS__ROS2__INTERNAL__SYSTEMHANDLE_HPP
