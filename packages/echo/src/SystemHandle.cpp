/*
 * Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include "MiddlewareConnection.hpp"
#include "Subscriber.hpp"
#include "Publisher.hpp"

#include <soss/SystemHandle.hpp>

#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>

class SystemHandle : public virtual soss::TopicSystem
{
public:

    virtual ~SystemHandle() override
    {
        connection_->stop();
    }

    bool configure(
            const soss::RequiredTypes& /*required_types*/,
            const YAML::Node& configuration,
            soss::TypeRegistry& /*type_registry*/) override
    {
        // This system handle fetches its types from the idls specified in the yaml.
        // The type_registry should already have the required types.
        bool roundtrip = configuration["roundtrip"] ? configuration["roundtrip"].as<bool>() : false;

        connection_.reset(new MiddlewareConnection(roundtrip));
        connection_->run();

        std::cout << "[soss-echo]: Initializing... "
                  << (roundtrip ? " (roundtrip mode)" : "") << std::endl;

        return true;
    }

    bool okay() const override
    {
        return true;
    }

    bool spin_once() override
    {
        using namespace std::chrono_literals;

        std::this_thread::sleep_for(100ms);

        return okay();
    }

    bool subscribe(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            SubscriptionCallback callback,
            const YAML::Node& /*configuration*/) override
    {
        auto subscriber = std::make_shared<Subscriber>(topic_name, message_type, callback, *connection_);
        subscribers_.emplace_back(std::move(subscriber));

        std::cout << "[soss-echo]: subscriber created. "
            "topic: " << topic_name << ", "
            "type: " << message_type.name() << std::endl;

        return true;
    }

    std::shared_ptr<soss::TopicPublisher> advertise(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            const YAML::Node& /*configuration*/) override
    {
        auto publisher = std::make_shared<Publisher>(topic_name, message_type, *connection_);
        publishers_.emplace_back(std::move(publisher));

        std::cout << "[soss-echo]: publisher created. "
            "topic: " << topic_name << ", "
            "type: " << message_type.name() << std::endl;

        return publishers_.back();
    }

private:

    std::unique_ptr<MiddlewareConnection> connection_;
    std::vector<std::shared_ptr<Publisher> > publishers_;
    std::vector<std::shared_ptr<Subscriber> > subscribers_;
};

SOSS_REGISTER_SYSTEM("echo", SystemHandle)
