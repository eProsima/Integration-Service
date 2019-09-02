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

#include "System.hpp"
#include "Subscriber.hpp"
#include "Publisher.hpp"

#include <soss/SystemHandle.hpp>
#include <dds/core/xtypes/PrimitiveTypes.hpp>

#include <chrono>
#include <thread>
#include <iostream>

namespace {

int64_t timeStamp()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

} //global

class SystemHandle : public virtual soss::TopicSystem
{
public:
    SystemHandle() : initial_time_stamp_ms_(timeStamp()) {}
    virtual ~SystemHandle() override = default;

    bool configure(
        const soss::RequiredTypes&,
        const YAML::Node& configuration,
        std::map<std::string, dds::core::xtypes::StructType>& type_register) override
    {
        // The system handle creates and manages its own types.
        // (It could come from buildes or from an idl compiler)
        dds::core::xtypes::StructType coord_2d("coordinate2d");
        coord_2d.add_member(dds::core::xtypes::Member("x", dds::core::xtypes::primitive_type<uint32_t>()));
        coord_2d.add_member(dds::core::xtypes::Member("y", dds::core::xtypes::primitive_type<uint32_t>()));
        types_.emplace(coord_2d.name(), std::move(coord_2d));

        dds::core::xtypes::StructType coord_3d("coordinate3d");
        coord_3d.add_member(dds::core::xtypes::Member("x", dds::core::xtypes::primitive_type<uint32_t>()));
        coord_3d.add_member(dds::core::xtypes::Member("y", dds::core::xtypes::primitive_type<uint32_t>()));
        coord_3d.add_member(dds::core::xtypes::Member("z", dds::core::xtypes::primitive_type<uint32_t>()));
        types_.emplace(coord_3d.name(), std::move(coord_3d));

        // Notify all the types to soss
        for (auto&& it: types_)
        {
            type_register.insert(std::make_pair(it.first, it.second));
        }

        //Mock connection
        bool roundtrip = configuration["roundtrip"] ? configuration["roundtrip"].as<bool>() : false;
        initial_msg_ms_ = configuration["initial_msg_ms"] ? configuration["initial_msg_ms"].as<int>() : 0;

        connection_.reset(new SystemConnection(roundtrip));

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

        if(initial_msg_ms_ && timeStamp() - initial_time_stamp_ms_ > initial_msg_ms_)
        {
            initial_msg_ms_ = 0;

            for(auto&& subscriber: subscribers_)
            {
                if(subscriber->type().name() == "coordinate2d")
                {
                    connection_->receive(subscriber->topic(), {{"x", 3} , {"y", 6 }});
                }
                else if(subscriber->type().name() == "coordinate3d")
                {
                    connection_->receive(subscriber->topic(), {{"x", 3} , {"y", 6 }, {"z", 9}});
                }
            }
        }

        return okay();
    }

    bool subscribe(
        const std::string& topic_name,
        const std::string& message_type,
        SubscriptionCallback callback,
        const YAML::Node& ) override
    {
        auto subscriber = std::make_shared<Subscriber>(topic_name, types_.at(message_type), callback, *connection_);
        subscribers_.emplace_back(std::move(subscriber));

        std::cout << "[soss-xtypes-example]: subscriber created. "
            "topic: " << topic_name << ", "
            "type: " << message_type << std::endl;

        return true;
    }

    std::shared_ptr<soss::TopicPublisher> advertise(
        const std::string& topic_name,
        const std::string& message_type,
        const YAML::Node& ) override
    {
        auto publisher = std::make_shared<Publisher>(topic_name, types_.at(message_type), *connection_);
        publishers_.emplace_back(std::move(publisher));

        std::cout << "[soss-xtypes-example]: publisher created. "
            "topic: " << topic_name << ", "
            "type: " << message_type << std::endl;

        return publishers_.back();
    }

private:
    std::unique_ptr<SystemConnection> connection_;
    std::map<std::string, dds::core::xtypes::StructType> types_;
    std::vector<std::shared_ptr<Publisher>> publishers_;
    std::vector<std::shared_ptr<Subscriber>> subscribers_;
    int initial_msg_ms_;
    int64_t initial_time_stamp_ms_;
};

SOSS_REGISTER_SYSTEM("xtypes-example", SystemHandle)
