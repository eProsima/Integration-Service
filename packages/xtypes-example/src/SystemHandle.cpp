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

#include <soss/SystemHandle.hpp>

#include <chrono>
#include <thread>
#include <iostream>

namespace {

void print_system_message(const std::string& prefix, const std::map<std::string, int>& system_message)
{
    std::stringstream ss; //Used to avoid mixing differents outs
    ss << prefix;
    for(auto&& member: system_message)
    {
        ss << member.first << ": " << member.second << ", ";
    }
    ss << std::endl;

    std::cout << ss.str();
}

}

class Subscriber
{
public:
    Subscriber(
            const std::string& topic_name,
            const std::shared_ptr<soss::MessageType>& type,
            soss::TopicSubscriberSystem::SubscriptionCallback soss_callback)
        : topic_name_(topic_name)
        , type_(type)
        , soss_callback_(soss_callback)
    {
    }

    virtual ~Subscriber() = default;
    Subscriber(const Subscriber& rhs) = delete;
    Subscriber& operator = (const Subscriber& rhs) = delete;
    Subscriber(Subscriber&& rhs) = delete;
    Subscriber& operator = (Subscriber&& rhs) = delete;

    void receive(const std::map<std::string, int>& system_message)
    {
        soss::MessageData message(*type_);

        // Conversion
        for(auto&& member: type_->get_members())
        {
            message[member.first] = std::to_string(system_message.at(member.first));
        }

        print_system_message("[system -> soss]: ", system_message);

        soss_callback_(message);
    }

    const std::string& get_topic_name() const { return topic_name_; }
    const std::shared_ptr<soss::MessageType>& get_type() const { return type_; }

private:
    const std::string topic_name_;
    const std::shared_ptr<soss::MessageType> type_;
    soss::TopicSubscriberSystem::SubscriptionCallback soss_callback_;
};



class Publisher : public virtual soss::TopicPublisher
{
public:
    Publisher(
            const std::string& topic_name,
            const std::shared_ptr<soss::MessageType>& type,
            bool roundtrip,
            std::vector<std::shared_ptr<Subscriber>>& subscribers)
        : topic_name_(topic_name)
        , type_(type)
        , roundtrip_(roundtrip)
        , subscribers_(subscribers)
    {
    }

    virtual ~Publisher() override = default;
    Publisher(const Publisher& rhs) = delete;
    Publisher& operator = (const Publisher& rhs) = delete;
    Publisher(Publisher&& rhs) = delete;
    Publisher& operator = (Publisher&& rhs) = delete;

    bool publish(const soss::MessageData& message) override
    {
        std::map<std::string, int> system_message;

        // Conversion
        for(auto&& member: type_->get_members())
        {
            system_message[member.first] = std::stoi(message[member.first]);
        }

        print_system_message("[soss -> system]: ", system_message);

        if(roundtrip_)
        {
            for(auto&& subscriber: subscribers_)
            {
                if(subscriber->get_topic_name() == topic_name_
                        && subscriber->get_type()->get_name() == type_->get_name())
                {
                    subscriber->receive(system_message);
                }
            }
        }

        return true;
    }

    const std::string& get_topic_name() const { return topic_name_; }
    const std::shared_ptr<soss::MessageType>& get_type() const { return type_; }

private:
    const std::string topic_name_;
    const std::shared_ptr<soss::MessageType> type_;
    bool roundtrip_;
    std::vector<std::shared_ptr<Subscriber>>& subscribers_;
};



class SystemHandle : public virtual soss::TopicSystem
{
public:
    SystemHandle() = default;
    virtual ~SystemHandle() override = default;

    bool configure(
        const soss::RequiredTypes& /* types */,
        const YAML::Node& configuration,
        std::map<std::string, soss::MessageType*>& type_register) override
    {
        autogenerate_ = configuration["autogenerate"] ? configuration["autogenerate"].as<bool>() : false;
        roundtrip_ = configuration["roundtrip"] ? configuration["roundtrip"].as<bool>() : false;

        // The system handle creates and manages its own types.
        // (It could come from buildes or from an idl compiler)
        auto coord_2d = std::make_shared<soss::MessageType>("coordinate2d");
        (*coord_2d)["x"] = soss::MessageType::Type::INT;
        (*coord_2d)["y"] = soss::MessageType::Type::INT;

        auto coord_3d = std::make_shared<soss::MessageType>("coordinate3d");
        (*coord_3d)["x"] = soss::MessageType::Type::INT;
        (*coord_3d)["y"] = soss::MessageType::Type::INT;
        (*coord_3d)["z"] = soss::MessageType::Type::INT;

        types_.emplace(coord_2d->get_name(), std::move(coord_2d));
        types_.emplace(coord_3d->get_name(), std::move(coord_3d));

        // Notify all the types to soss
        for (auto&& it: types_)
        {
            type_register.insert(std::make_pair(it.first, it.second.get()));
        }

        return true;
    }

    bool okay() const override
    {
        return true;
    }

    bool spin_once() override
    {
        using namespace std::chrono_literals;

        // --- Emulating reading from the world
        if(autogenerate_)
        {
            for(auto&& subscriber: subscribers_)
            {
                if(subscriber->get_type()->get_name() == "coordinate2d")
                {
                    subscriber->receive({{"x", 3} , {"y", 6 }});
                }

                else if(subscriber->get_type()->get_name() == "coordinate3d")
                {
                    subscriber->receive({{"x", 3} , {"y", 6 }, {"z", 9}});
                }
            }
        }
        // ---

        std::this_thread::sleep_for(1s);

        return okay();
    }

    bool subscribe(
        const std::string& topic_name,
        const std::string& message_type,
        SubscriptionCallback callback,
        const YAML::Node& /* configuration */) override
    {
        auto subscriber = std::make_shared<Subscriber>(topic_name, types_.at(message_type), callback);
        subscribers_.emplace_back(std::move(subscriber));

        std::cout << "[soss-xtypes-example]: subscriber created. "
            "topic: " << topic_name << ", "
            "type: " << message_type << std::endl;

        return true;
    }

    std::shared_ptr<soss::TopicPublisher> advertise(
        const std::string& topic_name,
        const std::string& message_type,
        const YAML::Node& /* configuration */) override
    {
        auto publisher = std::make_shared<Publisher>(topic_name, types_.at(message_type), roundtrip_, subscribers_);
        publishers_.emplace_back(std::move(publisher));

        std::cout << "[soss-xtypes-example]: publisher created. "
            "topic: " << topic_name << ", "
            "type: " << message_type << std::endl;

        return publishers_.back();
    }

private:
    std::map<std::string, std::shared_ptr<soss::MessageType>> types_;
    std::vector<std::shared_ptr<Publisher>> publishers_;
    std::vector<std::shared_ptr<Subscriber>> subscribers_;
    bool autogenerate_;
    bool roundtrip_;
};


SOSS_REGISTER_SYSTEM("xtypes-example", SystemHandle)
