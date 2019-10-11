#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP

#include "SystemConnection.hpp"

#include <soss/SystemHandle.hpp>

class Subscriber
{
public:
    Subscriber(
            const std::string& topic,
            const soss::xtypes::DynamicType& type,
            soss::TopicSubscriberSystem::SubscriptionCallback soss_callback,
            SystemConnection& connection)
        : topic_(topic)
        , type_(type)
        , soss_callback_(soss_callback)
    {
        connection.subscribe(topic, std::bind(&Subscriber::receive, this, std::placeholders::_1));
    }

    virtual ~Subscriber() = default;
    Subscriber(const Subscriber& rhs) = delete;
    Subscriber& operator = (const Subscriber& rhs) = delete;
    Subscriber(Subscriber&& rhs) = delete;
    Subscriber& operator = (Subscriber&& rhs) = delete;

    void receive(const SystemMessage& system_message)
    {
        soss::xtypes::DynamicData message(type_);

        // Conversion
        message["x"].value(system_message.at("x"));
        message["y"].value(system_message.at("y"));

        std::cout << "[system -> soss]: conversion to xtype:" << std::endl;

        soss_callback_(message);
    }

    const std::string& topic() const { return topic_; }
    const soss::xtypes::DynamicType& type() const { return type_; }

private:
    const std::string topic_;
    const soss::xtypes::DynamicType& type_;
    soss::TopicSubscriberSystem::SubscriptionCallback soss_callback_;

};

#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP
