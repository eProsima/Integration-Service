#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP

#include "System.hpp"

#include <soss/SystemHandle.hpp>

class Subscriber
{
public:
    Subscriber(
            const std::string& topic,
            const dds::core::xtypes::StructType& type,
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
        dds::core::xtypes::DynamicData message(type_);

        // Conversion
        message.value("x", system_message.at("x"));
        message.value("y", system_message.at("y"));

        std::cout << "[system -> soss]: conversion to xtype:" << std::endl;

        soss_callback_(message);
    }

    const std::string& topic() const { return topic_; }
    const dds::core::xtypes::StructType& type() const { return type_; }

private:
    const std::string topic_;
    const dds::core::xtypes::StructType type_;
    soss::TopicSubscriberSystem::SubscriptionCallback soss_callback_;

};

#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP
