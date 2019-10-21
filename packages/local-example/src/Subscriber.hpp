#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP

#include "MiddlewareConnection.hpp"
#include "conversion.hpp"

#include <soss/SystemHandle.hpp>

class Subscriber
{
public:
    Subscriber(
            const std::string& topic,
            const xtypes::DynamicType& type,
            soss::TopicSubscriberSystem::SubscriptionCallback soss_callback,
            MiddlewareConnection& connection)
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

    void receive(const MiddlewareMessage& middleware_message)
    {
        std::cout << "[soss-local-example]: (conversion) middleware -> soss" << std::endl;

        xtypes::DynamicData soss_message(type_);

        if(!conversion::middleware_to_soss(middleware_message, soss_message))
        {
            std::cerr << "Conversion error" << std::endl;
            return;
        }

        soss_callback_(soss_message);
    }

    const std::string& topic() const { return topic_; }
    const xtypes::DynamicType& type() const { return type_; }

private:
    const std::string topic_;
    const xtypes::DynamicType& type_;
    soss::TopicSubscriberSystem::SubscriptionCallback soss_callback_;

};

#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__SUBSCRIBER_HPP
