#ifndef _ECHO_IS_SH__INTERNAL__SUBSCRIBER_HPP_
#define _ECHO_IS_SH__INTERNAL__SUBSCRIBER_HPP_

#include "MiddlewareConnection.hpp"

#include <is/systemhandle/SystemHandle.hpp>

namespace xtypes = eprosima::xtypes;
namespace is = eprosima::is;
class Subscriber
{
public:

    Subscriber(
            const std::string& topic,
            const xtypes::DynamicType& type,
            is::TopicSubscriberSystem::SubscriptionCallback is_callback,
            MiddlewareConnection& connection)
        : topic_(topic)
        , type_(type)
        , is_callback_(is_callback)
    {
        connection.subscribe(topic, std::bind(&Subscriber::receive, this, std::placeholders::_1));
    }

    virtual ~Subscriber() = default;
    Subscriber(
            const Subscriber& rhs) = delete;
    Subscriber& operator = (
            const Subscriber& rhs) = delete;
    Subscriber(
            Subscriber&& rhs) = delete;
    Subscriber& operator = (
            Subscriber&& rhs) = delete;

    void receive(
            const Json& middleware_message)
    {
        std::cout << "[is-echo]: (conversion) middleware -> xtypes" << std::endl;

        xtypes::DynamicData is_message = is::json::convert(type_, middleware_message);

        is_callback_(is_message);
    }

    const std::string& topic() const
    {
        return topic_;
    }

    const xtypes::DynamicType& type() const
    {
        return type_;
    }

private:

    const std::string topic_;
    const xtypes::DynamicType& type_;
    is::TopicSubscriberSystem::SubscriptionCallback is_callback_;

};

#endif //  _ECHO_IS_SH__INTERNAL__SUBSCRIBER_HPP_
