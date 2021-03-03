#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP

#include "MiddlewareConnection.hpp"

#include <soss/SystemHandle.hpp>

class Publisher : public virtual soss::TopicPublisher
{
public:

    Publisher(
            const std::string& topic,
            const xtypes::DynamicType& type,
            MiddlewareConnection& connection)
        : topic_(topic)
        , type_(type)
        , connection_(connection)
    {
    }

    virtual ~Publisher() override = default;
    Publisher(
            const Publisher& rhs) = delete;
    Publisher& operator = (
            const Publisher& rhs) = delete;
    Publisher(
            Publisher&& rhs) = delete;
    Publisher& operator = (
            Publisher&& rhs) = delete;

    bool publish(
            const xtypes::DynamicData& soss_message) override
    {
        std::cout << "[soss-echo]: (conversion) soss -> middleware" << std::endl;

        Json middleware_message = soss::json::convert(soss_message);

        connection_.publish(topic_, middleware_message);

        return true;
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
    MiddlewareConnection& connection_;
};


#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP
