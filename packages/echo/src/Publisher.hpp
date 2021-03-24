#ifndef _ECHO_IS_SH__INTERNAL__PUBLISHER_HPP_
#define _ECHO_IS_SH__INTERNAL__PUBLISHER_HPP_

#include "MiddlewareConnection.hpp"

#include <is/systemhandle/SystemHandle.hpp>

namespace xtypes = eprosima::xtypes;
namespace is = eprosima::is;

class Publisher : public virtual is::TopicPublisher
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
            const xtypes::DynamicData& is_message) override
    {
        std::cout << "[is-echo]: (conversion) xtypes -> middleware" << std::endl;

        Json middleware_message = is::json::convert(is_message);

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


#endif //  _ECHO_IS_SH__INTERNAL__PUBLISHER_HPP_
