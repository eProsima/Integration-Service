#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP

#include "SystemConnection.hpp"

#include <soss/SystemHandle.hpp>

class Publisher : public virtual soss::TopicPublisher
{
public:
    Publisher(
            const std::string& topic,
            const soss::xtypes::DynamicType& type,
            SystemConnection& connection)
        : topic_(topic)
        , type_(type)
        , connection_(connection)
    {
    }

    virtual ~Publisher() override = default;
    Publisher(const Publisher& rhs) = delete;
    Publisher& operator = (const Publisher& rhs) = delete;
    Publisher(Publisher&& rhs) = delete;
    Publisher& operator = (Publisher&& rhs) = delete;

    bool publish(const soss::xtypes::DynamicData& message) override
    {
        SystemMessage system_message;

        // Conversion
        system_message.emplace("x", message["x"].value<uint32_t>());
        system_message.emplace("y", message["y"].value<uint32_t>());

        std::cout << "[soss -> system]: conversion to system:" << std::endl;

        connection_.publish(topic_, system_message);

        return true;
    }

    const std::string& topic() const { return topic_; }
    const soss::xtypes::DynamicType& type() const { return type_; }

private:
    const std::string topic_;
    const soss::xtypes::DynamicType& type_;
    SystemConnection& connection_;
};


#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP
