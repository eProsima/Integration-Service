#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP

#include "System.hpp"

#include <soss/SystemHandle.hpp>

class Publisher : public virtual soss::TopicPublisher
{
public:
    Publisher(
            const std::string& topic,
            const dds::core::xtypes::StructType& type,
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

    bool publish(const dds::core::xtypes::DynamicData& message) override
    {
        SystemMessage system_message;

        // Conversion
        system_message.emplace("x", message.value<uint32_t>("x"));
        system_message.emplace("y", message.value<uint32_t>("y"));

        connection_.publish(topic_, system_message);

        return true;
    }

    const std::string& topic() const { return topic_; }
    const dds::core::xtypes::StructType& type() const { return type_; }

private:
    const std::string topic_;
    const dds::core::xtypes::StructType type_;
    SystemConnection& connection_;
};


#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__PUBLISHER_HPP
