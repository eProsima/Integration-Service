#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__CONVERSION_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__CONVERSION_HPP

#include "MiddlewareConnection.hpp"

#include <soss/Message.hpp>

namespace conversion {

inline bool middleware_to_soss(
        const MiddlewareMessage& middleware_message,
        xtypes::DynamicData& soss_message)
{
    return soss_message.for_each([&](const xtypes::DynamicData::WritableNode& node)
    {
        const std::string& member_name = node.access().struct_member().name();

        switch(node.type().kind())
        {
            case xtypes::TypeKind::STRUCTURE_TYPE:
                break;
            case xtypes::TypeKind::UINT_32_TYPE:
                node.data().value(middleware_message.at(member_name));
                break;
            //... other types
            default:
                throw false;
        }
    });
}

inline bool soss_to_middleware(
        const xtypes::DynamicData& soss_message,
        MiddlewareMessage& middleware_message)
{
    return soss_message.for_each([&](const xtypes::DynamicData::ReadableNode& node)
    {
        const std::string& member_name = node.access().struct_member().name();

        switch(node.type().kind())
        {
            case xtypes::TypeKind::STRUCTURE_TYPE:
                break;
            case xtypes::TypeKind::UINT_32_TYPE:
                middleware_message.emplace(member_name, node.data().value<uint32_t>());
                break;
            //... other types
            default:
                throw false;
        }
    });
}

}

#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__CONVERSION_HPP
