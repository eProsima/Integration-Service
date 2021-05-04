/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <is/json-xtypes/conversion.hpp>
#include <stack>
#include <limits>

#include <iostream>

namespace eprosima {
namespace is {
namespace json_xtypes {

class UnsupportedType : public std::exception
{
public:

    UnsupportedType(
            const std::string& type_name)
        : std::exception()
        , type_name_(type_name)
    {
    }

    const char* what() const noexcept
    {
        std::ostringstream err;
        err << "[json-xtypes] Unsupported type '" << type_name_ << "'";
        return err.str().c_str();
    }

private:

    const std::string type_name_;

};

Json::const_reference access_json_value(
        const xtypes::DynamicData::ReadableNode& xtypes_node,
        Json::const_pointer json_node)
{
    if (xtypes_node.parent().type().is_collection_type())
    {
        size_t index = xtypes_node.from_index();
        return json_node->operator [](index);
    }
    if (xtypes_node.parent().type().is_aggregation_type())
    {
        const std::string& member_name = xtypes_node.from_member()->name();
        return json_node->operator [](member_name);
    }
    static Json none;
    return none;
}

template <typename T>
T get_json_float(
        Json::const_reference json_node)
{
    if (json_node.is_number())
    {
        return json_node.get<T>();
    }
    else
    {
        const std::string json_node_str = json_node.get<std::string>();

        if ("inf" == json_node_str)
        {
            return std::numeric_limits<T>::infinity();
        }
        else if ("-inf" == json_node_str)
        {
            return -std::numeric_limits<T>::infinity();
        }
        else if ("nan" == json_node_str)
        {
            return std::numeric_limits<T>::quiet_NaN();
        }
        else if ("-nan" == json_node_str)
        {
            return -std::numeric_limits<T>::quiet_NaN();
        }
    }

    std::ostringstream err;
    err << "Calling 'get_json_float' for a non-float value: '"
        << json_node << "'";
    throw UnsupportedType(err.str());
}

bool json_to_xtypes(
        const Json& json_message,
        xtypes::DynamicData& xtypes_message)
{
    std::stack<Json::const_pointer> json_stack;
    json_stack.push(&json_message);

    return xtypes_message.for_each(
        [&](xtypes::DynamicData::WritableNode& xtypes_node)
        {
            if (!xtypes_node.has_parent())
            {
                return;                 // avoid the first level already pushed into stack.

            }
            while (json_stack.size() > xtypes_node.deep())
            {
                json_stack.pop();
            }

            switch (xtypes_node.type().kind())
            {
                case xtypes::TypeKind::STRUCTURE_TYPE:
                    json_stack.push(&access_json_value(xtypes_node, json_stack.top()));
                    break;
                case xtypes::TypeKind::SEQUENCE_TYPE:
                    json_stack.push(&access_json_value(xtypes_node, json_stack.top()));
                    xtypes_node.data().resize(json_stack.top()->size());
                    break;
                case xtypes::TypeKind::ARRAY_TYPE:
                    json_stack.push(&access_json_value(xtypes_node, json_stack.top()));
                    break;
                case xtypes::TypeKind::STRING_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<std::string>());
                    break;
                case xtypes::TypeKind::BOOLEAN_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node, json_stack.top()).get<bool>());
                    break;
                case xtypes::TypeKind::CHAR_8_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node, json_stack.top()).get<char>());
                    break;
                case xtypes::TypeKind::INT_8_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node, json_stack.top()).get<int8_t>());
                    break;
                case xtypes::TypeKind::UINT_8_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<uint8_t>());
                    break;
                case xtypes::TypeKind::INT_16_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<int16_t>());
                    break;
                case xtypes::TypeKind::UINT_16_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<uint16_t>());
                    break;
                case xtypes::TypeKind::INT_32_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<int32_t>());
                    break;
                case xtypes::TypeKind::UINT_32_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<uint32_t>());
                    break;
                case xtypes::TypeKind::INT_64_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<int64_t>());
                    break;
                case xtypes::TypeKind::UINT_64_TYPE:
                    xtypes_node.data().value(access_json_value(xtypes_node,
                    json_stack.top()).get<uint64_t>());
                    break;
                case xtypes::TypeKind::FLOAT_32_TYPE:
                    xtypes_node.data().value(get_json_float<float>(access_json_value(xtypes_node,
                    json_stack.top())));
                    break;
                case xtypes::TypeKind::FLOAT_64_TYPE:
                    xtypes_node.data().value(get_json_float<double>(access_json_value(xtypes_node,
                    json_stack.top())));
                    break;
                default:
                    throw UnsupportedType(xtypes_node.type().name());
            }
        });
}

Json::reference add_json_node(
        const xtypes::DynamicData::ReadableNode& xtypes_node,
        Json::pointer json_node)
{
    if (xtypes_node.parent().type().is_collection_type())
    {
        size_t index = xtypes_node.from_index();
        json_node->operator [](index) = {};
        return json_node->operator [](index);
    }
    if (xtypes_node.parent().type().is_aggregation_type())
    {
        const std::string& member_name = xtypes_node.from_member()->name();
        json_node->operator [](member_name) = {};
        return json_node->operator [](member_name);
    }
    static Json none;
    return none;
}

template <typename T>
void add_json_float(
        const xtypes::DynamicData::ReadableNode& xtypes_node,
        Json::pointer json_node)
{
    T value = xtypes_node.data().value<T>();

    if (std::isnormal(value))
    {
        add_json_node(xtypes_node, json_node) = value;
    }
    else
    {
        add_json_node(xtypes_node, json_node) = std::to_string(value);
    }
}

bool xtypes_to_json(
        const xtypes::DynamicData& xtypes_message,
        Json& json_message)
{
    std::stack<Json::pointer> json_stack;
    json_stack.push(&json_message);

    return xtypes_message.for_each(
        [&](const xtypes::DynamicData::ReadableNode& xtypes_node)
        {
            if (!xtypes_node.has_parent())
            {
                return;                 // avoid the first struct level already pushed into stack.

            }
            while (json_stack.size() > xtypes_node.deep())
            {
                json_stack.pop();
            }

            switch (xtypes_node.type().kind())
            {
                case xtypes::TypeKind::STRUCTURE_TYPE:
                    json_stack.push(&add_json_node(xtypes_node, json_stack.top()));
                    break;
                case xtypes::TypeKind::SEQUENCE_TYPE:
                    json_stack.push(&add_json_node(xtypes_node, json_stack.top()));
                    break;
                case xtypes::TypeKind::ARRAY_TYPE:
                    json_stack.push(&add_json_node(xtypes_node, json_stack.top()));
                    break;
                case xtypes::TypeKind::STRING_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<std::string>();
                    break;
                case xtypes::TypeKind::BOOLEAN_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<bool>();
                    break;
                case xtypes::TypeKind::CHAR_8_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<char>();
                    break;
                case xtypes::TypeKind::INT_8_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<int8_t>();
                    break;
                case xtypes::TypeKind::UINT_8_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<uint8_t>();
                    break;
                case xtypes::TypeKind::INT_16_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<int16_t>();
                    break;
                case xtypes::TypeKind::UINT_16_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<uint16_t>();
                    break;
                case xtypes::TypeKind::INT_32_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<int32_t>();
                    break;
                case xtypes::TypeKind::UINT_32_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<uint32_t>();
                    break;
                case xtypes::TypeKind::INT_64_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<int64_t>();
                    break;
                case xtypes::TypeKind::UINT_64_TYPE:
                    add_json_node(xtypes_node, json_stack.top()) = xtypes_node.data().value<uint64_t>();
                    break;
                case xtypes::TypeKind::FLOAT_32_TYPE:
                    add_json_float<float>(xtypes_node, json_stack.top());
                    break;
                case xtypes::TypeKind::FLOAT_64_TYPE:
                    add_json_float<double>(xtypes_node, json_stack.top());
                    break;
                default:
                    throw UnsupportedType(xtypes_node.type().name());
            }
        });
}

//==============================================================================
Json convert(
        const xtypes::DynamicData& xtypes_message)
{
    Json json_message;
    xtypes_to_json(xtypes_message, json_message);
    return json_message;
}

//==============================================================================
xtypes::DynamicData convert(
        const xtypes::DynamicType& type,
        const Json& json_message)
{
    xtypes::DynamicData xtypes_message(type);
    json_to_xtypes(json_message, xtypes_message);
    return xtypes_message;
}

} //  namespace json_xtypes
} //  namespace is
} //  namespace eprosima
