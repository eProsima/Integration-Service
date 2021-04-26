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

#include <is/core/runtime/FieldToString.hpp>

#include <string>
#include <unordered_map>

namespace eprosima {
namespace is {
namespace core {

class FieldToString::Implementation
{
public:

    /**
     * @brief Gets a reference to this Factory class instance.
     */
    static Implementation& instance()
    {
        static Implementation instance;
        return instance;
    }

    const std::string to_string(
            eprosima::xtypes::ReadableDynamicDataRef field,
            const std::string& field_name,
            const std::string& details)
    {
        const std::string& type =
                (field.type().name().find("std::string") != std::string::npos)
                ? "std::string"
                : field.type().name();

        _logger << utils::Logger::Level::DEBUG << "Trying to convert type '" << type
                << "' to string" << std::endl;
        const auto it = _conversions.find(type);

        if (it != _conversions.end())
        {
            return it->second(field);
        }

        _logger << utils::Logger::Level::ERROR << "Failed convert type '" << type
                << "' to string" << std::endl;

        throw UnknownFieldToStringCast(type, field_name, details);
    }

private:

    using ConversionFunc = std::function<std::string (
                        eprosima::xtypes::ReadableDynamicDataRef)>;
    using ConversionMap = std::unordered_map<std::string, ConversionFunc>;

    Implementation()
        : _logger("is::core::FieldToString")
    {
        _conversions["std::string"] =
                [](xtypes::ReadableDynamicDataRef field) -> std::string
                {
                    return field;
                };

        add_primitive_conversion<bool>();
        add_primitive_conversion<char>();
        add_primitive_conversion<wchar_t>();
        add_primitive_conversion<int8_t>();
        add_primitive_conversion<uint8_t>();
        add_primitive_conversion<int16_t>();
        add_primitive_conversion<uint16_t>();
        add_primitive_conversion<int32_t>();
        add_primitive_conversion<uint32_t>();
        add_primitive_conversion<int64_t>();
        add_primitive_conversion<uint64_t>();
        add_primitive_conversion<float>();
        add_primitive_conversion<double>();
        add_primitive_conversion<long double>();
    }

    Implementation(
            const Implementation& /*other*/) = delete;

    Implementation(
            Implementation&& /*other*/) = delete;

    ~Implementation() = default;

    /**
     * @brief Adds conversion functions for primitive types.
     */
    template<typename T>
    void add_primitive_conversion()
    {
        _conversions[xtypes::primitive_type<T>().name()] =
                [](xtypes::ReadableDynamicDataRef field) -> std::string
                {
                    T temp = field;
                    return std::to_string(temp);
                };
    }

    /**
     * Class members.
     */

    ConversionMap _conversions;
    utils::Logger _logger;
};

//==============================================================================
FieldToString::FieldToString(
        const std::string& usage_details)
    : _pimpl(Implementation::instance())
    , _details(usage_details)
{
}

//==============================================================================
FieldToString::FieldToString(
        const FieldToString& other)
    : _pimpl(Implementation::instance())
    , _details(other._details)
{
}

//==============================================================================
FieldToString::FieldToString(
        FieldToString&& other)
    : _pimpl(Implementation::instance())
    , _details(std::move(other._details))
{
}

//==============================================================================
const std::string FieldToString::to_string(
        eprosima::xtypes::ReadableDynamicDataRef field,
        const std::string& field_name) const
{
    return _pimpl.to_string(field, field_name, _details);
}

//==============================================================================
const std::string& FieldToString::details() const
{
    return _details;
}

//==============================================================================
std::string& FieldToString::details()
{
    return _details;
}

//==============================================================================
UnknownFieldToStringCast::UnknownFieldToStringCast(
        const std::string& type,
        const std::string& field_name,
        const std::string& details)
    : std::runtime_error(
        std::string()
        + "ERROR: Unable to cast type '" + type + "' of field '" + field_name
        + "' to a string. Details: " + details)
    , _type(type)
    , _field_name(field_name)
{
}

//==============================================================================
const std::string& UnknownFieldToStringCast::type() const
{
    return _type;
}

//==============================================================================
const std::string& UnknownFieldToStringCast::field_name() const
{
    return _field_name;
}

} //  namespace core
} //  namespace is
} //  namespace eprosima
