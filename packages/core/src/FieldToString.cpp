/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <soss/FieldToString.hpp>

#include <string>
#include <unordered_map>

namespace soss {

namespace {
//==============================================================================
class FieldConversions
{
public:

    static const FieldConversions& instance()
    {
        static FieldConversions conversions;
        return conversions;
    }

    std::string to_string(
            xtypes::ReadableDynamicDataRef field,
            const std::string& field_name,
            const std::string& details) const
    {
        const std::string& type =
                (field.type().name().find("std::string") != std::string::npos)
                ? "std::string"
                : field.type().name();
        const auto it = conversions.find(type);
        if (it != conversions.end())
        {
            return it->second(field);
        }

        throw UnknownFieldToStringCast(type, field_name, details);
    }

private:

    FieldConversions()
    {
        conversions["std::string"] = [](xtypes::ReadableDynamicDataRef field) -> std::string
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

    template<typename T>
    void add_primitive_conversion()
    {
        conversions[xtypes::primitive_type<T>().name()] = [](xtypes::ReadableDynamicDataRef field) -> std::string
                {
                    T temp = field;
                    return std::to_string(temp);
                };
    }

    using ConversionFunc = std::function<std::string(xtypes::ReadableDynamicDataRef)>;
    using ConversionMap = std::unordered_map<std::string, ConversionFunc>;

    ConversionMap conversions;
};
} // anonymous namespace

//==============================================================================
FieldToString::FieldToString(
        const std::string& usage_details)
    : details(usage_details)
{
    // Do nothing
}

//==============================================================================
std::string FieldToString::to_string(
        xtypes::ReadableDynamicDataRef field,
        const std::string& field_name) const
{
    return FieldConversions::instance().to_string(field, field_name, details);
}

//==============================================================================
UnknownFieldToStringCast::UnknownFieldToStringCast(
        const std::string& type,
        const std::string& field_name,
        const std::string& details)
    : std::runtime_error(
        std::string()
        + "ERROR: Unable to cast type [" + type + "] of field [" + field_name
        + "] to a string. Details: " + details)
    , _type(type)
    , _field_name(field_name)
{
    // Do nothing
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

} // namespace soss
