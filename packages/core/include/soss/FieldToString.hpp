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

#ifndef SOSS__FIELDTOSTRING_HPP
#define SOSS__FIELDTOSTRING_HPP

#include <soss/Message.hpp>
#include <stdexcept>

namespace soss {

//==============================================================================
/// \brief Convenience class for converting simple field types into strings.
class FieldToString
{
public:

    /// Construct this and set the details for how the conversion is being used.
    FieldToString(
            const std::string& usage_details);

    /// Convert a field to a string.
    std::string to_string(
            xtypes::ReadableDynamicDataRef field,
            const std::string& field_name) const;

    /// If an unknown conversion is requested, this string will get passed along
    /// to the UnknownFieldToStringCast exception that gets thrown.
    ///
    /// Ideally this string should contain information like:
    /// 1. What middleware is using the conversion?
    /// 2. What message+topic pair is using the conversion?
    std::string details;

};

//==============================================================================
/// \brief Exception that gets thrown by FieldToString when it's unknown how to
/// convert a given field type into a string.
class UnknownFieldToStringCast : public std::runtime_error
{
public:

    UnknownFieldToStringCast(
            const std::string& type_info,
            const std::string& field_name,
            const std::string& details);

    const std::string& type() const;

    const std::string& field_name() const;

private:

    const std::string _type;
    const std::string _field_name;

};

} // namespace soss

#endif // SOSS__FIELDTOSTRING_HPP
