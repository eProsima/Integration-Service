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

#ifndef SOSS__JSON__CONVERSION_HPP
#define SOSS__JSON__CONVERSION_HPP

#include <soss/json/json.hpp>
#include <soss/Message.hpp>

#include <soss/Soss_export.hpp>

namespace soss {
namespace json {

using Json = nlohmann::json;

//==============================================================================
/// Convert from a soss message to a JSON message
Json SYSTEM_HANDLE_EXPORT convert(const soss::Message& input);

/// Convert from a JSON message to a soss message
soss::Message SYSTEM_HANDLE_EXPORT convert(const std::string& type, const Json& input);

//==============================================================================
/// Convert from a Json object to a string. The following types are supported:
/// * string
/// * boolean
/// * number_integer
/// * number_unsigned
/// * number_float
SYSTEM_HANDLE_EXPORT std::string to_string(const Json& input);

} // namespace json
} // namespace soss


#endif // SOSS__JSON__CONVERSION_HPP
