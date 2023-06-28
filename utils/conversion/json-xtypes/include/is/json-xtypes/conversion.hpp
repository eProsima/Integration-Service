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

#ifndef _JSON_XTYPES__INCLUDE__CONVERSION_HPP_
#define _JSON_XTYPES__INCLUDE__CONVERSION_HPP_

#include <is/json-xtypes/export.hpp>
#include <is/json-xtypes/json.hpp>
#include <is/core/Message.hpp>

namespace xtypes = eprosima::xtypes;

namespace eprosima {
namespace is {
namespace json_xtypes {

using Json = nlohmann::json;

/**
 * @class UnsupportedType
 *        Exception to be thrown when a type not yet supported by the `json-xtypes` library
 *        is attempted to be converted.
 */
class UnsupportedType : public std::exception
{
public:

    UnsupportedType(
            const std::string& type_name)
        : std::exception()
        , type_name_(type_name)
    {
        std::ostringstream err;
        err << "[json-xtypes] Unsupported type '" << type_name_ << "'";
        err_str_ = err.str();
    }

    const char* what() const noexcept
    {
        return err_str_.c_str();
    }

private:

    const std::string type_name_;
    std::string err_str_;
};

/**
 * @brief Convert a DynamicData instance into an equivalent JSON format representation
 *        of the very same data instance.
 *
 * @param[in] input The xTypes DynamicData to be converted to JSON format.
 *
 * @param[in] submember The submember of the Json data where the converted field will be inserted.
 *            Defaults to empty.
 *
 * @returns A Json object representing the converted data.
 */
Json IS_JSON_XTYPES_API convert(
        const xtypes::DynamicData& input,
        const std::string submember = "");

/**
 * @brief Convert a Json data representation into its equivalent xTypes DynamicData instance.
 *
 * @param[in] type The DynamicType used to construct the DynamicData, using the Json input.
 *
 * @param[in] input The Json to be converted to a DynamicData.
 *
 * @param[in] submember The submember of the Json value to be converted to xTypes. By default, it is empty.
 *
 * @returns The resulting DynamicData converted data instance.
 */
xtypes::DynamicData IS_JSON_XTYPES_API convert(
        const xtypes::DynamicType& type,
        const Json& input,
        const std::string submember = "");

} //  namespace json_xtypes
} //  namespace is
} //  namespace eprosima


#endif //  _JSON_XTYPES__INCLUDE__CONVERSION_HPP_
