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

#ifndef _IS_CORE_RUNTIME_FIELDTOSTRING_HPP_
#define _IS_CORE_RUNTIME_FIELDTOSTRING_HPP_

#include <is/core/Message.hpp>
#include <is/utils/Log.hpp>


#include <stdexcept>

// TODO jamoralp add Logger
namespace eprosima {
namespace is {
namespace core {

/**
 * @class FieldToString
 *        Convenience class for converting simple field types into strings.
 *        It is useful to discern between the different Dynamic Types that may
 *        be requested to be replaced in a certain StringTemplate, and perform
 *        the conversion accordingly.
 */
class FieldToString
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] usage_details Sets the details for how the conversion should be used.
     */
    FieldToString(
            const std::string& usage_details);

    /**
     * @brief Copy Constructor.
     *
     * @param[in] other The instance to be copied.
     */
    FieldToString(
            const FieldToString& other);

    /**
     * @brief Move Constructor.
     *
     * @param[in] other The instance to be moved.
     */
    FieldToString(
            FieldToString&& other);

    /**
     * @brief Destructor.
     */
    ~FieldToString() = default;

    /**
     * @brief Converts a certain field to a string, given the field name.
     *
     * @param[in] field Reference to the Dynamic Data instance representing the field's values.
     *
     * @param[in] field_name The specific field whose value should be retrieved.
     *
     * @returns A const string representation of the requested field.
     */
    const std::string to_string(
            eprosima::xtypes::ReadableDynamicDataRef field,
            const std::string& field_name) const;

    /**
     * @brief Gets a const reference to the details attribute.
     *
     * @returns A const string reference to "details".
     */
    const std::string& details() const;

    /**
     * @brief Gets a mutable reference to the details attribute.
     *
     * @returns A non-const string reference to "details".
     */
    std::string& details();

private:

    /**
     * @class Implementation
     *        Defines the actual implementation of the FieldToString class.
     *
     *        Allows to use the *pimpl* procedure to separate the implementation
     *        from the interface of FieldToString.
     *
     *        Methods named equal to some FieldToString method will not be
     *        documented again. Usually, the interface class will call
     *        `_pimpl->method()`, but the functionality and parameters
     *        are exactly the same.
     */
    class Implementation;

    /**
     * Class members.
     */

    Implementation& _pimpl;

    /**
     * If an unknown conversion is requested, this string will get passed along
     * to the UnknownFieldToStringCast exception that gets thrown.
     *
     * Ideally this string should contain information like:
     * 1. What *middleware* is using the conversion?
     * 2. Which *message + topic* pair is using the conversion?
     */
    std::string _details;
};

/**
 * @class UnknownFieldToStringCast
 *        Exception that gets thrown by FieldToString when it's unknown how to
 *        convert a given field type into a string.
 */
class UnknownFieldToStringCast : public std::runtime_error
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] type The type kind that should have been cast to a string.
     *
     * @param[in] field_name The field whose conversion to string was unsuccessfully attempted.
     *
     * @param[in] details The details on how the conversion is being done.
     */
    UnknownFieldToStringCast(
            const std::string& type,
            const std::string& field_name,
            const std::string& details);

    /**
     * @brief Destructor.
     */
    ~UnknownFieldToStringCast() = default;

    /**
     * @brief Getter method for `_type` parameter.
     *
     * @returns A const reference to the field type string.
     */
    const std::string& type() const;

    /**
     * @brief Getter method for the field's name.
     *
     * @returns A const reference to the field's name string.
     */
    const std::string& field_name() const;

private:

    /**
     * Class members.
     */

    const std::string _type;
    const std::string _field_name;
};

} //  namespace core
} //  namespace is
} //  namespace eprosima

#endif //  _IS_CORE_RUNTIME_FIELDTOSTRING_HPP_
