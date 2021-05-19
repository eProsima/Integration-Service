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

#ifndef _IS_CORE_RUNTIME_STRINGTEMPLATE_HPP_
#define _IS_CORE_RUNTIME_STRINGTEMPLATE_HPP_

#include <is/core/Message.hpp>
#include <is/core/export.hpp>
#include <is/core/runtime/FieldToString.hpp>

#include <stdexcept>

namespace eprosima {
namespace is {
namespace core {

/**
 * @class StringTemplate
 *        Allows to create a partially filled string with certain parameterizable
 *        fields that can be replaced during runtime.
 *        It is also possible to specify some details on how the template
 *        should be used.
 *
 *        More information about how to construct and properly use it is available on
 *        the StringTemplate constructor.
 */
class IS_CORE_API StringTemplate
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] template_string A string that describes the desired template.
     *            Varying components of the string must be wrapped in curly braces `{}`.
     *            Currently only `{message.<field>}` variables are supported.
     *            The varying components of the string will be replaced by the value of the
     *            requested field when `compute_string()` is called.
     *
     * @param[in] usage_details A string that describes how this StringTemplate is being used.
     */
    StringTemplate(
            const std::string& template_string,
            const std::string& usage_details);

    /**
     * @brief Copy constructor.
     *
     * @param[in] other const reference to a StringTemplate instance to be copied.
     */
    StringTemplate(
            const StringTemplate& other);

    /**
     * @brief Move constructor.
     *
     * @param[in] other Movable reference to a StringTemplate instance.
     */
    StringTemplate(
            StringTemplate&& other);

    /**
     * @brief Destructor.
     */
    ~StringTemplate();

    /**
     * @brief Computes the desired output string, given the input message.
     *
     * @param[in] message The message used to compute the string template parameters.
     *
     * @returns The computed string with the required substitutions properly made.
     */
    const std::string compute_string(
            const eprosima::xtypes::DynamicData& message) const;

    /**
     * @brief Gets a mutable reference to the usage_details for this StringTemplate.
     *
     * @returns The mutable reference to the `usage_details` string.
     */
    std::string& usage_details();

    /**
     * @brief Gets a const reference to the usage_details for this StringTemplate.
     *
     * @returns A const reference to the `usage_details` string.
     */
    const std::string& usage_details() const;

private:

    /**
     * @class Implementation
     *        Defines the actual implementation of the StringTemplate class.
     *
     *        Allows to use the *pimpl* procedure to separate the implementation
     *        from the interface of StringTemplate.
     *
     *        Methods named equal to some StringTemplate method will not be
     *        documented again. Usually, the interface class will call
     *        `_pimpl->method()`, but the functionality and parameters
     *        are exactly the same.
     */
    class Implementation;

    /**
     * Class members.
     */

    std::unique_ptr<Implementation> _pimpl;
};

/**
 * @class InvalidTemplateFormat
 *        Runtime error that gets thrown when a certain runtime substitution
 *        string template is malformed in the *YAML* file.
 */
class InvalidTemplateFormat : public std::runtime_error
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] template_string The source string containing the malformed template.
     *
     * @param[in] details Correct usage details of this template.
     */
    InvalidTemplateFormat(
            const std::string& template_string,
            const std::string& details);

    /**
     * @brief Destructor.
     */
    ~InvalidTemplateFormat() = default;

    /**
     * @brief Gets a const reference to the malformed StringTemplate.
     *
     * @returns A const reference to the template.
     */
    const std::string& template_string() const;

private:

    const std::string _template_string;
};

/**
 * @class UnavailableMessageField
 *        Runtime error that gets thrown when a certain field, required to perform
 *        the substitution in a StringTemplate, is unavailable within the provided Dynamic Data.
 */
class UnavailableMessageField : public std::runtime_error
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] field_name The field which was not found during the substitution.
     *
     * @param[in] details Details on how to use this template.
     */
    UnavailableMessageField(
            const std::string& field_name,
            const std::string& details);

    /**
     * @brief Gets a const reference to the field's name.
     *
     * @returns A const reference to the string representing the field's name.
     */
    const std::string& field_name() const;

private:

    const std::string _field_name;

};

} //  namespace core
} //  namespace is
} //  namespace eprosima

#endif // _IS_CORE_RUNTIME_STRINGTEMPLATE_HPP_
