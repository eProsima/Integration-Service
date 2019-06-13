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

#ifndef SOSS__STRINGTEMPLATE_HPP
#define SOSS__STRINGTEMPLATE_HPP

#include <soss/Message.hpp>

namespace soss {

//==============================================================================
class StringTemplate
{
public:

  /// Constructor
  ///
  /// \param[in] template_string
  ///   A string that describes the desired template. Varying components of the
  ///   string must be wrapped in curly braces {}. Currently only
  ///   {message.<field>} variables are supported. Those components of the
  ///   string will be replaced by the value of the requested field when
  ///   compute_string() is called.
  ///
  /// \param[in] usage_details
  ///   A string that describes how this StringTemplate is being used. Ideally
  ///   this should contain information like:
  ///   1. What middleware is using the template?
  ///   2. What message+topic pair is using the conversion?
  StringTemplate(
      const std::string& template_string,
      const std::string& usage_details);

  /// Compute the desired output string given the input message.
  std::string compute_string(const soss::Message& message) const;

  /// Mutable reference to the usage details for this StringTemplate
  std::string& usage_details();

  /// Const reference to the usage details for this StringTemplate
  const std::string& usage_details() const;

private:

  class Implementation;
  std::unique_ptr<Implementation> pimpl;

};

//==============================================================================
class InvalidTemplateFormat : public std::runtime_error
{
public:

  InvalidTemplateFormat(
      const std::string& template_string,
      const std::string& details);

  const std::string& template_string() const;

private:

  const std::string _template_string;

};

//==============================================================================
class UnavailableMessageField : public std::runtime_error
{
public:

  UnavailableMessageField(
      const std::string& field_name,
      const std::string& details);

  const std::string& field_name() const;

private:

  const std::string _field_name;

};

} // namespace soss

#endif // SOSS__STRINGTEMPLATE_HPP
