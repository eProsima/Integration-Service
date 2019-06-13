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

#include <soss/StringTemplate.hpp>
#include <soss/FieldToString.hpp>

namespace soss {

//==============================================================================
class StringTemplate::Implementation
{
public:

  Implementation(
      const std::string& template_string,
      const std::string& usage_details)
    : converter(usage_details)
  {
    std::size_t last_start = 0;
    std::size_t start = template_string.find('{', 0);
    while(start < template_string.size())
    {
      components.push_back(template_string.substr(last_start, start));

      const std::size_t end = template_string.find('}', start);
      if(end == std::string::npos)
      {
        throw InvalidTemplateFormat(template_string, usage_details);
      }

      substitutions[components.size()] = template_string.substr(start+1, end);
      // We use an empty string to represent components that will get
      // substituted later.
      components.push_back("");

      last_start = start;
      start = template_string.find('{', start);
    }
  }

  std::string compute_string(const soss::Message& message) const
  {
    std::string result;

    SubstitutionMap::const_iterator substitute_it = substitutions.begin();
    for(std::size_t i=0; i < components.size(); ++i)
    {
      if(substitute_it != substitutions.end() && substitute_it->first == i)
      {
        const std::string& field_name = substitute_it->second;
        const soss::Message::const_iterator field_it =
            message.data.find(field_name);

        if(field_it == message.data.end())
        {
          throw UnavailableMessageField(field_name, converter.details);
        }

        result += converter.to_string(field_it);
        ++substitute_it;
        continue;
      }

      result += components[i];
    }

    return result;
  }

  FieldToString converter;

  /// This contains a vector of string literal components.
  std::vector<std::string> components;

  // This uses an ordered map so that we can iterate through it linearly as we
  // perform substitutions.
  using SubstitutionMap = std::map<std::size_t, std::string>;

  /// Right now this simply maps a component index to a soss::Message field name.
  /// In the future, we can replace std::string with an abstract Substitution
  /// class that gets factory generated based on what type of substitution is
  /// requested. For right now we've only implemented "message.field"
  /// substitutions.
  SubstitutionMap substitutions;

};

//==============================================================================
StringTemplate::StringTemplate(
    const std::string& template_string,
    const std::string& usage_details)
  : pimpl(new Implementation(template_string, usage_details))
{
  // Do nothing
}

//==============================================================================
std::string StringTemplate::compute_string(const soss::Message& message) const
{
  return pimpl->compute_string(message);
}

//==============================================================================
std::string& StringTemplate::usage_details()
{
  return pimpl->converter.details;
}

//==============================================================================
const std::string& StringTemplate::usage_details() const
{
  return pimpl->converter.details;
}

//==============================================================================
InvalidTemplateFormat::InvalidTemplateFormat(
    const std::string& template_string,
    const std::string& details)
  : std::runtime_error(
      std::string()
      + "ERROR: Template string [" + template_string + "] was incorrectly "
      + "formatted. Details: " + details),
    _template_string(template_string)
{
  // Do nothing
}

//==============================================================================
const std::string& InvalidTemplateFormat::template_string() const
{
  return _template_string;
}

//==============================================================================
UnavailableMessageField::UnavailableMessageField(
    const std::string& field_name,
    const std::string& details)
  : std::runtime_error(
      std::string()
      + "ERROR: Unable to find a required field [" + field_name
      + "]. Details: " + details),
    _field_name(field_name)
{
  // Do nothing
}

//==============================================================================
const std::string& UnavailableMessageField::field_name() const
{
  return _field_name;
}

} // namespace soss
