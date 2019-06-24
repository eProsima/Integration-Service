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

#include <soss/json/conversion.hpp>
#include <soss/utilities.hpp>

#include <unordered_map>

namespace soss {
namespace json {

using field_iterator = std::map<std::string, Field>::iterator;
using const_field_iterator = std::map<std::string, Field>::const_iterator;

using nlohmann::detail::value_t;

using ToSossMap =
  std::unordered_map<
      value_t,
      std::function<Field(const Json& input)>>;

using ToSossArrayMap =
  std::unordered_map<
      value_t,
      std::function<Field(const Json& input)>>;

using ToJsonMap =
  std::unordered_map<
      std::string,
      std::function<Json(const const_field_iterator& input)>>;

using ToStringMap =
  std::unordered_map<
      value_t,
      std::function<std::string(const Json& input)>>;

//==============================================================================
struct Converter
{
  Converter()
  {
    // Conversions to/from soss::Message
    add_object_conversion();
    add_array_conversions();
    add_primitive_conversion<std::string>(value_t::string);
    add_primitive_conversion<bool>(value_t::boolean);
    add_primitive_conversion<int64_t>(value_t::number_integer);
    add_primitive_conversion<uint64_t>(value_t::number_unsigned);
    add_primitive_conversion<double>(value_t::number_float);

    // Conversions to std::string
    add_string_forwarding();
    add_primitive_to_string<bool>(value_t::boolean);
    add_primitive_to_string<int64_t>(value_t::number_integer);
    add_primitive_to_string<uint64_t>(value_t::number_unsigned);
    add_primitive_to_string<double>(value_t::number_float);
  }


  // ------------ Functions for configuring the converter ------------

  void add_object_conversion()
  {
    map_to_soss[value_t::object] =
        [](const Json& input) -> Field
    {
      soss::Message output;
      convert_from_json_object(input, output);
      return soss::make_field<soss::Message>(std::move(output));
    };

    map_to_json[typeid(soss::Message).name()] =
        [](const const_field_iterator& input) -> Json
    {
      const soss::Message& from_msg = *input->second.cast<soss::Message>();

      Json output;
      convert_from_soss_message(from_msg, output);
      return output;
    };
  }

  void add_array_conversions()
  {
    map_to_soss[nlohmann::json::value_t::array] =
        [](const Json& input) -> Field
    {
      using namespace nlohmann::detail;

      value_t array_type = value_t::discarded;
      std::string failure_message;
      std::size_t index = 0;
      ToSossArrayMap::const_iterator convert_it =
          instance().map_to_soss_array.end();

      for(Json::const_iterator it = input.begin();
          it != input.end(); ++it, ++index)
      {
        value_t entry_type = it->type();

        if(array_type != value_t::discarded)
        {
          // If the array type has a value besides "discarded", then it has been
          // initialized, and we should check to make sure that the current
          // entry is the same type.
          if(entry_type != array_type)
          {
            // If they are not the same type, then we'll tell the parser to
            // discard this array.
            failure_message =
                "[soss::json::array_conversion] Mismatch in array types. "
                "Expected [" + std::to_string(static_cast<int>(array_type))
                + "] instead got ["
                + std::to_string(static_cast<int>(it->type())) + "]:\n"
                + input.dump();
            array_type = value_t::discarded;
            break;
          }

          // If they are the same type, then we can move on to the next entry.
          continue;
        }

        // If the array type is currently uninitialized, we should make sure
        // that a conversion is available for the current entry type, and save
        // it for later if it's found.
        convert_it = instance().map_to_soss_array.find(entry_type);
        if(convert_it == instance().map_to_soss_array.end())
        {
          // If the type of this array entry is "discarded" or "null", then we
          // can't parse it, so we'll quit early.
          array_type = value_t::discarded;
          failure_message =
              "[soss::json::array_conversion] Invalid entry type at array "
              "index [" + std::to_string(index) + "]:\n" + input.dump();
          break;
        }

        // If this is the first entry in the array and it has a convertable type
        // then we will save its type as the array type, and save its entry in
        // the conversion map for later use.
        array_type = it->type();
      }

      if(value_t::discarded == array_type)
      {
        throw std::runtime_error(failure_message);
      }

      return convert_it->second(input);
    };

    add_object_array_conversion();
    add_primitive_array_conversion<std::string>(value_t::string);
    add_primitive_array_conversion<bool, uint64_t>(value_t::boolean);
    add_primitive_array_conversion<int64_t>(value_t::number_integer);
    add_primitive_array_conversion<uint64_t>(value_t::number_unsigned);
    add_primitive_array_conversion<double>(value_t::number_float);

    // Note: the map_to_json will be filled in by the individual
    // add_*_array_conversion functions.
  }

  template<typename T>
  void add_primitive_conversion(nlohmann::json::value_t type)
  {
    map_to_soss[type] =
        [](const Json& input) -> Field
    {
      return soss::Convert<T>::make_soss_field(input.get<T>());
    };

    map_to_json[typeid(T).name()] =
        [](const const_field_iterator& input) -> Json
    {
      T output;
      soss::Convert<T>::from_soss_field(input, output);
      return Json(output);
    };
  }

  template<typename JsonT, typename SossT=JsonT>
  void add_primitive_array_conversion(nlohmann::json::value_t type)
  {
    map_to_soss_array[type] =
        [](const Json& input) -> Field
    {
      std::vector<SossT> output;
      output.reserve(input.size());
      for(Json::const_iterator it = input.begin(); it != input.end(); ++it)
      {
        output.push_back(static_cast<SossT>(it->get<JsonT>()));
      }

      return soss::Convert<std::vector<SossT>>::make_soss_field(output);
    };

    map_to_json[typeid(std::vector<SossT>).name()] =
        [](const const_field_iterator& input) -> Json
    {
      std::vector<SossT> content;
      soss::Convert<std::vector<SossT>>::from_soss_field(input, content);

      std::vector<JsonT> output;
      output.reserve(content.size());
      for(const SossT& c : content)
        output.push_back(static_cast<JsonT>(c));

      return Json(output);
    };
  }

  void add_object_array_conversion()
  {
    map_to_soss_array[value_t::object] =
        [](const Json& input) -> Field
    {
      std::vector<soss::Message> output;
      output.reserve(input.size());
      for(Json::const_iterator it = input.begin(); it != input.end(); ++it)
      {
        soss::Message output_it;
        convert_from_json_object(*it, output_it);
        output.emplace_back(std::move(output_it));
      }

      return soss::Convert<std::vector<soss::Message>>
              ::make_soss_field(std::move(output));
    };

    map_to_json[typeid(std::vector<soss::Message>).name()] =
        [](const const_field_iterator& input) -> Json
    {
      const std::vector<soss::Message>& input_array =
          *input->second.cast<std::vector<soss::Message>>();

      std::vector<Json> output;
      output.resize(input_array.size());
      for(std::size_t i = 0; i < input_array.size(); ++i)
        convert_from_soss_message(input_array[i], output[i]);

      return Json(output);
    };
  }

  void add_string_forwarding()
  {
    map_to_string[value_t::string] =
        [](const Json& input) -> std::string
    {
      return input.get<std::string>();
    };
  }

  template<typename T>
  void add_primitive_to_string(nlohmann::json::value_t type)
  {
    map_to_string[type] =
        [](const Json& input) -> std::string
    {
      return std::to_string(input.get<T>());
    };
  }

  static void convert_from_json_object(const Json& input, soss::Message& output)
  {
    for(Json::const_iterator it = input.begin(); it != input.end(); ++it)
    {
      const auto& conversion = instance().map_to_soss.at(it.value().type());
      output.data[it.key()] = conversion(it.value());
    }
  }

  static void convert_from_soss_message(const soss::Message& input, Json& output)
  {
    for(const_field_iterator it = input.data.begin(); it != input.data.end(); ++it)
    {
      const std::string& key = it->first;
      const Field& value = it->second;

      const auto& conversion = instance().map_to_json.at(value.type());

      output[key] = conversion(it);
    }
  }


  // ------------ Functions for using converter ------------

  static Json to_json(const Message& input)
  {
    Json output;
    convert_from_soss_message(input, output);

    return output;
  }

  static Message to_soss(const std::string& type, const Json& input)
  {
    if(!input.is_object())
    {
      throw std::runtime_error(
            "[soss::json::convert] Received a JSON-encoded message that is not "
            "an object:\n" + input.dump());
    }

    soss::Message output;
    output.type = type;
    convert_from_json_object(input, output);

    return output;
  }

  static std::string to_string(const Json& input)
  {
    const auto& conversion_map = instance().map_to_string;
    const auto it = conversion_map.find(input.type());
    if(it == conversion_map.end())
    {
      throw std::runtime_error(
            "[soss::json] Cannot convert from Json type ["
            + std::to_string(static_cast<int>(input.type()))
            + "] to std::string");
    }

    return it->second(input);
  }

  static Converter& instance()
  {
    static Converter converter;
    return converter;
  }

  ToSossMap map_to_soss;
  ToSossArrayMap map_to_soss_array;
  ToJsonMap map_to_json;
  ToStringMap map_to_string;
};

//==============================================================================
Json convert(const soss::Message& input)
{
  return Converter::to_json(input);
}

//==============================================================================
soss::Message convert(const std::string& type, const Json& message)
{
  return Converter::to_soss(type, message);
}

//==============================================================================
std::string to_string(const Json& input)
{
  return Converter::to_string(input);
}

} // namespace json
} // namespace soss
