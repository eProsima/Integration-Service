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
#include <stack>

#include <iostream>

namespace soss {
namespace json {

Json::const_reference access_json_value(
        const xtypes::DynamicData::ReadableNode& soss_node,
        Json::const_pointer json_node)
{
  if(soss_node.parent().type().is_collection_type())
  {
    size_t index = soss_node.from_index();
    return json_node->operator[](index);
  }
  if(soss_node.parent().type().is_aggregation_type())
  {
    const std::string& member_name = soss_node.from_member()->name();
    return json_node->operator[](member_name);
  }
  static Json none;
  return none;
}

bool json_to_soss(
        const Json& json_message,
        xtypes::DynamicData& soss_message)
{
  std::stack<Json::const_pointer> json_stack;
  json_stack.push(&json_message);

  return soss_message.for_each([&](xtypes::DynamicData::WritableNode& soss_node)
  {
    if(!soss_node.has_parent()) return; // avoid the first level already pushed into stack.

    while(json_stack.size() > soss_node.deep())
    {
      json_stack.pop();
    }

    switch(soss_node.type().kind())
    {
      case xtypes::TypeKind::STRUCTURE_TYPE:
        json_stack.push(&access_json_value(soss_node, json_stack.top()));
        break;
      case xtypes::TypeKind::SEQUENCE_TYPE:
        json_stack.push(&access_json_value(soss_node, json_stack.top()));
        soss_node.data().resize(json_stack.top()->size());
        break;
      case xtypes::TypeKind::ARRAY_TYPE:
        json_stack.push(&access_json_value(soss_node, json_stack.top()));
        break;
      case xtypes::TypeKind::STRING_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<std::string>());
        break;
      case xtypes::TypeKind::CHAR_8_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<char>());
        break;
      case xtypes::TypeKind::INT_8_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<int8_t>());
        break;
      case xtypes::TypeKind::UINT_8_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<uint8_t>());
        break;
      case xtypes::TypeKind::INT_16_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<int16_t>());
        break;
      case xtypes::TypeKind::UINT_16_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<uint16_t>());
        break;
      case xtypes::TypeKind::INT_32_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<int32_t>());
        break;
      case xtypes::TypeKind::UINT_32_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<uint32_t>());
        break;
      case xtypes::TypeKind::INT_64_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<int64_t>());
        break;
      case xtypes::TypeKind::UINT_64_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<uint64_t>());
        break;
      case xtypes::TypeKind::FLOAT_32_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<float>());
        break;
      case xtypes::TypeKind::FLOAT_64_TYPE:
        soss_node.data().value(access_json_value(soss_node, json_stack.top()).get<double>());
        break;
      default:
        throw false;
    }
  });
}


Json::reference add_json_node(
        const xtypes::DynamicData::ReadableNode& soss_node,
        Json::pointer json_node)
{
  if(soss_node.parent().type().is_collection_type())
  {
    size_t index = soss_node.from_index();
    json_node->operator[](index) = {};
    return json_node->operator[](index);
  }
  if(soss_node.parent().type().is_aggregation_type())
  {
    const std::string& member_name = soss_node.from_member()->name();
    json_node->operator[](member_name) = {};
    return json_node->operator[](member_name);
  }
  static Json none;
  return none;
}

bool soss_to_json(
        const xtypes::DynamicData& soss_message,
        Json& json_message)
{
  std::stack<Json::pointer> json_stack;
  json_stack.push(&json_message);

  return soss_message.for_each([&](const xtypes::DynamicData::ReadableNode& soss_node)
  {
    if(!soss_node.has_parent()) return; // avoid the first struct level already pushed into stack.

    while(json_stack.size() > soss_node.deep())
    {
      json_stack.pop();
    }

    switch(soss_node.type().kind())
    {
      case xtypes::TypeKind::STRUCTURE_TYPE:
        json_stack.push(&add_json_node(soss_node, json_stack.top()));
        break;
      case xtypes::TypeKind::SEQUENCE_TYPE:
        json_stack.push(&add_json_node(soss_node, json_stack.top()));
        break;
      case xtypes::TypeKind::ARRAY_TYPE:
        json_stack.push(&add_json_node(soss_node, json_stack.top()));
        break;
      case xtypes::TypeKind::STRING_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<std::string>();
        break;
      case xtypes::TypeKind::CHAR_8_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<char>();
        break;
      case xtypes::TypeKind::INT_8_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<int8_t>();
        break;
      case xtypes::TypeKind::UINT_8_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<uint8_t>();
        break;
      case xtypes::TypeKind::INT_16_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<int16_t>();
        break;
      case xtypes::TypeKind::UINT_16_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<uint16_t>();
        break;
      case xtypes::TypeKind::INT_32_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<int32_t>();
        break;
      case xtypes::TypeKind::UINT_32_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<uint32_t>();
        break;
      case xtypes::TypeKind::INT_64_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<int64_t>();
        break;
      case xtypes::TypeKind::UINT_64_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<uint64_t>();
        break;
      case xtypes::TypeKind::FLOAT_32_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<float>();
        break;
      case xtypes::TypeKind::FLOAT_64_TYPE:
        add_json_node(soss_node, json_stack.top()) = soss_node.data().value<double>();
        break;
      default:
        throw false;
    }
  });
}

//==============================================================================
Json convert(const xtypes::DynamicData& soss_message)
{
  Json json_message;
  soss_to_json(soss_message, json_message);
  return json_message;
}

//==============================================================================
xtypes::DynamicData convert(const xtypes::DynamicType& type, const Json& json_message)
{
  xtypes::DynamicData soss_message(type);
  json_to_soss(json_message, soss_message);
  return soss_message;
}

} // namespace json
} // namespace soss
