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

#include "Encoding.hpp"
#include "Endpoint.hpp"

#include <soss/json/conversion.hpp>
#include <soss/json/json.hpp>

#include <unordered_set>

namespace soss {
namespace websocket {

using json::Json;
using nlohmann::detail::value_t;

//==============================================================================
// message fields
const std::string JsonOpKey = "op";
const std::string JsonIdKey = "id";
const std::string JsonTopicNameKey = "topic";
const std::string JsonTypeNameKey = "type";
const std::string JsonMsgKey = "msg";
const std::string JsonServiceKey = "service";
const std::string JsonArgsKey = "args";
const std::string JsonValuesKey = "values";
const std::string JsonResultKey = "result";


// op codes
const std::string JsonOpAdvertiseTopicKey = "advertise";
const std::string JsonOpUnadvertiseTopicKey = "unadvertise";
const std::string JsonOpPublishKey = "publish";
const std::string JsonOpSubscribeKey = "subscribe";
const std::string JsonOpUnsubscribeKey = "unsubscribe";
const std::string JsonOpServiceRequestKey = "call_service";
const std::string JsonOpAdvertiseServiceKey = "advertise_service";
const std::string JsonOpUnadvertiseServiceKey = "unadvertise_service";
const std::string JsonOpServiceResponseKey = "service_response";

//==============================================================================
static void throw_missing_key(
    const Json& object,
    const std::string& key)
{
  const std::string op_code = object.at("op").get<std::string>();
  throw std::runtime_error(
        "[soss::websocket::rosbridge_v2] Incoming websocket message with op "
        "code [" + op_code + "] is missing the required field [" + key
        + "]:\n" + object.dump());
}

//==============================================================================
static std::string get_optional_string(
    const Json& object,
    const std::string& key)
{
  const auto it = object.find(key);
  return it == object.end()? "" : soss::json::to_string(it.value());
}

//==============================================================================
static std::string get_required_string(
    const Json& object,
    const std::string& key)
{
  const auto it = object.find(key);
  if(it == object.end())
    throw_missing_key(object, key);

  return soss::json::to_string(it.value());
}

//==============================================================================
static soss::Message get_required_msg(
    const Json& object,
    const std::string& key)
{
  const auto it = object.find(key);
  if(it == object.end())
    throw_missing_key(object, key);

  return json::convert("", it.value());
}

//==============================================================================
class RosbridgeV2_0 : public Encoding
{
public:

  void interpret_websocket_msg(
      const std::string& msg_str,
      Endpoint& endpoint,
      std::shared_ptr<void> connection_handle) const override
  {
    const auto msg = Json::parse(msg_str);

    const auto op_it = msg.find(JsonOpKey);
    if(op_it == msg.end())
    {
      throw std::runtime_error(
            "[soss::websocket::rosbridge_v2] Incoming message was missing "
            "the required op code: " + msg_str);
    }

    const std::string& op_str = op_it.value().get<std::string>();

    // Publish is the most likely type of message to be received, so we'll check
    // for that type first.
    if(op_str == JsonOpPublishKey)
    {
      endpoint.receive_publication_ws(
            get_required_string(msg, JsonTopicNameKey),
            get_required_msg(msg, JsonMsgKey),
            std::move(connection_handle));
      return;
    }

    // A service request is the roughly the second/third most likely type of
    // message to be received, so we'll check for that type next.
    if(op_str == JsonOpServiceRequestKey)
    {
      endpoint.receive_service_request_ws(
            get_required_string(msg, JsonServiceKey),
            get_required_msg(msg, JsonArgsKey),
            get_optional_string(msg, JsonIdKey),
            std::move(connection_handle));
      return;
    }

    // A service response is the roughly the second/third most likely type of
    // message to be received, so we'll check for that type next.
    if(op_str == JsonOpServiceResponseKey)
    {
      endpoint.receive_service_response_ws(
            get_required_string(msg, JsonServiceKey),
            get_required_msg(msg, JsonValuesKey),
            get_optional_string(msg, JsonIdKey),
            std::move(connection_handle));
      return;
    }

    if(op_str == JsonOpAdvertiseTopicKey)
    {
      endpoint.receive_topic_advertisement_ws(
            get_required_string(msg, JsonTopicNameKey),
            get_required_string(msg, JsonTypeNameKey),
            get_optional_string(msg, JsonIdKey),
            std::move(connection_handle));
      return;
    }

    if(op_str == JsonOpUnadvertiseTopicKey)
    {
      endpoint.receive_topic_unadvertisement_ws(
            get_required_string(msg, JsonTopicNameKey),
            get_optional_string(msg, JsonIdKey),
            std::move(connection_handle));
      return;
    }

    if(op_str == JsonOpSubscribeKey)
    {
      endpoint.receive_subscribe_request_ws(
            get_required_string(msg, JsonTopicNameKey),
            get_optional_string(msg, JsonTypeNameKey),
            get_optional_string(msg, JsonIdKey),
            std::move(connection_handle));
      return;
    }

    if(op_str == JsonOpUnsubscribeKey)
    {
      endpoint.receive_unsubscribe_request_ws(
            get_required_string(msg, JsonTopicNameKey),
            get_optional_string(msg, JsonIdKey),
            std::move(connection_handle));
      return;
    }

    if(op_str == JsonOpAdvertiseServiceKey)
    {
      endpoint.receive_service_advertisement_ws(
            get_required_string(msg, JsonServiceKey),
            get_required_string(msg, JsonTypeNameKey),
            std::move(connection_handle));
      return;
    }

    if(op_str == JsonOpUnadvertiseServiceKey)
    {
      endpoint.receive_service_unadvertisement_ws(
            get_required_string(msg, JsonServiceKey),
            get_optional_string(msg, JsonTypeNameKey),
            std::move(connection_handle));
    }
  }

  std::string encode_publication_msg(
      const std::string& topic_name,
      const std::string& /*topic_type*/,
      const std::string& id,
      const soss::Message& msg) const override
  {
    Json output;
    output[JsonOpKey] = JsonOpPublishKey;
    output[JsonTopicNameKey] = topic_name;
    output[JsonMsgKey] = json::convert(msg);
    if(!id.empty())
      output[JsonIdKey] = id;

    return output.dump();
  }

  std::string encode_service_response_msg(
      const std::string& service_name,
      const std::string& /*service_type*/,
      const std::string& id,
      const soss::Message& response,
      const bool result) const override
  {
    Json output;
    output[JsonOpKey] = JsonOpServiceResponseKey;
    output[JsonServiceKey] = service_name;
    output[JsonValuesKey] = json::convert(response);
    output[JsonResultKey] = result;
    if(!id.empty())
      output[JsonIdKey] = id;

    return output.dump();
  }

  std::string encode_subscribe_msg(
      const std::string& topic_name,
      const std::string& message_type,
      const std::string& id,
      const YAML::Node& /*configuration*/) const override
  {
    // TODO(MXG): Consider parsing the `configuration` for details like
    // throttle_rate, queue_length, fragment_size, and compression
    Json output;
    output[JsonOpKey] = JsonOpSubscribeKey;
    output[JsonTopicNameKey] = topic_name;
    output[JsonTypeNameKey] = message_type;
    if(!id.empty())
      output[JsonIdKey] = id;

    return output.dump();
  }

  std::string encode_advertise_msg(
      const std::string& topic_name,
      const std::string& message_type,
      const std::string& id,
      const YAML::Node& /*configuration*/) const override
  {
    Json output;
    output[JsonOpKey] = JsonOpAdvertiseTopicKey;
    output[JsonTopicNameKey] = topic_name;
    output[JsonTypeNameKey] = message_type;
    if(!id.empty())
      output[JsonIdKey] = id;

    return output.dump();
  }

  std::string encode_call_service_msg(
      const std::string& service_name,
      const std::string& /*service_type*/,
      const soss::Message& service_request,
      const std::string& id,
      const YAML::Node& /*configuration*/) const override
  {
    // TODO(MXG): Consider parsing the `configuration` for details like
    // fragment_size and compression
    Json output;
    output[JsonOpKey] = JsonOpServiceRequestKey;
    output[JsonServiceKey] = service_name;
    output[JsonArgsKey] = json::convert(service_request);
    if(!id.empty())
      output[JsonIdKey] = id;

    return output.dump();
  }

  std::string encode_advertise_service_msg(
      const std::string& service_name,
      const std::string& service_type,
      const std::string& /*id*/,
      const YAML::Node& /*configuration*/) const override
  {
    Json output;
    output[JsonOpKey] = JsonOpAdvertiseServiceKey;
    output[JsonTypeNameKey] = service_type;
    output[JsonServiceKey] = service_name;

    return output.dump();
  }

};

//==============================================================================
EncodingPtr make_rosbridge_v2_0()
{
  return std::make_shared<RosbridgeV2_0>();
}

} // namespace websocket
} // namespace soss
