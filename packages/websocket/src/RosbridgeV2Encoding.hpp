/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef SOSS_WEBSOCKET__ROSBRIDGEV2ENCODING__HPP
#define SOSS_WEBSOCKET__ROSBRIDGEV2ENCODING__HPP

#include "Encoding.hpp"
#include "Endpoint.hpp"
#include "JsonSerializer.hpp"

#include <soss/json/conversion.hpp>

#include <unordered_set>

namespace soss {
namespace websocket {

//==============================================================================
// message fields
extern std::string JsonIdKey;
extern std::string JsonOpKey;
extern std::string JsonTopicNameKey;
extern std::string JsonTypeNameKey;
extern std::string JsonMsgKey;
extern std::string JsonServiceKey;
extern std::string JsonArgsKey;
extern std::string JsonValuesKey;
extern std::string JsonResultKey;


// op codes
extern std::string JsonOpAdvertiseTopicKey;
extern std::string JsonOpUnadvertiseTopicKey;
extern std::string JsonOpPublishKey;
extern std::string JsonOpSubscribeKey;
extern std::string JsonOpUnsubscribeKey;
extern std::string JsonOpServiceRequestKey;
extern std::string JsonOpAdvertiseServiceKey;
extern std::string JsonOpUnadvertiseServiceKey;
extern std::string JsonOpServiceResponseKey;

//==============================================================================
void throw_missing_key(const json::Json& object, const std::string& key);

//==============================================================================
std::string get_optional_string(const json::Json& object, const std::string& key);

//==============================================================================
std::string get_required_string(const json::Json& object, const std::string& key);

//==============================================================================
soss::Message get_required_msg(const json::Json& object, const std::string& key);

//==============================================================================

template<class SerializerT = JsonSerializer>
class RosbridgeV2Encoding : public Encoding {
public:
  inline WsOpcode opcode() const override {
    return SerializerT::opcode;
  }

  void interpret_websocket_msg(
    const std::string& msg_str,
    Endpoint& endpoint,
    std::shared_ptr<void> connection_handle) const override;

  MessagePtrT encode_publication_msg(
    ConMsgManagerPtrT& con_msg_mgr,
    const std::string& topic_name,
    const std::string& /*topic_type*/,
    const std::string& id,
    const soss::Message& msg) const override;

  MessagePtrT encode_service_response_msg(
    ConMsgManagerPtrT& con_msg_mgr,
    const std::string& service_name,
    const std::string& /*service_type*/,
    const std::string& id,
    const soss::Message& response,
    const bool result) const override;

  MessagePtrT encode_subscribe_msg(
    ConMsgManagerPtrT& con_msg_mgr,
    const std::string& topic_name,
    const std::string& message_type,
    const std::string& id,
    const YAML::Node& /*configuration*/) const override;

  MessagePtrT encode_advertise_msg(
    ConMsgManagerPtrT& con_msg_mgr,
    const std::string& topic_name,
    const std::string& message_type,
    const std::string& id,
    const YAML::Node& /*configuration*/) const override;

  MessagePtrT encode_call_service_msg(
    ConMsgManagerPtrT& con_msg_mgr,
    const std::string& service_name,
    const std::string& /*service_type*/,
    const soss::Message& service_request,
    const std::string& id,
    const YAML::Node& /*configuration*/) const override;

  MessagePtrT encode_advertise_service_msg(
    ConMsgManagerPtrT& con_msg_mgr,
    const std::string& service_name,
    const std::string& service_type,
    const std::string& /*id*/,
    const YAML::Node& /*configuration*/) const override;
};

template<class SerializerT>
void RosbridgeV2Encoding<SerializerT>::interpret_websocket_msg(
  const std::string& msg_str,
  Endpoint& endpoint,
  std::shared_ptr<void> connection_handle) const
{
  const auto msg = SerializerT::deserialize(msg_str);

  const auto op_it = msg.find(JsonOpKey);
  if (op_it == msg.end()) {
    throw std::runtime_error(
      "[soss::websocket::rosbridge_v2] Incoming message was missing "
      "the required op code: " + msg_str);
  }

  const std::string& op_str = op_it.value().template get<std::string>();

  // Publish is the most likely type of message to be received, so we'll check
  // for that type first.
  if (op_str == JsonOpPublishKey) {
    endpoint.receive_publication_ws(
      get_required_string(msg, JsonTopicNameKey),
      get_required_msg(msg, JsonMsgKey),
      std::move(connection_handle));
    return;
  }

  // A service request is the roughly the second/third most likely type of
  // message to be received, so we'll check for that type next.
  if (op_str == JsonOpServiceRequestKey) {
    endpoint.receive_service_request_ws(
      get_required_string(msg, JsonServiceKey),
      get_required_msg(msg, JsonArgsKey),
      get_optional_string(msg, JsonIdKey),
      std::move(connection_handle));
    return;
  }

  // A service response is the roughly the second/third most likely type of
  // message to be received, so we'll check for that type next.
  if (op_str == JsonOpServiceResponseKey) {
    endpoint.receive_service_response_ws(
      get_required_string(msg, JsonServiceKey),
      get_required_msg(msg, JsonValuesKey),
      get_optional_string(msg, JsonIdKey),
      std::move(connection_handle));
    return;
  }

  if (op_str == JsonOpAdvertiseTopicKey) {
    endpoint.receive_topic_advertisement_ws(
      get_required_string(msg, JsonTopicNameKey),
      get_required_string(msg, JsonTypeNameKey),
      get_optional_string(msg, JsonIdKey),
      std::move(connection_handle));
    return;
  }

  if (op_str == JsonOpUnadvertiseTopicKey) {
    endpoint.receive_topic_unadvertisement_ws(
      get_required_string(msg, JsonTopicNameKey),
      get_optional_string(msg, JsonIdKey),
      std::move(connection_handle));
    return;
  }

  if (op_str == JsonOpSubscribeKey) {
    endpoint.receive_subscribe_request_ws(
      get_required_string(msg, JsonTopicNameKey),
      get_optional_string(msg, JsonTypeNameKey),
      get_optional_string(msg, JsonIdKey),
      std::move(connection_handle));
    return;
  }

  if (op_str == JsonOpUnsubscribeKey) {
    endpoint.receive_unsubscribe_request_ws(
      get_required_string(msg, JsonTopicNameKey),
      get_optional_string(msg, JsonIdKey),
      std::move(connection_handle));
    return;
  }

  if (op_str == JsonOpAdvertiseServiceKey) {
    endpoint.receive_service_advertisement_ws(
      get_required_string(msg, JsonServiceKey),
      get_required_string(msg, JsonTypeNameKey),
      std::move(connection_handle));
    return;
  }

  if (op_str == JsonOpUnadvertiseServiceKey) {
    endpoint.receive_service_unadvertisement_ws(
      get_required_string(msg, JsonServiceKey),
      get_optional_string(msg, JsonTypeNameKey),
      std::move(connection_handle));
  }
}

template<class SerializerT>
MessagePtrT RosbridgeV2Encoding<SerializerT>::encode_publication_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& topic_name,
  const std::string& /*topic_type*/,
  const std::string& id,
  const soss::Message& msg) const
{
  json::Json output;
  output[JsonOpKey] = JsonOpPublishKey;
  output[JsonTopicNameKey] = topic_name;
  output[JsonMsgKey] = json::convert(msg);
  if (!id.empty())
    output[JsonIdKey] = id;

  return SerializerT::serialize(con_msg_mgr, output);
}

template<class SerializerT>
MessagePtrT RosbridgeV2Encoding<SerializerT>::encode_service_response_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& service_name,
  const std::string& /*service_type*/,
  const std::string& id,
  const soss::Message& response,
  const bool result) const
{
  json::Json output;
  output[JsonOpKey] = JsonOpServiceResponseKey;
  output[JsonServiceKey] = service_name;
  output[JsonValuesKey] = json::convert(response);
  output[JsonResultKey] = result;
  if (!id.empty())
    output[JsonIdKey] = id;

  return SerializerT::serialize(con_msg_mgr, output);
}

template<class SerializerT>
MessagePtrT RosbridgeV2Encoding<SerializerT>::encode_subscribe_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& topic_name,
  const std::string& message_type,
  const std::string& id,
  const YAML::Node& /*configuration*/) const
{
  // TODO(MXG): Consider parsing the `configuration` for details like
  // throttle_rate, queue_length, fragment_size, and compression
  json::Json output;
  output[JsonOpKey] = JsonOpSubscribeKey;
  output[JsonTopicNameKey] = topic_name;
  output[JsonTypeNameKey] = message_type;
  if (!id.empty())
    output[JsonIdKey] = id;

  return SerializerT::serialize(con_msg_mgr, output);
}

template<class SerializerT>
MessagePtrT RosbridgeV2Encoding<SerializerT>::encode_advertise_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& topic_name,
  const std::string& message_type,
  const std::string& id,
  const YAML::Node& /*configuration*/) const
{
  json::Json output;
  output[JsonOpKey] = JsonOpAdvertiseTopicKey;
  output[JsonTopicNameKey] = topic_name;
  output[JsonTypeNameKey] = message_type;
  if (!id.empty())
    output[JsonIdKey] = id;

  return SerializerT::serialize(con_msg_mgr, output);
}

template<class SerializerT>
MessagePtrT RosbridgeV2Encoding<SerializerT>::encode_call_service_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& service_name,
  const std::string& /*service_type*/,
  const soss::Message& service_request,
  const std::string& id,
  const YAML::Node& /*configuration*/) const
{
  // TODO(MXG): Consider parsing the `configuration` for details like
  // fragment_size and compression
  json::Json output;
  output[JsonOpKey] = JsonOpServiceRequestKey;
  output[JsonServiceKey] = service_name;
  output[JsonArgsKey] = json::convert(service_request);
  if (!id.empty())
    output[JsonIdKey] = id;

  return SerializerT::serialize(con_msg_mgr, output);
}

template<class SerializerT>
MessagePtrT RosbridgeV2Encoding<SerializerT>::encode_advertise_service_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& service_name,
  const std::string& service_type,
  const std::string& /*id*/,
  const YAML::Node& /*configuration*/) const
{
  json::Json output;
  output[JsonOpKey] = JsonOpAdvertiseServiceKey;
  output[JsonTypeNameKey] = service_type;
  output[JsonServiceKey] = service_name;

  return SerializerT::serialize(con_msg_mgr, output);
}

} // namespace websocket
} // namespace soss

#endif //SOSS_WEBSOCKET__ROSBRIDGEV2ENCODING__HPP
