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
#include "JsonSerialization.hpp"

#include <soss/json/conversion.hpp>

#include <unordered_set>

#include <iostream>

namespace soss {
namespace websocket {

//==============================================================================
// message fields
extern const std::string JsonIdKey;
extern const std::string JsonOpKey;
extern const std::string JsonTopicNameKey;
extern const std::string JsonTypeNameKey;
extern const std::string JsonMsgKey;
extern const std::string JsonServiceKey;
extern const std::string JsonArgsKey;
extern const std::string JsonValuesKey;
extern const std::string JsonResultKey;


// op codes
extern const std::string JsonOpAdvertiseTopicKey;
extern const std::string JsonOpUnadvertiseTopicKey;
extern const std::string JsonOpPublishKey;
extern const std::string JsonOpSubscribeKey;
extern const std::string JsonOpUnsubscribeKey;
extern const std::string JsonOpServiceRequestKey;
extern const std::string JsonOpAdvertiseServiceKey;
extern const std::string JsonOpUnadvertiseServiceKey;
extern const std::string JsonOpServiceResponseKey;

template<typename SerializationT = JsonSerialization>
class RosbridgeV2Encoding : public Encoding {
public:
  inline WsOpcode opcode() const override {
    return SerializationT::Serializer::opcode;
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

  [[noreturn]] void throw_missing_key(const typename SerializationT::Message& object, const std::string& key) const;

  std::string get_optional_string(const typename SerializationT::Message& object, const std::string& key) const;

  std::string get_required_string(const typename SerializationT::Message& object, const std::string& key) const;

  soss::Message get_required_msg(const typename SerializationT::Message& object, const std::string& key) const;
};

//==============================================================================
template<typename SerializationT>
void RosbridgeV2Encoding<SerializationT>::interpret_websocket_msg(
  const std::string& msg_str,
  Endpoint& endpoint,
  std::shared_ptr<void> connection_handle) const
{
  const auto msg = SerializationT::Serializer::deserialize(msg_str);

  const std::string op_str = [&msg, &msg_str]() -> std::string
  {
    try
    {
      return SerializationT::Accessor::template get<std::string>(msg, JsonOpKey);
    }
    catch (const std::out_of_range&)
    {
      throw std::runtime_error(
        "[soss::websocket::rosbridge_v2] Incoming message was missing "
        "the required op code: " + msg_str);
    }
  }();

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

//==============================================================================
template<typename SerializationT>
MessagePtrT RosbridgeV2Encoding<SerializationT>::encode_publication_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& topic_name,
  const std::string& /*topic_type*/,
  const std::string& id,
  const soss::Message& msg) const
{
  typename SerializationT::Builder builder;
  builder
    .add(JsonOpKey, JsonOpPublishKey)
    .add(JsonTopicNameKey, topic_name)
    .add(JsonMsgKey, msg);
  if (!id.empty())
    builder.add(JsonIdKey, id);

  return builder.serialize(con_msg_mgr);
}

//==============================================================================
template<typename SerializationT>
MessagePtrT RosbridgeV2Encoding<SerializationT>::encode_service_response_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& service_name,
  const std::string& /*service_type*/,
  const std::string& id,
  const soss::Message& response,
  const bool result) const
{
  typename SerializationT::Builder builder;
  builder
    .add(JsonOpKey, JsonOpServiceResponseKey)
    .add(JsonServiceKey, service_name)
    .add(JsonValuesKey, response)
    .add(JsonResultKey, result);
  if (!id.empty())
    builder.add(JsonIdKey, id);

  return builder.serialize(con_msg_mgr);
}

//==============================================================================
template<typename SerializationT>
MessagePtrT RosbridgeV2Encoding<SerializationT>::encode_subscribe_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& topic_name,
  const std::string& message_type,
  const std::string& id,
  const YAML::Node& /*configuration*/) const
{
  // TODO(MXG): Consider parsing the `configuration` for details like
  // throttle_rate, queue_length, fragment_size, and compression
  typename SerializationT::Builder builder;
  builder
    .add(JsonOpKey, JsonOpSubscribeKey)
    .add(JsonTopicNameKey, topic_name)
    .add(JsonTypeNameKey, message_type);
  if (!id.empty())
    builder.add(JsonIdKey, id);

  return builder.serialize(con_msg_mgr);
}

//==============================================================================
template<typename SerializationT>
MessagePtrT RosbridgeV2Encoding<SerializationT>::encode_advertise_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& topic_name,
  const std::string& message_type,
  const std::string& id,
  const YAML::Node& /*configuration*/) const
{
  typename SerializationT::Builder builder;
  builder
    .add(JsonOpKey, JsonOpAdvertiseTopicKey)
    .add(JsonTopicNameKey, topic_name)
    .add(JsonTypeNameKey, message_type);
  if (!id.empty())
    builder.add(JsonIdKey, id);

  return builder.serialize(con_msg_mgr);
}

//==============================================================================
template<typename SerializationT>
MessagePtrT RosbridgeV2Encoding<SerializationT>::encode_call_service_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& service_name,
  const std::string& /*service_type*/,
  const soss::Message& service_request,
  const std::string& id,
  const YAML::Node& /*configuration*/) const
{
  // TODO(MXG): Consider parsing the `configuration` for details like
  // fragment_size and compression
  typename SerializationT::Builder builder;
  builder
    .add(JsonOpKey, JsonOpServiceRequestKey)
    .add(JsonServiceKey, service_name)
    .add(JsonArgsKey, service_request);
  if (!id.empty())
    builder.add(JsonIdKey, id);

  return builder.serialize(con_msg_mgr);
}

//==============================================================================
template<typename SerializationT>
MessagePtrT RosbridgeV2Encoding<SerializationT>::encode_advertise_service_msg(
  ConMsgManagerPtrT& con_msg_mgr,
  const std::string& service_name,
  const std::string& service_type,
  const std::string& /*id*/,
  const YAML::Node& /*configuration*/) const
{
  typename SerializationT::Builder builder;
  builder
    .add(JsonOpKey, JsonOpAdvertiseServiceKey)
    .add(JsonTypeNameKey, service_type)
    .add(JsonServiceKey, service_name);

  return builder.serialize(con_msg_mgr);
}

//==============================================================================
template<typename SerializationT>
void RosbridgeV2Encoding<SerializationT>::throw_missing_key(
    const typename SerializationT::Message& object,
    const std::string& key) const
{
  const std::string op_code = get_optional_string(object, JsonOpKey);
  throw std::runtime_error(
        "[soss::websocket::rosbridge_v2] Incoming websocket message with op "
        "code [" + op_code + "] is missing the required field [" + key
        + "]:\n");
}

//==============================================================================
template<typename SerializationT>
std::string RosbridgeV2Encoding<SerializationT>::get_optional_string(
    const typename SerializationT::Message& object,
    const std::string& key) const
{
  try
  {
    return SerializationT::Accessor::template get<std::string>(object, key);
  }
  catch (const std::out_of_range&)
  {
    return "";
  }
  catch (const std::bad_cast&)
  {
    return "";
  }
}

//==============================================================================
template<typename SerializationT>
std::string RosbridgeV2Encoding<SerializationT>::get_required_string(
    const typename SerializationT::Message& object,
    const std::string& key) const
{
  try
  {
    return SerializationT::Accessor::template get<std::string>(object, key);
  }
  catch (const std::out_of_range&)
  {
    throw_missing_key(object, key);
  }
}

//==============================================================================
template<typename SerializationT>
Message RosbridgeV2Encoding<SerializationT>::get_required_msg(
    const typename SerializationT::Message& object,
    const std::string& key) const
{
  try
  {
    return SerializationT::Accessor::template get<Message>(object, key);
  }
  catch (const std::out_of_range&)
  {
    throw_missing_key(object, key);
  }
}

//==============================================================================

} // namespace websocket
} // namespace soss

#endif //SOSS_WEBSOCKET__ROSBRIDGEV2ENCODING__HPP
