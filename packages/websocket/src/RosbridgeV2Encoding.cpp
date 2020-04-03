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

#include "RosbridgeV2Encoding.hpp"

namespace soss {
namespace websocket {

using json::Json;

//==============================================================================
// message fields
std::string JsonOpKey = "op";
std::string JsonIdKey = "id";
std::string JsonTopicNameKey = "topic";
std::string JsonTypeNameKey = "type";
std::string JsonMsgKey = "msg";
std::string JsonServiceKey = "service";
std::string JsonArgsKey = "args";
std::string JsonValuesKey = "values";
std::string JsonResultKey = "result";


// op codes
std::string JsonOpAdvertiseTopicKey = "advertise";
std::string JsonOpUnadvertiseTopicKey = "unadvertise";
std::string JsonOpPublishKey = "publish";
std::string JsonOpSubscribeKey = "subscribe";
std::string JsonOpUnsubscribeKey = "unsubscribe";
std::string JsonOpServiceRequestKey = "call_service";
std::string JsonOpAdvertiseServiceKey = "advertise_service";
std::string JsonOpUnadvertiseServiceKey = "unadvertise_service";
std::string JsonOpServiceResponseKey = "service_response";

//==============================================================================
void throw_missing_key(
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
std::string get_optional_string(
    const Json& object,
    const std::string& key)
{
  const auto it = object.find(key);
  return it == object.end()? "" : json::to_string(it.value());
}

//==============================================================================
std::string get_required_string(
    const Json& object,
    const std::string& key)
{
  const auto it = object.find(key);
  if(it == object.end())
    throw_missing_key(object, key);

  return json::to_string(it.value());
}

//==============================================================================
Message get_required_msg(
    const Json& object,
    const std::string& key)
{
  const auto it = object.find(key);
  if(it == object.end())
    throw_missing_key(object, key);

  return json::convert("", it.value());
}

//==============================================================================

} // namespace websocket
} // namespace soss
