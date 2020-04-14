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

} // namespace websocket
} // namespace soss
