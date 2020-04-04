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

#include "BsonSerializer.hpp"

namespace soss {
namespace websocket {

SharedBuffer BsonSerializer::serialize(const nlohmann::json& msg) const {
  auto payload = nlohmann::json::to_bson(msg);
  return SharedBuffer{
    payload.data(),
    payload.size(),
    websocketpp::frame::opcode::value::BINARY,
    std::make_shared<decltype(payload)>(std::move(payload))
  };
}

nlohmann::json BsonSerializer::deserialize(const std::vector<uint8_t>& data) const {
  return nlohmann::json::from_bson(data);
}

} // namespace websocket
} // namespace soss