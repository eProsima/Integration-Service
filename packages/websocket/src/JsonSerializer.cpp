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

#include "JsonSerializer.hpp"
#include <soss/json/conversion.hpp>
#include <iostream>

namespace soss {
namespace websocket {

std::vector<uint8_t> JsonSerializer::serialize(const nlohmann::json& msg) const {
  auto str = msg.dump();
  return std::vector<uint8_t>(str.begin(), str.end());
}

nlohmann::json JsonSerializer::deserialize(const std::vector<uint8_t>& data) const {
  return json::Json::parse(data.begin(), data.end());
}

} // namespace websocket
} // namespace soss