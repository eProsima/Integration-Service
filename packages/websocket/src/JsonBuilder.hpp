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

#ifndef SOSS_WEBSOCKET__JSONBUILDER_HPP
#define SOSS_WEBSOCKET__JSONBUILDER_HPP

#include "JsonSerializer.hpp"
#include <soss/Message.hpp>
#include <soss/json/conversion.hpp>
#include <string>

namespace soss {
namespace websocket {

template<typename SerializerT = JsonSerializer>
class JsonBuilder {
public:

  using Serializer = SerializerT;

  inline MessagePtrT serialize(ConMsgManagerPtrT& con_msg_mgr) const {
    return Serializer::serialize(con_msg_mgr, _json_msg);
  }

  template<typename T>
  inline JsonBuilder& add(const std::string& key, const T& val) {
    _json_msg[key] = val;
    return *this;
  }

  inline JsonBuilder& add(const std::string& key, const Message& msg) {
    _json_msg[key] = json::convert(msg);
    return *this;
  }

private:
  nlohmann::json _json_msg;
};

} // namespace websocket
} // namespace soss

#endif //SOSS_WEBSOCKET__JSONBUILDER_HPP
