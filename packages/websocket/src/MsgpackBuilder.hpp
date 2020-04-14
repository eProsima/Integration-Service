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

#ifndef SOSS_WEBSOCKET__MSGPACKBUILDER_HPP
#define SOSS_WEBSOCKET__MSGPACKBUILDER_HPP

#include "MsgpackTypes.hpp"
#include <map>

namespace soss {
namespace websocket {

class MsgpackBuilder {
public:

  inline MessagePtrT serialize(ConMsgManagerPtrT& con_msg_mgr) const
  {
    return MsgpackSerializer::serialize(con_msg_mgr, _msg);
  }

  template<typename T>
  inline MsgpackBuilder& add(const std::string& key, const T& val)
  {
    _msg.emplace(key, msgpack::object{val, _zone});
    return *this;
  }

private:
  MsgpackMessage _msg;
  msgpack::zone _zone;
};

template<>
MsgpackBuilder& MsgpackBuilder::add<Message>(const std::string& key, const Message& val)
{
  throw std::runtime_error("not implemented");
}

} // namespace websocket
} // namespace soss

#endif //SOSS_WEBSOCKET__MSGPACKBUILDER_HPP
