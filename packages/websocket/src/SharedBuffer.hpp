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

#ifndef SOSS_WEBSOCKET__BUFFERVIEW_HPP
#define SOSS_WEBSOCKET__BUFFERVIEW_HPP

#include <websocketpp/frame.hpp>
#include <memory>

namespace soss {
namespace websocket {

/**
 * A simple <i>data owning</i> buffer.
 */
struct SharedBuffer {
  const void* data;
  const size_t len;
  const websocketpp::frame::opcode::value opcode;
  std::shared_ptr<const void> ref;
};

} // namespace websocket
} // namespace soss

#endif //SOSS_WEBSOCKET__BUFFERVIEW_HPP
