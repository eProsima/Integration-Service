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

#include "MsgpackAdaptor.hpp"

namespace soss {
namespace websocket {

void MsgpackConverter::convert(const soss::Field& from, msgpack::object::with_zone& to)
{
  static _ConvertFuncMap _convert_map = _make_convert_map();
  _convert_map.at(from.type())(from, to);
}

MsgpackConverter::_ConvertFuncMap MsgpackConverter::_make_convert_map()
{
  _ConvertFuncMap map{};
  _add_native_conversion<int8_t>(map);
  _add_native_conversion<int16_t>(map);
  _add_native_conversion<int32_t>(map);
  _add_native_conversion<int64_t>(map);
  _add_native_conversion<uint8_t>(map);
  _add_native_conversion<uint16_t>(map);
  _add_native_conversion<uint32_t>(map);
  _add_native_conversion<uint64_t>(map);
  _add_native_conversion<bool>(map);
  _add_native_conversion<std::string>(map);
  return map;
}

} // namespace websocket
} // namespace soss
