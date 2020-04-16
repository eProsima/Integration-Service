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

#ifndef SOSS_WEBSOCKET__MSGPACKADAPTOR_HPP
#define SOSS_WEBSOCKET__MSGPACKADAPTOR_HPP

#include "MsgpackTypes.hpp"
#include <soss/Message.hpp>
#include <msgpack.hpp>
#include <soss/utilities.hpp>

namespace soss {
namespace websocket {

class MsgpackConverter {
public:

  static void convert(const Field& from, msgpack::object::with_zone& to);

private:

  using _ConvertFunc = std::function<void(const Field& from, msgpack::object::with_zone& to)>;
  using _ConvertFuncMap = std::unordered_map<
    std::string,
    _ConvertFunc
  >;

  static _ConvertFuncMap _make_convert_map();

  template<typename T>
  inline static void _convert_direct(const Field& from, msgpack::object::with_zone& to)
  {
    msgpack::adaptor::object_with_zone<T>{}(to, *from.cast<T>());
  }

  template<typename T>
  static void _add_native_conversion(_ConvertFuncMap& map)
  {
    map.emplace(typeid(T).name(), _convert_direct<T>);
    map.emplace(typeid(std::vector<T>).name(), _convert_direct<std::vector<T>>);
  }
};

} // namespace websocket
} // namespace soss


// =============================================================================


namespace msgpack {

template<>
struct adaptor::object_with_zone<soss::Field>
{
  inline void operator()(msgpack::object::with_zone& o, const soss::Field& f) const
  {
    soss::websocket::MsgpackConverter::convert(f, o);
  }
};

template<>
struct adaptor::object_with_zone<soss::Message>
{
  inline void operator()(msgpack::object::with_zone& obj, const soss::Message& m) const
  {
    using ObjectMap = std::map<std::string, msgpack::object::with_zone>;
    ObjectMap field_objs;
    for (const auto& kv : m.data)
    {
      auto new_pair = field_objs.emplace(kv.first, msgpack::object::with_zone{obj.zone}).first;
      adaptor::object_with_zone<soss::Field>{}(new_pair->second, kv.second);
    }
    adaptor::object_with_zone<ObjectMap>{}(obj, field_objs);
  }
};

//template<>
//struct adaptor::pack<Foo> {
//  template<typename Stream>
//  packer<Stream>& operator()(packer<Stream>& p, const Foo& foo) const {
//    std::cout << "pack" << std::endl;
//    return adaptor::pack<std::string>()(p, foo.foo);
//  }
//};
//

template<>
struct adaptor::as<soss::Field>
{
  soss::Field operator()(const msgpack::object& o) const
  {
    soss::Field f{};
    o.convert(f);
    return f;
  }
};

template<>
struct adaptor::convert<soss::Field>
{
  const msgpack::object& operator()(const msgpack::object& o, soss::Field& f) const
  {
    switch (o.type)
    {
      case type::object_type::POSITIVE_INTEGER:
        f.set(soss::Convert<uint64_t>::make_soss(o.via.u64));
        return o;
      case type::object_type::NEGATIVE_INTEGER:
        f.set(soss::Convert<int64_t>::make_soss(o.via.i64));
        return o;
      case type::object_type::FLOAT32:
        f.set(soss::Convert<float>::make_soss(o.via.f64));
        return o;
      case type::object_type::FLOAT64:
        f.set(soss::Convert<double>::make_soss(o.via.f64));
        return o;
      case type::object_type::BOOLEAN:
        f.set(soss::Convert<bool>::make_soss(o.via.boolean));
        return o;
      case type::object_type::STR:
        f.set(soss::Convert<std::string>::make_soss(o.via.str.ptr, o.via.str.size));
        return o;
      case type::object_type::BIN:
      {
        using SossType = soss::Convert<uint8_t>::soss_type;

        std::vector<SossType> byte_array{o.via.bin.size};
        for (uint32_t i = 0; i < o.via.bin.size; i++)
          byte_array.emplace_back(o.via.bin.ptr[i]);
        f.set(byte_array);
        return o;
      }
      case type::object_type::ARRAY:
      {
        std::vector<soss::Field> inner_fields{o.via.array.size};
        for (uint32_t i = 0; i < o.via.array.size; i++)
          adaptor::convert<soss::Field>{}(o.via.array.ptr[i], inner_fields[i]);
        f.set(inner_fields);
        return o;
      }
      case type::object_type::MAP:
      {
        soss::Message soss_msg{};
        for (uint32_t i = 0; i < o.via.map.size; i++) {
          const auto map_ptr = o.via.map.ptr[i];
          soss_msg.data.emplace(
            map_ptr.key.as<std::string>(),
            adaptor::as<soss::Field>{}(map_ptr.val)
          );
        }
        f.set(soss_msg);
        return o;
      }
      default:
        throw msgpack::type_error{};
    }
  }
};

template<>
struct adaptor::convert<soss::Message>
{
  const msgpack::object& operator()(const msgpack::object& o, soss::Message& m) const
  {
    auto msg = o.as<soss::websocket::MsgpackMessage>();
    for (const auto& kv : msg)
    {
      m.data[kv.first] = kv.second.as<soss::Field>();
    }
    return o;
  }
};

} // namespace msgpack

#endif //SOSS_WEBSOCKET__MSGPACKADAPTOR_HPP
