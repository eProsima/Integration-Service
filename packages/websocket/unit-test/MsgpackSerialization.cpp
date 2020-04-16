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

#include <MsgpackSerialization.hpp>
#include <soss/Message.hpp>
#include <soss/utilities.hpp>
#include <catch2/catch.hpp>
#include <limits>

namespace soss {
namespace websocket {

TEST_CASE("serialize soss message smoke test", "[msgpack]")
{
  Message soss_msg;

  // signed int
  soss_msg.data["int8"] = Convert<int8_t>::make_soss_field(std::numeric_limits<int8_t>::max());
  soss_msg.data["int16"] = Convert<int16_t>::make_soss_field(std::numeric_limits<int16_t>::max());
  soss_msg.data["int32"] = Convert<int32_t>::make_soss_field(std::numeric_limits<int32_t>::max());
  soss_msg.data["int64"] = Convert<int64_t>::make_soss_field(std::numeric_limits<int64_t>::max());

  // unsigned int
  soss_msg.data["uint8"] = Convert<uint8_t>::make_soss_field(std::numeric_limits<uint8_t>::max());
  soss_msg.data["uint16"] = Convert<uint16_t>::make_soss_field(std::numeric_limits<uint16_t>::max());
  soss_msg.data["uint32"] = Convert<uint32_t>::make_soss_field(std::numeric_limits<uint32_t>::max());
  soss_msg.data["uint64"] = Convert<uint64_t>::make_soss_field(std::numeric_limits<uint64_t>::max());

  soss_msg.data["string"] = Convert<std::string>::make_soss_field("string");

  // signed arrays
  soss_msg.data["int8_array"] = Convert<std::vector<int8_t>>::make_soss_field(
    std::initializer_list<Convert<int8_t>::soss_type>{
      std::numeric_limits<int8_t>::min(),
      std::numeric_limits<int8_t>::max()
    }
  );
  soss_msg.data["int16_array"] = Convert<std::vector<int16_t>>::make_soss_field(
    std::initializer_list<Convert<int16_t>::soss_type>{
      std::numeric_limits<int16_t>::min(),
      std::numeric_limits<int16_t>::max()
    }
  );
  soss_msg.data["int32_array"] = Convert<std::vector<int32_t>>::make_soss_field(
    std::initializer_list<Convert<int32_t>::soss_type>{
      std::numeric_limits<int32_t>::min(),
      std::numeric_limits<int32_t>::max()
    }
  );
  soss_msg.data["int64_array"] = Convert<std::vector<int64_t>>::make_soss_field(
    std::initializer_list<Convert<int64_t>::soss_type>{
      std::numeric_limits<int64_t>::min(),
      std::numeric_limits<int64_t>::max()
    }
  );

  // unsigned arrays
  soss_msg.data["uint8_array"] = Convert<std::vector<uint8_t>>::make_soss_field(
    std::initializer_list<Convert<uint8_t>::soss_type>{
      std::numeric_limits<uint8_t>::min(),
      std::numeric_limits<uint8_t>::max()
    }
  );
  soss_msg.data["uint16_array"] = Convert<std::vector<uint16_t>>::make_soss_field(
    std::initializer_list<Convert<uint16_t>::soss_type>{
      std::numeric_limits<uint16_t>::min(),
      std::numeric_limits<uint16_t>::max()
    }
  );
  soss_msg.data["uint32_array"] = Convert<std::vector<uint32_t>>::make_soss_field(
    std::initializer_list<Convert<uint32_t>::soss_type>{
      std::numeric_limits<uint32_t>::min(),
      std::numeric_limits<uint32_t>::max()
    }
  );
  soss_msg.data["uint64_array"] = Convert<std::vector<uint64_t>>::make_soss_field(
    std::initializer_list<Convert<uint64_t>::soss_type>{
      std::numeric_limits<uint64_t>::min(),
      std::numeric_limits<uint64_t>::max()
    }
  );

  soss_msg.data["string_array"] = Convert<std::vector<std::string>>::make_soss_field(
    std::initializer_list<Convert<std::string>::soss_type>{
      "hello",
      "world"
    }
  );

  MsgpackSerialization::Builder builder;
  builder.add("soss", soss_msg);

  auto con_msg_mgr = std::make_shared<ConMsgManagerT>();
  builder.serialize(con_msg_mgr);
}

TEMPLATE_TEST_CASE(
  "message with primitive int",
  "[msgpack]",
  int8_t,
  int16_t,
  int32_t,
  int64_t,
  uint8_t,
  uint16_t,
  uint32_t,
  uint64_t,
  bool
)
{
  Message soss_msg;
  auto data = std::numeric_limits<TestType>::max();
  soss_msg.data["data"] = Convert<TestType>::make_soss_field(data);

  MsgpackSerialization::Builder builder;
  builder.add("soss", soss_msg);

  auto con_msg_mgr = std::make_shared<ConMsgManagerT>();
  auto ws_msg = builder.serialize(con_msg_mgr);

  auto payload = std::string{ws_msg->get_payload().data(), ws_msg->get_payload().size()};
  auto lp_msgpack_msg = MsgpackSerialization::Serializer::deserialize(payload);
  auto lp_soss_msg = MsgpackSerialization::Accessor::get<Message>(lp_msgpack_msg, "soss");

  auto lp_data_ptr = lp_soss_msg.data["data"].cast<typename Convert<TestType>::soss_type>();
  REQUIRE(lp_data_ptr);
  CHECK(*lp_data_ptr == data);
}

} // namespace websocket
} // namespace soss
