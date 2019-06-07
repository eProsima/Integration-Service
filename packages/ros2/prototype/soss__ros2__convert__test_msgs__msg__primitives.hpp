/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef SOSS__ROS2__CONVERT__TEST_MSGS__MSG__PRIMITIVES_HPP
#define SOSS__ROS2__CONVERT__TEST_MSGS__MSG__PRIMITIVES_HPP

// Include the header for the generic message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete message type
#include <test_msgs/msg/primitives.hpp>

// Include the headers for the soss message dependencies
// <none>

namespace soss {
namespace ros2 {
namespace convert__test_msgs__msg__primitives {

using Ros2_Msg = test_msgs::msg::Primitives;
const std::string g_msg_name = "test_msgs/Primitives";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
  soss::Convert<Ros2_Msg::_bool_value_type>::add_field(msg, "bool_value");
  soss::Convert<Ros2_Msg::_byte_value_type>::add_field(msg, "byte_value");
  soss::Convert<Ros2_Msg::_char_value_type>::add_field(msg, "char_value");
  soss::Convert<Ros2_Msg::_float32_value_type>::add_field(msg, "float32_value");
  soss::Convert<Ros2_Msg::_float64_value_type>::add_field(msg, "float64_value");
  soss::Convert<Ros2_Msg::_int16_value_type>::add_field(msg, "int16_value");
  soss::Convert<Ros2_Msg::_int32_value_type>::add_field(msg, "int32_value");
  soss::Convert<Ros2_Msg::_int64_value_type>::add_field(msg, "int64_value");
  soss::Convert<Ros2_Msg::_int8_value_type>::add_field(msg, "int8_value");
  soss::Convert<Ros2_Msg::_string_value_type>::add_field(msg, "string_value");
  soss::Convert<Ros2_Msg::_uint16_value_type>::add_field(msg, "uint16_value");
  soss::Convert<Ros2_Msg::_uint32_value_type>::add_field(msg, "uint32_value");
  soss::Convert<Ros2_Msg::_uint64_value_type>::add_field(msg, "uint64_value");
  soss::Convert<Ros2_Msg::_uint8_value_type>::add_field(msg, "uint8_value");

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto from_field = from.data.begin();
  soss::Convert<Ros2_Msg::_bool_value_type>::from_soss_field(from_field++, to.bool_value);
  soss::Convert<Ros2_Msg::_byte_value_type>::from_soss_field(from_field++, to.byte_value);
  soss::Convert<Ros2_Msg::_char_value_type>::from_soss_field(from_field++, to.char_value);
  soss::Convert<Ros2_Msg::_float32_value_type>::from_soss_field(from_field++, to.float32_value);
  soss::Convert<Ros2_Msg::_float64_value_type>::from_soss_field(from_field++, to.float64_value);
  soss::Convert<Ros2_Msg::_int16_value_type>::from_soss_field(from_field++, to.int16_value);
  soss::Convert<Ros2_Msg::_int32_value_type>::from_soss_field(from_field++, to.int32_value);
  soss::Convert<Ros2_Msg::_int64_value_type>::from_soss_field(from_field++, to.int64_value);
  soss::Convert<Ros2_Msg::_int8_value_type>::from_soss_field(from_field++, to.int8_value);
  soss::Convert<Ros2_Msg::_string_value_type>::from_soss_field(from_field++, to.string_value);
  soss::Convert<Ros2_Msg::_uint16_value_type>::from_soss_field(from_field++, to.uint16_value);
  soss::Convert<Ros2_Msg::_uint32_value_type>::from_soss_field(from_field++, to.uint32_value);
  soss::Convert<Ros2_Msg::_uint64_value_type>::from_soss_field(from_field++, to.uint64_value);
  soss::Convert<Ros2_Msg::_uint8_value_type>::from_soss_field(from_field++, to.uint8_value);
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto to_field = to.data.begin();
  soss::Convert<Ros2_Msg::_bool_value_type>::to_soss_field(from.bool_value, to_field++);
  soss::Convert<Ros2_Msg::_byte_value_type>::to_soss_field(from.byte_value, to_field++);
  soss::Convert<Ros2_Msg::_char_value_type>::to_soss_field(from.char_value, to_field++);
  soss::Convert<Ros2_Msg::_float32_value_type>::to_soss_field(from.float32_value, to_field++);
  soss::Convert<Ros2_Msg::_float64_value_type>::to_soss_field(from.float64_value, to_field++);
  soss::Convert<Ros2_Msg::_int16_value_type>::to_soss_field(from.int16_value, to_field++);
  soss::Convert<Ros2_Msg::_int32_value_type>::to_soss_field(from.int32_value, to_field++);
  soss::Convert<Ros2_Msg::_int64_value_type>::to_soss_field(from.int64_value, to_field++);
  soss::Convert<Ros2_Msg::_int8_value_type>::to_soss_field(from.int8_value, to_field++);
  soss::Convert<Ros2_Msg::_string_value_type>::to_soss_field(from.string_value, to_field++);
  soss::Convert<Ros2_Msg::_uint16_value_type>::to_soss_field(from.uint16_value, to_field++);
  soss::Convert<Ros2_Msg::_uint32_value_type>::to_soss_field(from.uint32_value, to_field++);
  soss::Convert<Ros2_Msg::_uint64_value_type>::to_soss_field(from.uint64_value, to_field++);
  soss::Convert<Ros2_Msg::_uint8_value_type>::to_soss_field(from.uint8_value, to_field++);
}

} // namespace convert__test_msgs__msg__primitives
} // namespace ros2

template<>
struct Convert<ros2::convert__test_msgs__msg__primitives::Ros2_Msg>
    : MessageConvert<
     ros2::convert__test_msgs__msg__primitives::Ros2_Msg,
    &ros2::convert__test_msgs__msg__primitives::initialize,
    &ros2::convert__test_msgs__msg__primitives::convert_to_ros2,
    &ros2::convert__test_msgs__msg__primitives::convert_to_soss
    > { };

} // namespace soss

#endif // SOSS__ROS2__CONVERT__TEST_MSGS__MSG__PRIMITIVES_HPP
