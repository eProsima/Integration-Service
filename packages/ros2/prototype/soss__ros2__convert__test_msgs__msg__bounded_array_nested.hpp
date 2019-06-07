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

#ifndef SOSS__ROS2__CONVERT__TEST_MSGS__MSG__BOUNDED_ARRAY_NESTED_HPP
#define SOSS__ROS2__CONVERT__TEST_MSGS__MSG__BOUNDED_ARRAY_NESTED_HPP

// Include the header for the generic message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete message type
#include <test_msgs/msg/bounded_array_nested.hpp>

// Include the headers for the soss message dependencies
#include "soss__ros2__convert__test_msgs__msg__primitives.hpp"

#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace soss {
namespace ros2 {
namespace convert__test_msgs__msg__bounded_array_nested {

using Ros2_Msg = test_msgs::msg::BoundedArrayNested;
const std::string g_msg_name = "test_msgs/BoundedArrayNested";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
  soss::Convert<Ros2_Msg::_primitive_values_type>::add_field(msg, "primitive_values");

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto from_field = from.data.begin();
  soss::Convert<Ros2_Msg::_primitive_values_type>::from_soss_field(from_field++, to.primitive_values);
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto to_field = to.data.begin();
  soss::Convert<Ros2_Msg::_primitive_values_type>::to_soss_field(from.primitive_values, to_field++);
}

} // namespace convert__test_msgs__msg__bounded_array_nested
} // namespace ros2

template<>
struct Convert<ros2::convert__test_msgs__msg__bounded_array_nested::Ros2_Msg>
    : MessageConvert<
     ros2::convert__test_msgs__msg__bounded_array_nested::Ros2_Msg,
    &ros2::convert__test_msgs__msg__bounded_array_nested::initialize,
    &ros2::convert__test_msgs__msg__bounded_array_nested::convert_to_ros2,
    &ros2::convert__test_msgs__msg__bounded_array_nested::convert_to_soss
    > { };

} // namespace soss

#endif // SOSS__ROS2__CONVERT__TEST_MSGS__MSG__BOUNDED_ARRAY_NESTED_HPP
