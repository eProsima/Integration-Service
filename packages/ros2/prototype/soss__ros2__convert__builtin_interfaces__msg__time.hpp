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

#ifndef SOSS__ROS2__CONVERT__BUILTIN_INTERFACES__MSG__TIME_HPP
#define SOSS__ROS2__CONVERT__BUILTIN_INTERFACES__MSG__TIME_HPP

// Include the header for the generic message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete message type
#include <builtin_interfaces/msg/time.hpp>

// Include the headers for the soss message dependencies
// <None>

namespace soss {
namespace ros2 {
namespace convert__builtin_interfaces__msg__time {

using Ros2_Msg = builtin_interfaces::msg::Time;
const std::string g_msg_name = "builtin_interfaces/Time";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
  soss::Convert<Ros2_Msg::_nanosec_type>::add_field(msg, "nanosec");
  soss::Convert<Ros2_Msg::_sec_type>::add_field(msg, "sec");

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto it = from.data.begin();
  soss::Convert<Ros2_Msg::_nanosec_type>::from_soss_field(it++, to.nanosec);
  soss::Convert<Ros2_Msg::_sec_type>::from_soss_field(it++, to.sec);
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto it = to.data.begin();
  soss::Convert<Ros2_Msg::_nanosec_type>::to_soss_field(from.nanosec, it++);
  soss::Convert<Ros2_Msg::_sec_type>::to_soss_field(from.sec, it++);
}

} // namespace convert__builtin_interfaces__msg__time
} // namespace ros2

template<>
struct Convert<ros2::convert__builtin_interfaces__msg__time::Ros2_Msg>
    : MessageConvert<
     ros2::convert__builtin_interfaces__msg__time::Ros2_Msg,
    &ros2::convert__builtin_interfaces__msg__time::initialize,
    &ros2::convert__builtin_interfaces__msg__time::convert_to_ros2,
    &ros2::convert__builtin_interfaces__msg__time::convert_to_soss
    > { };

} // namespace soss

#endif // SOSS__ROS2__CONVERT__BUILTIN_INTERFACES__MSG__TIME_HPP
