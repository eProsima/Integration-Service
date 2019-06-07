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

#ifndef SOSS__ROS2__CONVERT__NAV_MSGS__MSG__PATH_HPP
#define SOSS__ROS2__CONVERT__NAV_MSGS__MSG__PATH_HPP

// Include the header for the generic message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete message type
#include <nav_msgs/msg/path.hpp>

// Include the headers for the soss message dependencies
#include "soss__ros2__convert__std_msgs__msg__header.hpp"
#include "soss__ros2__convert__geometry_msgs__msg__pose_stamped.hpp"

namespace soss {
namespace ros2 {
namespace convert__nav_msgs__msg__path {

using Ros2_Msg = nav_msgs::msg::Path;
const std::string g_msg_name = "nav_msgs/Path";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
  soss::Convert<Ros2_Msg::_header_type>::add_field(msg, "header");
  soss::Convert<Ros2_Msg::_poses_type>::add_field(msg, "poses");

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto it = from.data.begin();
  soss::Convert<Ros2_Msg::_header_type>::from_soss_field(it++, to.header);
  soss::Convert<Ros2_Msg::_poses_type>::from_soss_field(it++, to.poses);
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto it = to.data.begin();
  soss::Convert<Ros2_Msg::_header_type>::to_soss_field(from.header, it++);
  soss::Convert<Ros2_Msg::_poses_type>::to_soss_field(from.poses, it++);
}

} // namespace convert__nav_msgs__msg__path
} // namespace ros2

template<>
struct Convert<ros2::convert__nav_msgs__msg__path::Ros2_Msg>
    : MessageConvert<
     ros2::convert__nav_msgs__msg__path::Ros2_Msg,
    &ros2::convert__nav_msgs__msg__path::initialize,
    &ros2::convert__nav_msgs__msg__path::convert_to_ros2,
    &ros2::convert__nav_msgs__msg__path::convert_to_soss
    > { };

} // namespace soss

#endif // SOSS__ROS2__CONVERT__NAV_MSGS__MSG__PATH_HPP
