#ifndef SOSS__ROS2__CONVERT__STD_MSGS__MSG__HEADER_HPP
#define SOSS__ROS2__CONVERT__STD_MSGS__MSG__HEADER_HPP

// Include the header for the generic message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete message type
#include <std_msgs/msg/header.hpp>

// Include the headers for the soss message dependencies
#include "soss__ros2__convert__builtin_interfaces__msg__time.hpp"

namespace soss {
namespace ros2 {
namespace convert__std_msgs__msg__header {

using Ros2_Msg = std_msgs::msg::Header;
const std::string g_msg_name = "std_msgs/Header";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
  soss::Convert<Ros2_Msg::_frame_id_type>::add_field(msg, "frame_id");
  soss::Convert<Ros2_Msg::_stamp_type>::add_field(msg, "stamp");

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto it = from.data.begin();
  soss::Convert<Ros2_Msg::_frame_id_type>::from_soss_field(it++, to.frame_id);
  soss::Convert<Ros2_Msg::_stamp_type>::from_soss_field(it++, to.stamp);
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto it = to.data.begin();
  soss::Convert<Ros2_Msg::_frame_id_type>::to_soss_field(from.frame_id, it++);
  soss::Convert<Ros2_Msg::_stamp_type>::to_soss_field(from.stamp, it++);
}

} // namespace convert__std_msgs__msg__header
} // namespace ros2

template<>
struct Convert<ros2::convert__std_msgs__msg__header::Ros2_Msg>
    : MessageConvert<
     ros2::convert__std_msgs__msg__header::Ros2_Msg,
    &ros2::convert__std_msgs__msg__header::initialize,
    &ros2::convert__std_msgs__msg__header::convert_to_ros2,
    &ros2::convert__std_msgs__msg__header::convert_to_soss
    > { };

} // namespace soss

#endif // SOSS__ROS2__CONVERT__STD_MSGS__MSG__HEADER_HPP
