#ifndef SOSS__ROS2__CONVERT__GEOMETRY_MSGS__MSG__POSE_HPP
#define SOSS__ROS2__CONVERT__GEOMETRY_MSGS__MSG__POSE_HPP

// Include the header for the generic message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete message type
#include <geometry_msgs/msg/pose.hpp>

// Include the headers for the soss message dependencies
#include "soss__ros2__convert__geometry_msgs__msg__point.hpp"
#include "soss__ros2__convert__geometry_msgs__msg__quaternion.hpp"

namespace soss {
namespace ros2 {
namespace convert__geometry_msgs__msg__pose {

using Ros2_Msg = geometry_msgs::msg::Pose;
const std::string g_msg_name = "geometry_msgs/Pose";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
  soss::Convert<Ros2_Msg::_position_type>::add_field(msg, "position");
  soss::Convert<Ros2_Msg::_orientation_type>::add_field(msg, "orientation");

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto it = from.data.begin();
  soss::Convert<Ros2_Msg::_orientation_type>::from_soss_field(it++, to.orientation);
  soss::Convert<Ros2_Msg::_position_type>::from_soss_field(it++, to.position);
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto it = to.data.begin();
  soss::Convert<Ros2_Msg::_orientation_type>::to_soss_field(from.orientation, it++);
  soss::Convert<Ros2_Msg::_position_type>::to_soss_field(from.position, it++);
}

} // namespace convert__geometry_msgs__msg__pose
} // namespace ros2

template<>
struct Convert<ros2::convert__geometry_msgs__msg__pose::Ros2_Msg>
    : MessageConvert<
     ros2::convert__geometry_msgs__msg__pose::Ros2_Msg,
    &ros2::convert__geometry_msgs__msg__pose::initialize,
    &ros2::convert__geometry_msgs__msg__pose::convert_to_ros2,
    &ros2::convert__geometry_msgs__msg__pose::convert_to_soss
    > { };

} // namespace soss

#endif // SOSS__ROS2__CONVERT__GEOMETRY_MSGS__MSG__POSE_HPP
