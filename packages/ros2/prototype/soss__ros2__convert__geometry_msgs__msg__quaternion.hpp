#ifndef SOSS__ROS2__CONVERT__GEOMETRY_MSGS__MSG__QUATERNION_HPP
#define SOSS__ROS2__CONVERT__GEOMETRY_MSGS__MSG__QUATERNION_HPP

// Include the header for the generic message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete message type
#include <geometry_msgs/msg/quaternion.hpp>

// Include the headers for the soss message dependencies
// <None>

namespace soss {
namespace ros2 {
namespace convert__geometry_msgs__msg__quaternion {

using Ros2_Msg = geometry_msgs::msg::Quaternion;
const std::string g_msg_name = "geometry_msgs/Quaternion";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
  soss::Convert<Ros2_Msg::_x_type>::add_field(msg, "x");
  soss::Convert<Ros2_Msg::_y_type>::add_field(msg, "y");
  soss::Convert<Ros2_Msg::_z_type>::add_field(msg, "z");
  soss::Convert<Ros2_Msg::_w_type>::add_field(msg, "w");

  // NOTE(MXG): Investigate how to find the default values for message fields

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto it = from.data.begin();
  soss::Convert<Ros2_Msg::_w_type>::from_soss_field(it++, to.w);
  soss::Convert<Ros2_Msg::_x_type>::from_soss_field(it++, to.x);
  soss::Convert<Ros2_Msg::_y_type>::from_soss_field(it++, to.y);
  soss::Convert<Ros2_Msg::_z_type>::from_soss_field(it++, to.z);
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto it = to.data.begin();
  soss::Convert<Ros2_Msg::_w_type>::to_soss_field(from.w, it++);
  soss::Convert<Ros2_Msg::_x_type>::to_soss_field(from.x, it++);
  soss::Convert<Ros2_Msg::_y_type>::to_soss_field(from.y, it++);
  soss::Convert<Ros2_Msg::_z_type>::to_soss_field(from.z, it++);
}

} // namespace convert__geometry_msgs__msg__quaternion
} // namespace ros2

template<>
struct Convert<ros2::convert__geometry_msgs__msg__quaternion::Ros2_Msg>
    : MessageConvert<
     ros2::convert__geometry_msgs__msg__quaternion::Ros2_Msg,
    &ros2::convert__geometry_msgs__msg__quaternion::initialize,
    &ros2::convert__geometry_msgs__msg__quaternion::convert_to_ros2,
    &ros2::convert__geometry_msgs__msg__quaternion::convert_to_soss
    > { };

} // namespace soss

#endif // SOSS__ROS2__CONVERT__GEOMETRY_MSGS__MSG__QUATERNION_HPP
