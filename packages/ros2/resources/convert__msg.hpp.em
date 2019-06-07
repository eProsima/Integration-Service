// generated from soss/cpp/ros2/resource/soss__ros2__message.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating soss/rosidl/ros2/<package>/include/soss/rosidl/ros2/<package>/msg/convert__msg__<msg>.hpp files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg' or 'srv'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################

@{
camelcase_msg_type = spec.base_type.type
underscore_msg_type = get_header_filename_from_msg_name(camelcase_msg_type)

cpp_msg_type = '{}::msg::{}'.format(
      spec.base_type.pkg_name, camelcase_msg_type)

msg_type_string = '{}/{}'.format(
      spec.base_type.pkg_name, camelcase_msg_type)

header_guard_parts = [
    'SOSS__ROSIDL__ROS2', spec.base_type.pkg_name, 'MSG__CONVERT__MSG',
    underscore_msg_type + '_HPP']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts])

namespace_parts = [
    'convert', spec.base_type.pkg_name, 'msg', underscore_msg_type]
namespace_variable = '__'.join(namespace_parts)

ros2_msg_dependency = '{}/msg/{}.hpp'.format(
      spec.base_type.pkg_name, underscore_msg_type)

conversion_dependencies = {}
for field in spec.fields:
    if field.type.is_primitive_type():
        continue

    key = 'soss/rosidl/ros2/{}/msg/convert__msg__{}.hpp'.format(
        field.type.pkg_name, field.type.type)
    if key not in conversion_dependencies:
        conversion_dependencies[key] = set([])
    conversion_dependencies[key].add(field.name)

alphabetical_fields = sorted(spec.fields, key=lambda x: x.name)
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

// Include the header for the generic soss message type
#include <soss/ros2/utilities.hpp>

// Include the header for the concrete ros2 message type
#include <@(ros2_msg_dependency)>

// Include the headers for the soss message conversion dependencies
@[if conversion_dependencies.keys()]@
@[    for key in sorted(conversion_dependencies.keys())]@
#include <@(key)> // @(', '.join(conversion_dependencies[key]))
@[    end for]@
@[else]@
// <none>
@[end if]@

namespace soss {
namespace ros2 {
namespace @(namespace_variable) {

using Ros2_Msg = @(cpp_msg_type);
const std::string g_msg_name = "@(msg_type_string)";

//==============================================================================
inline soss::Message initialize()
{
  soss::Message msg;
  msg.type = g_msg_name;
@[for field in alphabetical_fields]@
  soss::Convert<Ros2_Msg::_@(field.name)_type>::add_field(msg, "@(field.name)");
@[end for]@

  return msg;
}

//==============================================================================
inline void convert_to_ros2(const soss::Message& from, Ros2_Msg& to)
{
  auto from_field = from.data.begin();
@[for field in alphabetical_fields]@
  soss::Convert<Ros2_Msg::_@(field.name)_type>::from_soss_field(from_field++, to.@(field.name));
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
  (void)from_field;
}

//==============================================================================
inline void convert_to_soss(const Ros2_Msg& from, soss::Message& to)
{
  auto to_field = to.data.begin();
@[for field in alphabetical_fields]@
  soss::Convert<Ros2_Msg::_@(field.name)_type>::to_soss_field(from.@(field.name), to_field++);
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
  (void)to_field;
}

} // namespace @(namespace_variable)
} // namespace ros2

template<>
struct Convert<ros2::@(namespace_variable)::Ros2_Msg>
    : MessageConvert<
     ros2::@(namespace_variable)::Ros2_Msg,
    &ros2::@(namespace_variable)::initialize,
    &ros2::@(namespace_variable)::convert_to_ros2,
    &ros2::@(namespace_variable)::convert_to_soss
    > { };

} // namespace soss

#endif // @(header_guard_variable)
