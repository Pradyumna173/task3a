// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from usb_servo:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef USB_SERVO__MSG__DETAIL__NUM__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define USB_SERVO__MSG__DETAIL__NUM__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "usb_servo/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "usb_servo/msg/detail/num__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace usb_servo
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_usb_servo
cdr_serialize(
  const usb_servo::msg::Num & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_usb_servo
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  usb_servo::msg::Num & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_usb_servo
get_serialized_size(
  const usb_servo::msg::Num & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_usb_servo
max_serialized_size_Num(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace usb_servo

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_usb_servo
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, usb_servo, msg, Num)();

#ifdef __cplusplus
}
#endif

#endif  // USB_SERVO__MSG__DETAIL__NUM__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
