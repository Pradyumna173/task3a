// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from usb_servo:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef USB_SERVO__MSG__DETAIL__NUM__BUILDER_HPP_
#define USB_SERVO__MSG__DETAIL__NUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "usb_servo/msg/detail/num__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace usb_servo
{

namespace msg
{

namespace builder
{

class Init_Num_num
{
public:
  Init_Num_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::usb_servo::msg::Num num(::usb_servo::msg::Num::_num_type arg)
  {
    msg_.num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::usb_servo::msg::Num msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::usb_servo::msg::Num>()
{
  return usb_servo::msg::builder::Init_Num_num();
}

}  // namespace usb_servo

#endif  // USB_SERVO__MSG__DETAIL__NUM__BUILDER_HPP_
