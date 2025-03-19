// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice

#ifndef USB_SERVO__SRV__DETAIL__SERVO_SW__BUILDER_HPP_
#define USB_SERVO__SRV__DETAIL__SERVO_SW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "usb_servo/srv/detail/servo_sw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace usb_servo
{

namespace srv
{

namespace builder
{

class Init_ServoSw_Request_servostate
{
public:
  Init_ServoSw_Request_servostate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::usb_servo::srv::ServoSw_Request servostate(::usb_servo::srv::ServoSw_Request::_servostate_type arg)
  {
    msg_.servostate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::usb_servo::srv::ServoSw_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::usb_servo::srv::ServoSw_Request>()
{
  return usb_servo::srv::builder::Init_ServoSw_Request_servostate();
}

}  // namespace usb_servo


namespace usb_servo
{

namespace srv
{

namespace builder
{

class Init_ServoSw_Response_num
{
public:
  Init_ServoSw_Response_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::usb_servo::srv::ServoSw_Response num(::usb_servo::srv::ServoSw_Response::_num_type arg)
  {
    msg_.num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::usb_servo::srv::ServoSw_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::usb_servo::srv::ServoSw_Response>()
{
  return usb_servo::srv::builder::Init_ServoSw_Response_num();
}

}  // namespace usb_servo

#endif  // USB_SERVO__SRV__DETAIL__SERVO_SW__BUILDER_HPP_
