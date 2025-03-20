// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice

#ifndef USB_SERVO__SRV__DETAIL__SERVO_SW__TRAITS_HPP_
#define USB_SERVO__SRV__DETAIL__SERVO_SW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "usb_servo/srv/detail/servo_sw__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace usb_servo
{

namespace srv
{

inline void to_flow_style_yaml(
  const ServoSw_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: servostate
  {
    out << "servostate: ";
    rosidl_generator_traits::value_to_yaml(msg.servostate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ServoSw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: servostate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servostate: ";
    rosidl_generator_traits::value_to_yaml(msg.servostate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ServoSw_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace usb_servo

namespace rosidl_generator_traits
{

[[deprecated("use usb_servo::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const usb_servo::srv::ServoSw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  usb_servo::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use usb_servo::srv::to_yaml() instead")]]
inline std::string to_yaml(const usb_servo::srv::ServoSw_Request & msg)
{
  return usb_servo::srv::to_yaml(msg);
}

template<>
inline const char * data_type<usb_servo::srv::ServoSw_Request>()
{
  return "usb_servo::srv::ServoSw_Request";
}

template<>
inline const char * name<usb_servo::srv::ServoSw_Request>()
{
  return "usb_servo/srv/ServoSw_Request";
}

template<>
struct has_fixed_size<usb_servo::srv::ServoSw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<usb_servo::srv::ServoSw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<usb_servo::srv::ServoSw_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace usb_servo
{

namespace srv
{

inline void to_flow_style_yaml(
  const ServoSw_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: num
  {
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ServoSw_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ServoSw_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace usb_servo

namespace rosidl_generator_traits
{

[[deprecated("use usb_servo::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const usb_servo::srv::ServoSw_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  usb_servo::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use usb_servo::srv::to_yaml() instead")]]
inline std::string to_yaml(const usb_servo::srv::ServoSw_Response & msg)
{
  return usb_servo::srv::to_yaml(msg);
}

template<>
inline const char * data_type<usb_servo::srv::ServoSw_Response>()
{
  return "usb_servo::srv::ServoSw_Response";
}

template<>
inline const char * name<usb_servo::srv::ServoSw_Response>()
{
  return "usb_servo/srv/ServoSw_Response";
}

template<>
struct has_fixed_size<usb_servo::srv::ServoSw_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<usb_servo::srv::ServoSw_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<usb_servo::srv::ServoSw_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<usb_servo::srv::ServoSw>()
{
  return "usb_servo::srv::ServoSw";
}

template<>
inline const char * name<usb_servo::srv::ServoSw>()
{
  return "usb_servo/srv/ServoSw";
}

template<>
struct has_fixed_size<usb_servo::srv::ServoSw>
  : std::integral_constant<
    bool,
    has_fixed_size<usb_servo::srv::ServoSw_Request>::value &&
    has_fixed_size<usb_servo::srv::ServoSw_Response>::value
  >
{
};

template<>
struct has_bounded_size<usb_servo::srv::ServoSw>
  : std::integral_constant<
    bool,
    has_bounded_size<usb_servo::srv::ServoSw_Request>::value &&
    has_bounded_size<usb_servo::srv::ServoSw_Response>::value
  >
{
};

template<>
struct is_service<usb_servo::srv::ServoSw>
  : std::true_type
{
};

template<>
struct is_service_request<usb_servo::srv::ServoSw_Request>
  : std::true_type
{
};

template<>
struct is_service_response<usb_servo::srv::ServoSw_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // USB_SERVO__SRV__DETAIL__SERVO_SW__TRAITS_HPP_
