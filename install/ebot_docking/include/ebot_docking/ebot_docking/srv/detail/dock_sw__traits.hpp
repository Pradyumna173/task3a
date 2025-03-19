// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ebot_docking:srv/DockSw.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__DOCK_SW__TRAITS_HPP_
#define EBOT_DOCKING__SRV__DETAIL__DOCK_SW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ebot_docking/srv/detail/dock_sw__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ebot_docking
{

namespace srv
{

inline void to_flow_style_yaml(
  const DockSw_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target
  {
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DockSw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DockSw_Request & msg, bool use_flow_style = false)
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

}  // namespace ebot_docking

namespace rosidl_generator_traits
{

[[deprecated("use ebot_docking::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ebot_docking::srv::DockSw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebot_docking::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebot_docking::srv::to_yaml() instead")]]
inline std::string to_yaml(const ebot_docking::srv::DockSw_Request & msg)
{
  return ebot_docking::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ebot_docking::srv::DockSw_Request>()
{
  return "ebot_docking::srv::DockSw_Request";
}

template<>
inline const char * name<ebot_docking::srv::DockSw_Request>()
{
  return "ebot_docking/srv/DockSw_Request";
}

template<>
struct has_fixed_size<ebot_docking::srv::DockSw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ebot_docking::srv::DockSw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ebot_docking::srv::DockSw_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ebot_docking
{

namespace srv
{

inline void to_flow_style_yaml(
  const DockSw_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DockSw_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DockSw_Response & msg, bool use_flow_style = false)
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

}  // namespace ebot_docking

namespace rosidl_generator_traits
{

[[deprecated("use ebot_docking::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ebot_docking::srv::DockSw_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebot_docking::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebot_docking::srv::to_yaml() instead")]]
inline std::string to_yaml(const ebot_docking::srv::DockSw_Response & msg)
{
  return ebot_docking::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ebot_docking::srv::DockSw_Response>()
{
  return "ebot_docking::srv::DockSw_Response";
}

template<>
inline const char * name<ebot_docking::srv::DockSw_Response>()
{
  return "ebot_docking/srv/DockSw_Response";
}

template<>
struct has_fixed_size<ebot_docking::srv::DockSw_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ebot_docking::srv::DockSw_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ebot_docking::srv::DockSw_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ebot_docking::srv::DockSw>()
{
  return "ebot_docking::srv::DockSw";
}

template<>
inline const char * name<ebot_docking::srv::DockSw>()
{
  return "ebot_docking/srv/DockSw";
}

template<>
struct has_fixed_size<ebot_docking::srv::DockSw>
  : std::integral_constant<
    bool,
    has_fixed_size<ebot_docking::srv::DockSw_Request>::value &&
    has_fixed_size<ebot_docking::srv::DockSw_Response>::value
  >
{
};

template<>
struct has_bounded_size<ebot_docking::srv::DockSw>
  : std::integral_constant<
    bool,
    has_bounded_size<ebot_docking::srv::DockSw_Request>::value &&
    has_bounded_size<ebot_docking::srv::DockSw_Response>::value
  >
{
};

template<>
struct is_service<ebot_docking::srv::DockSw>
  : std::true_type
{
};

template<>
struct is_service_request<ebot_docking::srv::DockSw_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ebot_docking::srv::DockSw_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EBOT_DOCKING__SRV__DETAIL__DOCK_SW__TRAITS_HPP_
