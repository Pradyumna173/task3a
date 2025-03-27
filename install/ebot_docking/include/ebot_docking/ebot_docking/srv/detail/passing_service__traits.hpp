// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ebot_docking:srv/PassingService.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__TRAITS_HPP_
#define EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ebot_docking/srv/detail/passing_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ebot_docking
{

namespace srv
{

inline void to_flow_style_yaml(
  const PassingService_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PassingService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PassingService_Request & msg, bool use_flow_style = false)
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
  const ebot_docking::srv::PassingService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebot_docking::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebot_docking::srv::to_yaml() instead")]]
inline std::string to_yaml(const ebot_docking::srv::PassingService_Request & msg)
{
  return ebot_docking::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ebot_docking::srv::PassingService_Request>()
{
  return "ebot_docking::srv::PassingService_Request";
}

template<>
inline const char * name<ebot_docking::srv::PassingService_Request>()
{
  return "ebot_docking/srv/PassingService_Request";
}

template<>
struct has_fixed_size<ebot_docking::srv::PassingService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ebot_docking::srv::PassingService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ebot_docking::srv::PassingService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ebot_docking
{

namespace srv
{

inline void to_flow_style_yaml(
  const PassingService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: conveyer
  {
    out << "conveyer: ";
    rosidl_generator_traits::value_to_yaml(msg.conveyer, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PassingService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: conveyer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "conveyer: ";
    rosidl_generator_traits::value_to_yaml(msg.conveyer, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PassingService_Response & msg, bool use_flow_style = false)
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
  const ebot_docking::srv::PassingService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebot_docking::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebot_docking::srv::to_yaml() instead")]]
inline std::string to_yaml(const ebot_docking::srv::PassingService_Response & msg)
{
  return ebot_docking::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ebot_docking::srv::PassingService_Response>()
{
  return "ebot_docking::srv::PassingService_Response";
}

template<>
inline const char * name<ebot_docking::srv::PassingService_Response>()
{
  return "ebot_docking/srv/PassingService_Response";
}

template<>
struct has_fixed_size<ebot_docking::srv::PassingService_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ebot_docking::srv::PassingService_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ebot_docking::srv::PassingService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ebot_docking::srv::PassingService>()
{
  return "ebot_docking::srv::PassingService";
}

template<>
inline const char * name<ebot_docking::srv::PassingService>()
{
  return "ebot_docking/srv/PassingService";
}

template<>
struct has_fixed_size<ebot_docking::srv::PassingService>
  : std::integral_constant<
    bool,
    has_fixed_size<ebot_docking::srv::PassingService_Request>::value &&
    has_fixed_size<ebot_docking::srv::PassingService_Response>::value
  >
{
};

template<>
struct has_bounded_size<ebot_docking::srv::PassingService>
  : std::integral_constant<
    bool,
    has_bounded_size<ebot_docking::srv::PassingService_Request>::value &&
    has_bounded_size<ebot_docking::srv::PassingService_Response>::value
  >
{
};

template<>
struct is_service<ebot_docking::srv::PassingService>
  : std::true_type
{
};

template<>
struct is_service_request<ebot_docking::srv::PassingService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ebot_docking::srv::PassingService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__TRAITS_HPP_
