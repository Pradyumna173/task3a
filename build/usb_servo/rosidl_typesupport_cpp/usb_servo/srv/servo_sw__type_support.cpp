// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "usb_servo/srv/detail/servo_sw__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace usb_servo
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ServoSw_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ServoSw_Request_type_support_ids_t;

static const _ServoSw_Request_type_support_ids_t _ServoSw_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ServoSw_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ServoSw_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ServoSw_Request_type_support_symbol_names_t _ServoSw_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, usb_servo, srv, ServoSw_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, usb_servo, srv, ServoSw_Request)),
  }
};

typedef struct _ServoSw_Request_type_support_data_t
{
  void * data[2];
} _ServoSw_Request_type_support_data_t;

static _ServoSw_Request_type_support_data_t _ServoSw_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ServoSw_Request_message_typesupport_map = {
  2,
  "usb_servo",
  &_ServoSw_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ServoSw_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ServoSw_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ServoSw_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ServoSw_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace usb_servo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<usb_servo::srv::ServoSw_Request>()
{
  return &::usb_servo::srv::rosidl_typesupport_cpp::ServoSw_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, usb_servo, srv, ServoSw_Request)() {
  return get_message_type_support_handle<usb_servo::srv::ServoSw_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "usb_servo/srv/detail/servo_sw__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace usb_servo
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ServoSw_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ServoSw_Response_type_support_ids_t;

static const _ServoSw_Response_type_support_ids_t _ServoSw_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ServoSw_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ServoSw_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ServoSw_Response_type_support_symbol_names_t _ServoSw_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, usb_servo, srv, ServoSw_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, usb_servo, srv, ServoSw_Response)),
  }
};

typedef struct _ServoSw_Response_type_support_data_t
{
  void * data[2];
} _ServoSw_Response_type_support_data_t;

static _ServoSw_Response_type_support_data_t _ServoSw_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ServoSw_Response_message_typesupport_map = {
  2,
  "usb_servo",
  &_ServoSw_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ServoSw_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ServoSw_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ServoSw_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ServoSw_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace usb_servo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<usb_servo::srv::ServoSw_Response>()
{
  return &::usb_servo::srv::rosidl_typesupport_cpp::ServoSw_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, usb_servo, srv, ServoSw_Response)() {
  return get_message_type_support_handle<usb_servo::srv::ServoSw_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "usb_servo/srv/detail/servo_sw__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace usb_servo
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ServoSw_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ServoSw_type_support_ids_t;

static const _ServoSw_type_support_ids_t _ServoSw_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ServoSw_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ServoSw_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ServoSw_type_support_symbol_names_t _ServoSw_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, usb_servo, srv, ServoSw)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, usb_servo, srv, ServoSw)),
  }
};

typedef struct _ServoSw_type_support_data_t
{
  void * data[2];
} _ServoSw_type_support_data_t;

static _ServoSw_type_support_data_t _ServoSw_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ServoSw_service_typesupport_map = {
  2,
  "usb_servo",
  &_ServoSw_service_typesupport_ids.typesupport_identifier[0],
  &_ServoSw_service_typesupport_symbol_names.symbol_name[0],
  &_ServoSw_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ServoSw_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ServoSw_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace usb_servo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<usb_servo::srv::ServoSw>()
{
  return &::usb_servo::srv::rosidl_typesupport_cpp::ServoSw_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, usb_servo, srv, ServoSw)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<usb_servo::srv::ServoSw>();
}

#ifdef __cplusplus
}
#endif
