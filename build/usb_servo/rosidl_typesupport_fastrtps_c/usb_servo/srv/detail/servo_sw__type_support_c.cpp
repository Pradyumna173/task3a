// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice
#include "usb_servo/srv/detail/servo_sw__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "usb_servo/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "usb_servo/srv/detail/servo_sw__struct.h"
#include "usb_servo/srv/detail/servo_sw__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ServoSw_Request__ros_msg_type = usb_servo__srv__ServoSw_Request;

static bool _ServoSw_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ServoSw_Request__ros_msg_type * ros_message = static_cast<const _ServoSw_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: servostate
  {
    cdr << (ros_message->servostate ? true : false);
  }

  return true;
}

static bool _ServoSw_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ServoSw_Request__ros_msg_type * ros_message = static_cast<_ServoSw_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: servostate
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->servostate = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_usb_servo
size_t get_serialized_size_usb_servo__srv__ServoSw_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ServoSw_Request__ros_msg_type * ros_message = static_cast<const _ServoSw_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name servostate
  {
    size_t item_size = sizeof(ros_message->servostate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ServoSw_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_usb_servo__srv__ServoSw_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_usb_servo
size_t max_serialized_size_usb_servo__srv__ServoSw_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: servostate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = usb_servo__srv__ServoSw_Request;
    is_plain =
      (
      offsetof(DataType, servostate) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ServoSw_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_usb_servo__srv__ServoSw_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ServoSw_Request = {
  "usb_servo::srv",
  "ServoSw_Request",
  _ServoSw_Request__cdr_serialize,
  _ServoSw_Request__cdr_deserialize,
  _ServoSw_Request__get_serialized_size,
  _ServoSw_Request__max_serialized_size
};

static rosidl_message_type_support_t _ServoSw_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ServoSw_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, usb_servo, srv, ServoSw_Request)() {
  return &_ServoSw_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "usb_servo/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "usb_servo/srv/detail/servo_sw__struct.h"
// already included above
// #include "usb_servo/srv/detail/servo_sw__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ServoSw_Response__ros_msg_type = usb_servo__srv__ServoSw_Response;

static bool _ServoSw_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ServoSw_Response__ros_msg_type * ros_message = static_cast<const _ServoSw_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: num
  {
    cdr << ros_message->num;
  }

  return true;
}

static bool _ServoSw_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ServoSw_Response__ros_msg_type * ros_message = static_cast<_ServoSw_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: num
  {
    cdr >> ros_message->num;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_usb_servo
size_t get_serialized_size_usb_servo__srv__ServoSw_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ServoSw_Response__ros_msg_type * ros_message = static_cast<const _ServoSw_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name num
  {
    size_t item_size = sizeof(ros_message->num);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ServoSw_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_usb_servo__srv__ServoSw_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_usb_servo
size_t max_serialized_size_usb_servo__srv__ServoSw_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: num
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = usb_servo__srv__ServoSw_Response;
    is_plain =
      (
      offsetof(DataType, num) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ServoSw_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_usb_servo__srv__ServoSw_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ServoSw_Response = {
  "usb_servo::srv",
  "ServoSw_Response",
  _ServoSw_Response__cdr_serialize,
  _ServoSw_Response__cdr_deserialize,
  _ServoSw_Response__get_serialized_size,
  _ServoSw_Response__max_serialized_size
};

static rosidl_message_type_support_t _ServoSw_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ServoSw_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, usb_servo, srv, ServoSw_Response)() {
  return &_ServoSw_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "usb_servo/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "usb_servo/srv/servo_sw.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ServoSw__callbacks = {
  "usb_servo::srv",
  "ServoSw",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, usb_servo, srv, ServoSw_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, usb_servo, srv, ServoSw_Response)(),
};

static rosidl_service_type_support_t ServoSw__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ServoSw__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, usb_servo, srv, ServoSw)() {
  return &ServoSw__handle;
}

#if defined(__cplusplus)
}
#endif
