// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ebot_docking:srv/DockSw.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ebot_docking/srv/detail/dock_sw__rosidl_typesupport_introspection_c.h"
#include "ebot_docking/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ebot_docking/srv/detail/dock_sw__functions.h"
#include "ebot_docking/srv/detail/dock_sw__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ebot_docking__srv__DockSw_Request__init(message_memory);
}

void ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_fini_function(void * message_memory)
{
  ebot_docking__srv__DockSw_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_member_array[1] = {
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ebot_docking__srv__DockSw_Request, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_members = {
  "ebot_docking__srv",  // message namespace
  "DockSw_Request",  // message name
  1,  // number of fields
  sizeof(ebot_docking__srv__DockSw_Request),
  ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_member_array,  // message members
  ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_type_support_handle = {
  0,
  &ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ebot_docking
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebot_docking, srv, DockSw_Request)() {
  if (!ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_type_support_handle.typesupport_identifier) {
    ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ebot_docking__srv__DockSw_Request__rosidl_typesupport_introspection_c__DockSw_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ebot_docking/srv/detail/dock_sw__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ebot_docking/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ebot_docking/srv/detail/dock_sw__functions.h"
// already included above
// #include "ebot_docking/srv/detail/dock_sw__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ebot_docking__srv__DockSw_Response__init(message_memory);
}

void ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_fini_function(void * message_memory)
{
  ebot_docking__srv__DockSw_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ebot_docking__srv__DockSw_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_members = {
  "ebot_docking__srv",  // message namespace
  "DockSw_Response",  // message name
  1,  // number of fields
  sizeof(ebot_docking__srv__DockSw_Response),
  ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_member_array,  // message members
  ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_type_support_handle = {
  0,
  &ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ebot_docking
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebot_docking, srv, DockSw_Response)() {
  if (!ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_type_support_handle.typesupport_identifier) {
    ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ebot_docking__srv__DockSw_Response__rosidl_typesupport_introspection_c__DockSw_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ebot_docking/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ebot_docking/srv/detail/dock_sw__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_service_members = {
  "ebot_docking__srv",  // service namespace
  "DockSw",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_Request_message_type_support_handle,
  NULL  // response message
  // ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_Response_message_type_support_handle
};

static rosidl_service_type_support_t ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_service_type_support_handle = {
  0,
  &ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebot_docking, srv, DockSw_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebot_docking, srv, DockSw_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ebot_docking
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebot_docking, srv, DockSw)() {
  if (!ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_service_type_support_handle.typesupport_identifier) {
    ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebot_docking, srv, DockSw_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebot_docking, srv, DockSw_Response)()->data;
  }

  return &ebot_docking__srv__detail__dock_sw__rosidl_typesupport_introspection_c__DockSw_service_type_support_handle;
}
