// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ebot_docking:srv/PassingService.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__STRUCT_H_
#define EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/PassingService in the package ebot_docking.
typedef struct ebot_docking__srv__PassingService_Request
{
  uint8_t structure_needs_at_least_one_member;
} ebot_docking__srv__PassingService_Request;

// Struct for a sequence of ebot_docking__srv__PassingService_Request.
typedef struct ebot_docking__srv__PassingService_Request__Sequence
{
  ebot_docking__srv__PassingService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ebot_docking__srv__PassingService_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/PassingService in the package ebot_docking.
typedef struct ebot_docking__srv__PassingService_Response
{
  int32_t conveyer;
} ebot_docking__srv__PassingService_Response;

// Struct for a sequence of ebot_docking__srv__PassingService_Response.
typedef struct ebot_docking__srv__PassingService_Response__Sequence
{
  ebot_docking__srv__PassingService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ebot_docking__srv__PassingService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__STRUCT_H_
