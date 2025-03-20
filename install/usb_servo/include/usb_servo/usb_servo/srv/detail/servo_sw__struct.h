// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice

#ifndef USB_SERVO__SRV__DETAIL__SERVO_SW__STRUCT_H_
#define USB_SERVO__SRV__DETAIL__SERVO_SW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ServoSw in the package usb_servo.
typedef struct usb_servo__srv__ServoSw_Request
{
  bool servostate;
} usb_servo__srv__ServoSw_Request;

// Struct for a sequence of usb_servo__srv__ServoSw_Request.
typedef struct usb_servo__srv__ServoSw_Request__Sequence
{
  usb_servo__srv__ServoSw_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} usb_servo__srv__ServoSw_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ServoSw in the package usb_servo.
typedef struct usb_servo__srv__ServoSw_Response
{
  int16_t num;
} usb_servo__srv__ServoSw_Response;

// Struct for a sequence of usb_servo__srv__ServoSw_Response.
typedef struct usb_servo__srv__ServoSw_Response__Sequence
{
  usb_servo__srv__ServoSw_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} usb_servo__srv__ServoSw_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // USB_SERVO__SRV__DETAIL__SERVO_SW__STRUCT_H_
