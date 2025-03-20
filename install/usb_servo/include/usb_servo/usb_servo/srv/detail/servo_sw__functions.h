// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice

#ifndef USB_SERVO__SRV__DETAIL__SERVO_SW__FUNCTIONS_H_
#define USB_SERVO__SRV__DETAIL__SERVO_SW__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "usb_servo/msg/rosidl_generator_c__visibility_control.h"

#include "usb_servo/srv/detail/servo_sw__struct.h"

/// Initialize srv/ServoSw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * usb_servo__srv__ServoSw_Request
 * )) before or use
 * usb_servo__srv__ServoSw_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Request__init(usb_servo__srv__ServoSw_Request * msg);

/// Finalize srv/ServoSw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Request__fini(usb_servo__srv__ServoSw_Request * msg);

/// Create srv/ServoSw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * usb_servo__srv__ServoSw_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
usb_servo__srv__ServoSw_Request *
usb_servo__srv__ServoSw_Request__create();

/// Destroy srv/ServoSw message.
/**
 * It calls
 * usb_servo__srv__ServoSw_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Request__destroy(usb_servo__srv__ServoSw_Request * msg);

/// Check for srv/ServoSw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Request__are_equal(const usb_servo__srv__ServoSw_Request * lhs, const usb_servo__srv__ServoSw_Request * rhs);

/// Copy a srv/ServoSw message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Request__copy(
  const usb_servo__srv__ServoSw_Request * input,
  usb_servo__srv__ServoSw_Request * output);

/// Initialize array of srv/ServoSw messages.
/**
 * It allocates the memory for the number of elements and calls
 * usb_servo__srv__ServoSw_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Request__Sequence__init(usb_servo__srv__ServoSw_Request__Sequence * array, size_t size);

/// Finalize array of srv/ServoSw messages.
/**
 * It calls
 * usb_servo__srv__ServoSw_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Request__Sequence__fini(usb_servo__srv__ServoSw_Request__Sequence * array);

/// Create array of srv/ServoSw messages.
/**
 * It allocates the memory for the array and calls
 * usb_servo__srv__ServoSw_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
usb_servo__srv__ServoSw_Request__Sequence *
usb_servo__srv__ServoSw_Request__Sequence__create(size_t size);

/// Destroy array of srv/ServoSw messages.
/**
 * It calls
 * usb_servo__srv__ServoSw_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Request__Sequence__destroy(usb_servo__srv__ServoSw_Request__Sequence * array);

/// Check for srv/ServoSw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Request__Sequence__are_equal(const usb_servo__srv__ServoSw_Request__Sequence * lhs, const usb_servo__srv__ServoSw_Request__Sequence * rhs);

/// Copy an array of srv/ServoSw messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Request__Sequence__copy(
  const usb_servo__srv__ServoSw_Request__Sequence * input,
  usb_servo__srv__ServoSw_Request__Sequence * output);

/// Initialize srv/ServoSw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * usb_servo__srv__ServoSw_Response
 * )) before or use
 * usb_servo__srv__ServoSw_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Response__init(usb_servo__srv__ServoSw_Response * msg);

/// Finalize srv/ServoSw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Response__fini(usb_servo__srv__ServoSw_Response * msg);

/// Create srv/ServoSw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * usb_servo__srv__ServoSw_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
usb_servo__srv__ServoSw_Response *
usb_servo__srv__ServoSw_Response__create();

/// Destroy srv/ServoSw message.
/**
 * It calls
 * usb_servo__srv__ServoSw_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Response__destroy(usb_servo__srv__ServoSw_Response * msg);

/// Check for srv/ServoSw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Response__are_equal(const usb_servo__srv__ServoSw_Response * lhs, const usb_servo__srv__ServoSw_Response * rhs);

/// Copy a srv/ServoSw message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Response__copy(
  const usb_servo__srv__ServoSw_Response * input,
  usb_servo__srv__ServoSw_Response * output);

/// Initialize array of srv/ServoSw messages.
/**
 * It allocates the memory for the number of elements and calls
 * usb_servo__srv__ServoSw_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Response__Sequence__init(usb_servo__srv__ServoSw_Response__Sequence * array, size_t size);

/// Finalize array of srv/ServoSw messages.
/**
 * It calls
 * usb_servo__srv__ServoSw_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Response__Sequence__fini(usb_servo__srv__ServoSw_Response__Sequence * array);

/// Create array of srv/ServoSw messages.
/**
 * It allocates the memory for the array and calls
 * usb_servo__srv__ServoSw_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
usb_servo__srv__ServoSw_Response__Sequence *
usb_servo__srv__ServoSw_Response__Sequence__create(size_t size);

/// Destroy array of srv/ServoSw messages.
/**
 * It calls
 * usb_servo__srv__ServoSw_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
void
usb_servo__srv__ServoSw_Response__Sequence__destroy(usb_servo__srv__ServoSw_Response__Sequence * array);

/// Check for srv/ServoSw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Response__Sequence__are_equal(const usb_servo__srv__ServoSw_Response__Sequence * lhs, const usb_servo__srv__ServoSw_Response__Sequence * rhs);

/// Copy an array of srv/ServoSw messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_usb_servo
bool
usb_servo__srv__ServoSw_Response__Sequence__copy(
  const usb_servo__srv__ServoSw_Response__Sequence * input,
  usb_servo__srv__ServoSw_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // USB_SERVO__SRV__DETAIL__SERVO_SW__FUNCTIONS_H_
