// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ebot_docking:srv/DockSw.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__DOCK_SW__FUNCTIONS_H_
#define EBOT_DOCKING__SRV__DETAIL__DOCK_SW__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ebot_docking/msg/rosidl_generator_c__visibility_control.h"

#include "ebot_docking/srv/detail/dock_sw__struct.h"

/// Initialize srv/DockSw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ebot_docking__srv__DockSw_Request
 * )) before or use
 * ebot_docking__srv__DockSw_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Request__init(ebot_docking__srv__DockSw_Request * msg);

/// Finalize srv/DockSw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Request__fini(ebot_docking__srv__DockSw_Request * msg);

/// Create srv/DockSw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ebot_docking__srv__DockSw_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
ebot_docking__srv__DockSw_Request *
ebot_docking__srv__DockSw_Request__create();

/// Destroy srv/DockSw message.
/**
 * It calls
 * ebot_docking__srv__DockSw_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Request__destroy(ebot_docking__srv__DockSw_Request * msg);

/// Check for srv/DockSw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Request__are_equal(const ebot_docking__srv__DockSw_Request * lhs, const ebot_docking__srv__DockSw_Request * rhs);

/// Copy a srv/DockSw message.
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
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Request__copy(
  const ebot_docking__srv__DockSw_Request * input,
  ebot_docking__srv__DockSw_Request * output);

/// Initialize array of srv/DockSw messages.
/**
 * It allocates the memory for the number of elements and calls
 * ebot_docking__srv__DockSw_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Request__Sequence__init(ebot_docking__srv__DockSw_Request__Sequence * array, size_t size);

/// Finalize array of srv/DockSw messages.
/**
 * It calls
 * ebot_docking__srv__DockSw_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Request__Sequence__fini(ebot_docking__srv__DockSw_Request__Sequence * array);

/// Create array of srv/DockSw messages.
/**
 * It allocates the memory for the array and calls
 * ebot_docking__srv__DockSw_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
ebot_docking__srv__DockSw_Request__Sequence *
ebot_docking__srv__DockSw_Request__Sequence__create(size_t size);

/// Destroy array of srv/DockSw messages.
/**
 * It calls
 * ebot_docking__srv__DockSw_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Request__Sequence__destroy(ebot_docking__srv__DockSw_Request__Sequence * array);

/// Check for srv/DockSw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Request__Sequence__are_equal(const ebot_docking__srv__DockSw_Request__Sequence * lhs, const ebot_docking__srv__DockSw_Request__Sequence * rhs);

/// Copy an array of srv/DockSw messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Request__Sequence__copy(
  const ebot_docking__srv__DockSw_Request__Sequence * input,
  ebot_docking__srv__DockSw_Request__Sequence * output);

/// Initialize srv/DockSw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ebot_docking__srv__DockSw_Response
 * )) before or use
 * ebot_docking__srv__DockSw_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Response__init(ebot_docking__srv__DockSw_Response * msg);

/// Finalize srv/DockSw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Response__fini(ebot_docking__srv__DockSw_Response * msg);

/// Create srv/DockSw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ebot_docking__srv__DockSw_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
ebot_docking__srv__DockSw_Response *
ebot_docking__srv__DockSw_Response__create();

/// Destroy srv/DockSw message.
/**
 * It calls
 * ebot_docking__srv__DockSw_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Response__destroy(ebot_docking__srv__DockSw_Response * msg);

/// Check for srv/DockSw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Response__are_equal(const ebot_docking__srv__DockSw_Response * lhs, const ebot_docking__srv__DockSw_Response * rhs);

/// Copy a srv/DockSw message.
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
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Response__copy(
  const ebot_docking__srv__DockSw_Response * input,
  ebot_docking__srv__DockSw_Response * output);

/// Initialize array of srv/DockSw messages.
/**
 * It allocates the memory for the number of elements and calls
 * ebot_docking__srv__DockSw_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Response__Sequence__init(ebot_docking__srv__DockSw_Response__Sequence * array, size_t size);

/// Finalize array of srv/DockSw messages.
/**
 * It calls
 * ebot_docking__srv__DockSw_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Response__Sequence__fini(ebot_docking__srv__DockSw_Response__Sequence * array);

/// Create array of srv/DockSw messages.
/**
 * It allocates the memory for the array and calls
 * ebot_docking__srv__DockSw_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
ebot_docking__srv__DockSw_Response__Sequence *
ebot_docking__srv__DockSw_Response__Sequence__create(size_t size);

/// Destroy array of srv/DockSw messages.
/**
 * It calls
 * ebot_docking__srv__DockSw_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
void
ebot_docking__srv__DockSw_Response__Sequence__destroy(ebot_docking__srv__DockSw_Response__Sequence * array);

/// Check for srv/DockSw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Response__Sequence__are_equal(const ebot_docking__srv__DockSw_Response__Sequence * lhs, const ebot_docking__srv__DockSw_Response__Sequence * rhs);

/// Copy an array of srv/DockSw messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ebot_docking
bool
ebot_docking__srv__DockSw_Response__Sequence__copy(
  const ebot_docking__srv__DockSw_Response__Sequence * input,
  ebot_docking__srv__DockSw_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EBOT_DOCKING__SRV__DETAIL__DOCK_SW__FUNCTIONS_H_
