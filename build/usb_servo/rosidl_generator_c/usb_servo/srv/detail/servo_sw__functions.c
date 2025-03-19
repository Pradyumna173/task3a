// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice
#include "usb_servo/srv/detail/servo_sw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
usb_servo__srv__ServoSw_Request__init(usb_servo__srv__ServoSw_Request * msg)
{
  if (!msg) {
    return false;
  }
  // servostate
  return true;
}

void
usb_servo__srv__ServoSw_Request__fini(usb_servo__srv__ServoSw_Request * msg)
{
  if (!msg) {
    return;
  }
  // servostate
}

bool
usb_servo__srv__ServoSw_Request__are_equal(const usb_servo__srv__ServoSw_Request * lhs, const usb_servo__srv__ServoSw_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // servostate
  if (lhs->servostate != rhs->servostate) {
    return false;
  }
  return true;
}

bool
usb_servo__srv__ServoSw_Request__copy(
  const usb_servo__srv__ServoSw_Request * input,
  usb_servo__srv__ServoSw_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // servostate
  output->servostate = input->servostate;
  return true;
}

usb_servo__srv__ServoSw_Request *
usb_servo__srv__ServoSw_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  usb_servo__srv__ServoSw_Request * msg = (usb_servo__srv__ServoSw_Request *)allocator.allocate(sizeof(usb_servo__srv__ServoSw_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(usb_servo__srv__ServoSw_Request));
  bool success = usb_servo__srv__ServoSw_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
usb_servo__srv__ServoSw_Request__destroy(usb_servo__srv__ServoSw_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    usb_servo__srv__ServoSw_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
usb_servo__srv__ServoSw_Request__Sequence__init(usb_servo__srv__ServoSw_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  usb_servo__srv__ServoSw_Request * data = NULL;

  if (size) {
    data = (usb_servo__srv__ServoSw_Request *)allocator.zero_allocate(size, sizeof(usb_servo__srv__ServoSw_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = usb_servo__srv__ServoSw_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        usb_servo__srv__ServoSw_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
usb_servo__srv__ServoSw_Request__Sequence__fini(usb_servo__srv__ServoSw_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      usb_servo__srv__ServoSw_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

usb_servo__srv__ServoSw_Request__Sequence *
usb_servo__srv__ServoSw_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  usb_servo__srv__ServoSw_Request__Sequence * array = (usb_servo__srv__ServoSw_Request__Sequence *)allocator.allocate(sizeof(usb_servo__srv__ServoSw_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = usb_servo__srv__ServoSw_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
usb_servo__srv__ServoSw_Request__Sequence__destroy(usb_servo__srv__ServoSw_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    usb_servo__srv__ServoSw_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
usb_servo__srv__ServoSw_Request__Sequence__are_equal(const usb_servo__srv__ServoSw_Request__Sequence * lhs, const usb_servo__srv__ServoSw_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!usb_servo__srv__ServoSw_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
usb_servo__srv__ServoSw_Request__Sequence__copy(
  const usb_servo__srv__ServoSw_Request__Sequence * input,
  usb_servo__srv__ServoSw_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(usb_servo__srv__ServoSw_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    usb_servo__srv__ServoSw_Request * data =
      (usb_servo__srv__ServoSw_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!usb_servo__srv__ServoSw_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          usb_servo__srv__ServoSw_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!usb_servo__srv__ServoSw_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
usb_servo__srv__ServoSw_Response__init(usb_servo__srv__ServoSw_Response * msg)
{
  if (!msg) {
    return false;
  }
  // num
  return true;
}

void
usb_servo__srv__ServoSw_Response__fini(usb_servo__srv__ServoSw_Response * msg)
{
  if (!msg) {
    return;
  }
  // num
}

bool
usb_servo__srv__ServoSw_Response__are_equal(const usb_servo__srv__ServoSw_Response * lhs, const usb_servo__srv__ServoSw_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num
  if (lhs->num != rhs->num) {
    return false;
  }
  return true;
}

bool
usb_servo__srv__ServoSw_Response__copy(
  const usb_servo__srv__ServoSw_Response * input,
  usb_servo__srv__ServoSw_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // num
  output->num = input->num;
  return true;
}

usb_servo__srv__ServoSw_Response *
usb_servo__srv__ServoSw_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  usb_servo__srv__ServoSw_Response * msg = (usb_servo__srv__ServoSw_Response *)allocator.allocate(sizeof(usb_servo__srv__ServoSw_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(usb_servo__srv__ServoSw_Response));
  bool success = usb_servo__srv__ServoSw_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
usb_servo__srv__ServoSw_Response__destroy(usb_servo__srv__ServoSw_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    usb_servo__srv__ServoSw_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
usb_servo__srv__ServoSw_Response__Sequence__init(usb_servo__srv__ServoSw_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  usb_servo__srv__ServoSw_Response * data = NULL;

  if (size) {
    data = (usb_servo__srv__ServoSw_Response *)allocator.zero_allocate(size, sizeof(usb_servo__srv__ServoSw_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = usb_servo__srv__ServoSw_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        usb_servo__srv__ServoSw_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
usb_servo__srv__ServoSw_Response__Sequence__fini(usb_servo__srv__ServoSw_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      usb_servo__srv__ServoSw_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

usb_servo__srv__ServoSw_Response__Sequence *
usb_servo__srv__ServoSw_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  usb_servo__srv__ServoSw_Response__Sequence * array = (usb_servo__srv__ServoSw_Response__Sequence *)allocator.allocate(sizeof(usb_servo__srv__ServoSw_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = usb_servo__srv__ServoSw_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
usb_servo__srv__ServoSw_Response__Sequence__destroy(usb_servo__srv__ServoSw_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    usb_servo__srv__ServoSw_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
usb_servo__srv__ServoSw_Response__Sequence__are_equal(const usb_servo__srv__ServoSw_Response__Sequence * lhs, const usb_servo__srv__ServoSw_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!usb_servo__srv__ServoSw_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
usb_servo__srv__ServoSw_Response__Sequence__copy(
  const usb_servo__srv__ServoSw_Response__Sequence * input,
  usb_servo__srv__ServoSw_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(usb_servo__srv__ServoSw_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    usb_servo__srv__ServoSw_Response * data =
      (usb_servo__srv__ServoSw_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!usb_servo__srv__ServoSw_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          usb_servo__srv__ServoSw_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!usb_servo__srv__ServoSw_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
