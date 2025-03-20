// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ebot_docking:srv/PassingService.idl
// generated code does not contain a copyright notice
#include "ebot_docking/srv/detail/passing_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
ebot_docking__srv__PassingService_Request__init(ebot_docking__srv__PassingService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ebot_docking__srv__PassingService_Request__fini(ebot_docking__srv__PassingService_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ebot_docking__srv__PassingService_Request__are_equal(const ebot_docking__srv__PassingService_Request * lhs, const ebot_docking__srv__PassingService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ebot_docking__srv__PassingService_Request__copy(
  const ebot_docking__srv__PassingService_Request * input,
  ebot_docking__srv__PassingService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ebot_docking__srv__PassingService_Request *
ebot_docking__srv__PassingService_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebot_docking__srv__PassingService_Request * msg = (ebot_docking__srv__PassingService_Request *)allocator.allocate(sizeof(ebot_docking__srv__PassingService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ebot_docking__srv__PassingService_Request));
  bool success = ebot_docking__srv__PassingService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ebot_docking__srv__PassingService_Request__destroy(ebot_docking__srv__PassingService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ebot_docking__srv__PassingService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ebot_docking__srv__PassingService_Request__Sequence__init(ebot_docking__srv__PassingService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebot_docking__srv__PassingService_Request * data = NULL;

  if (size) {
    data = (ebot_docking__srv__PassingService_Request *)allocator.zero_allocate(size, sizeof(ebot_docking__srv__PassingService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ebot_docking__srv__PassingService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ebot_docking__srv__PassingService_Request__fini(&data[i - 1]);
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
ebot_docking__srv__PassingService_Request__Sequence__fini(ebot_docking__srv__PassingService_Request__Sequence * array)
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
      ebot_docking__srv__PassingService_Request__fini(&array->data[i]);
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

ebot_docking__srv__PassingService_Request__Sequence *
ebot_docking__srv__PassingService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebot_docking__srv__PassingService_Request__Sequence * array = (ebot_docking__srv__PassingService_Request__Sequence *)allocator.allocate(sizeof(ebot_docking__srv__PassingService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ebot_docking__srv__PassingService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ebot_docking__srv__PassingService_Request__Sequence__destroy(ebot_docking__srv__PassingService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ebot_docking__srv__PassingService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ebot_docking__srv__PassingService_Request__Sequence__are_equal(const ebot_docking__srv__PassingService_Request__Sequence * lhs, const ebot_docking__srv__PassingService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ebot_docking__srv__PassingService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ebot_docking__srv__PassingService_Request__Sequence__copy(
  const ebot_docking__srv__PassingService_Request__Sequence * input,
  ebot_docking__srv__PassingService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ebot_docking__srv__PassingService_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ebot_docking__srv__PassingService_Request * data =
      (ebot_docking__srv__PassingService_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ebot_docking__srv__PassingService_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ebot_docking__srv__PassingService_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ebot_docking__srv__PassingService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ebot_docking__srv__PassingService_Response__init(ebot_docking__srv__PassingService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // conveyer
  return true;
}

void
ebot_docking__srv__PassingService_Response__fini(ebot_docking__srv__PassingService_Response * msg)
{
  if (!msg) {
    return;
  }
  // conveyer
}

bool
ebot_docking__srv__PassingService_Response__are_equal(const ebot_docking__srv__PassingService_Response * lhs, const ebot_docking__srv__PassingService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // conveyer
  if (lhs->conveyer != rhs->conveyer) {
    return false;
  }
  return true;
}

bool
ebot_docking__srv__PassingService_Response__copy(
  const ebot_docking__srv__PassingService_Response * input,
  ebot_docking__srv__PassingService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // conveyer
  output->conveyer = input->conveyer;
  return true;
}

ebot_docking__srv__PassingService_Response *
ebot_docking__srv__PassingService_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebot_docking__srv__PassingService_Response * msg = (ebot_docking__srv__PassingService_Response *)allocator.allocate(sizeof(ebot_docking__srv__PassingService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ebot_docking__srv__PassingService_Response));
  bool success = ebot_docking__srv__PassingService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ebot_docking__srv__PassingService_Response__destroy(ebot_docking__srv__PassingService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ebot_docking__srv__PassingService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ebot_docking__srv__PassingService_Response__Sequence__init(ebot_docking__srv__PassingService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebot_docking__srv__PassingService_Response * data = NULL;

  if (size) {
    data = (ebot_docking__srv__PassingService_Response *)allocator.zero_allocate(size, sizeof(ebot_docking__srv__PassingService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ebot_docking__srv__PassingService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ebot_docking__srv__PassingService_Response__fini(&data[i - 1]);
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
ebot_docking__srv__PassingService_Response__Sequence__fini(ebot_docking__srv__PassingService_Response__Sequence * array)
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
      ebot_docking__srv__PassingService_Response__fini(&array->data[i]);
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

ebot_docking__srv__PassingService_Response__Sequence *
ebot_docking__srv__PassingService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebot_docking__srv__PassingService_Response__Sequence * array = (ebot_docking__srv__PassingService_Response__Sequence *)allocator.allocate(sizeof(ebot_docking__srv__PassingService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ebot_docking__srv__PassingService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ebot_docking__srv__PassingService_Response__Sequence__destroy(ebot_docking__srv__PassingService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ebot_docking__srv__PassingService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ebot_docking__srv__PassingService_Response__Sequence__are_equal(const ebot_docking__srv__PassingService_Response__Sequence * lhs, const ebot_docking__srv__PassingService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ebot_docking__srv__PassingService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ebot_docking__srv__PassingService_Response__Sequence__copy(
  const ebot_docking__srv__PassingService_Response__Sequence * input,
  ebot_docking__srv__PassingService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ebot_docking__srv__PassingService_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ebot_docking__srv__PassingService_Response * data =
      (ebot_docking__srv__PassingService_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ebot_docking__srv__PassingService_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ebot_docking__srv__PassingService_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ebot_docking__srv__PassingService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
