// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from omo_r1mini_interfaces:srv/Onoff.idl
// generated code does not contain a copyright notice
#include "omo_r1mini_interfaces/srv/detail/onoff__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
omo_r1mini_interfaces__srv__Onoff_Request__init(omo_r1mini_interfaces__srv__Onoff_Request * msg)
{
  if (!msg) {
    return false;
  }
  // set
  return true;
}

void
omo_r1mini_interfaces__srv__Onoff_Request__fini(omo_r1mini_interfaces__srv__Onoff_Request * msg)
{
  if (!msg) {
    return;
  }
  // set
}

bool
omo_r1mini_interfaces__srv__Onoff_Request__are_equal(const omo_r1mini_interfaces__srv__Onoff_Request * lhs, const omo_r1mini_interfaces__srv__Onoff_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // set
  if (lhs->set != rhs->set) {
    return false;
  }
  return true;
}

bool
omo_r1mini_interfaces__srv__Onoff_Request__copy(
  const omo_r1mini_interfaces__srv__Onoff_Request * input,
  omo_r1mini_interfaces__srv__Onoff_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // set
  output->set = input->set;
  return true;
}

omo_r1mini_interfaces__srv__Onoff_Request *
omo_r1mini_interfaces__srv__Onoff_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  omo_r1mini_interfaces__srv__Onoff_Request * msg = (omo_r1mini_interfaces__srv__Onoff_Request *)allocator.allocate(sizeof(omo_r1mini_interfaces__srv__Onoff_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(omo_r1mini_interfaces__srv__Onoff_Request));
  bool success = omo_r1mini_interfaces__srv__Onoff_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
omo_r1mini_interfaces__srv__Onoff_Request__destroy(omo_r1mini_interfaces__srv__Onoff_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    omo_r1mini_interfaces__srv__Onoff_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
omo_r1mini_interfaces__srv__Onoff_Request__Sequence__init(omo_r1mini_interfaces__srv__Onoff_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  omo_r1mini_interfaces__srv__Onoff_Request * data = NULL;

  if (size) {
    data = (omo_r1mini_interfaces__srv__Onoff_Request *)allocator.zero_allocate(size, sizeof(omo_r1mini_interfaces__srv__Onoff_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = omo_r1mini_interfaces__srv__Onoff_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        omo_r1mini_interfaces__srv__Onoff_Request__fini(&data[i - 1]);
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
omo_r1mini_interfaces__srv__Onoff_Request__Sequence__fini(omo_r1mini_interfaces__srv__Onoff_Request__Sequence * array)
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
      omo_r1mini_interfaces__srv__Onoff_Request__fini(&array->data[i]);
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

omo_r1mini_interfaces__srv__Onoff_Request__Sequence *
omo_r1mini_interfaces__srv__Onoff_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  omo_r1mini_interfaces__srv__Onoff_Request__Sequence * array = (omo_r1mini_interfaces__srv__Onoff_Request__Sequence *)allocator.allocate(sizeof(omo_r1mini_interfaces__srv__Onoff_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = omo_r1mini_interfaces__srv__Onoff_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
omo_r1mini_interfaces__srv__Onoff_Request__Sequence__destroy(omo_r1mini_interfaces__srv__Onoff_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    omo_r1mini_interfaces__srv__Onoff_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
omo_r1mini_interfaces__srv__Onoff_Request__Sequence__are_equal(const omo_r1mini_interfaces__srv__Onoff_Request__Sequence * lhs, const omo_r1mini_interfaces__srv__Onoff_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!omo_r1mini_interfaces__srv__Onoff_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
omo_r1mini_interfaces__srv__Onoff_Request__Sequence__copy(
  const omo_r1mini_interfaces__srv__Onoff_Request__Sequence * input,
  omo_r1mini_interfaces__srv__Onoff_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(omo_r1mini_interfaces__srv__Onoff_Request);
    omo_r1mini_interfaces__srv__Onoff_Request * data =
      (omo_r1mini_interfaces__srv__Onoff_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!omo_r1mini_interfaces__srv__Onoff_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          omo_r1mini_interfaces__srv__Onoff_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!omo_r1mini_interfaces__srv__Onoff_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
omo_r1mini_interfaces__srv__Onoff_Response__init(omo_r1mini_interfaces__srv__Onoff_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
omo_r1mini_interfaces__srv__Onoff_Response__fini(omo_r1mini_interfaces__srv__Onoff_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
omo_r1mini_interfaces__srv__Onoff_Response__are_equal(const omo_r1mini_interfaces__srv__Onoff_Response * lhs, const omo_r1mini_interfaces__srv__Onoff_Response * rhs)
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
omo_r1mini_interfaces__srv__Onoff_Response__copy(
  const omo_r1mini_interfaces__srv__Onoff_Response * input,
  omo_r1mini_interfaces__srv__Onoff_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

omo_r1mini_interfaces__srv__Onoff_Response *
omo_r1mini_interfaces__srv__Onoff_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  omo_r1mini_interfaces__srv__Onoff_Response * msg = (omo_r1mini_interfaces__srv__Onoff_Response *)allocator.allocate(sizeof(omo_r1mini_interfaces__srv__Onoff_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(omo_r1mini_interfaces__srv__Onoff_Response));
  bool success = omo_r1mini_interfaces__srv__Onoff_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
omo_r1mini_interfaces__srv__Onoff_Response__destroy(omo_r1mini_interfaces__srv__Onoff_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    omo_r1mini_interfaces__srv__Onoff_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
omo_r1mini_interfaces__srv__Onoff_Response__Sequence__init(omo_r1mini_interfaces__srv__Onoff_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  omo_r1mini_interfaces__srv__Onoff_Response * data = NULL;

  if (size) {
    data = (omo_r1mini_interfaces__srv__Onoff_Response *)allocator.zero_allocate(size, sizeof(omo_r1mini_interfaces__srv__Onoff_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = omo_r1mini_interfaces__srv__Onoff_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        omo_r1mini_interfaces__srv__Onoff_Response__fini(&data[i - 1]);
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
omo_r1mini_interfaces__srv__Onoff_Response__Sequence__fini(omo_r1mini_interfaces__srv__Onoff_Response__Sequence * array)
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
      omo_r1mini_interfaces__srv__Onoff_Response__fini(&array->data[i]);
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

omo_r1mini_interfaces__srv__Onoff_Response__Sequence *
omo_r1mini_interfaces__srv__Onoff_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  omo_r1mini_interfaces__srv__Onoff_Response__Sequence * array = (omo_r1mini_interfaces__srv__Onoff_Response__Sequence *)allocator.allocate(sizeof(omo_r1mini_interfaces__srv__Onoff_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = omo_r1mini_interfaces__srv__Onoff_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
omo_r1mini_interfaces__srv__Onoff_Response__Sequence__destroy(omo_r1mini_interfaces__srv__Onoff_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    omo_r1mini_interfaces__srv__Onoff_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
omo_r1mini_interfaces__srv__Onoff_Response__Sequence__are_equal(const omo_r1mini_interfaces__srv__Onoff_Response__Sequence * lhs, const omo_r1mini_interfaces__srv__Onoff_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!omo_r1mini_interfaces__srv__Onoff_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
omo_r1mini_interfaces__srv__Onoff_Response__Sequence__copy(
  const omo_r1mini_interfaces__srv__Onoff_Response__Sequence * input,
  omo_r1mini_interfaces__srv__Onoff_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(omo_r1mini_interfaces__srv__Onoff_Response);
    omo_r1mini_interfaces__srv__Onoff_Response * data =
      (omo_r1mini_interfaces__srv__Onoff_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!omo_r1mini_interfaces__srv__Onoff_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          omo_r1mini_interfaces__srv__Onoff_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!omo_r1mini_interfaces__srv__Onoff_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
