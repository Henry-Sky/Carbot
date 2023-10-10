// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from carbot_interfaces:srv/Test.idl
// generated code does not contain a copyright notice
#include "carbot_interfaces/srv/detail/test__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
carbot_interfaces__srv__Test_Request__init(carbot_interfaces__srv__Test_Request * msg)
{
  if (!msg) {
    return false;
  }
  // get_num
  return true;
}

void
carbot_interfaces__srv__Test_Request__fini(carbot_interfaces__srv__Test_Request * msg)
{
  if (!msg) {
    return;
  }
  // get_num
}

bool
carbot_interfaces__srv__Test_Request__are_equal(const carbot_interfaces__srv__Test_Request * lhs, const carbot_interfaces__srv__Test_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // get_num
  if (lhs->get_num != rhs->get_num) {
    return false;
  }
  return true;
}

bool
carbot_interfaces__srv__Test_Request__copy(
  const carbot_interfaces__srv__Test_Request * input,
  carbot_interfaces__srv__Test_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // get_num
  output->get_num = input->get_num;
  return true;
}

carbot_interfaces__srv__Test_Request *
carbot_interfaces__srv__Test_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carbot_interfaces__srv__Test_Request * msg = (carbot_interfaces__srv__Test_Request *)allocator.allocate(sizeof(carbot_interfaces__srv__Test_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(carbot_interfaces__srv__Test_Request));
  bool success = carbot_interfaces__srv__Test_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
carbot_interfaces__srv__Test_Request__destroy(carbot_interfaces__srv__Test_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    carbot_interfaces__srv__Test_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
carbot_interfaces__srv__Test_Request__Sequence__init(carbot_interfaces__srv__Test_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carbot_interfaces__srv__Test_Request * data = NULL;

  if (size) {
    data = (carbot_interfaces__srv__Test_Request *)allocator.zero_allocate(size, sizeof(carbot_interfaces__srv__Test_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = carbot_interfaces__srv__Test_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        carbot_interfaces__srv__Test_Request__fini(&data[i - 1]);
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
carbot_interfaces__srv__Test_Request__Sequence__fini(carbot_interfaces__srv__Test_Request__Sequence * array)
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
      carbot_interfaces__srv__Test_Request__fini(&array->data[i]);
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

carbot_interfaces__srv__Test_Request__Sequence *
carbot_interfaces__srv__Test_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carbot_interfaces__srv__Test_Request__Sequence * array = (carbot_interfaces__srv__Test_Request__Sequence *)allocator.allocate(sizeof(carbot_interfaces__srv__Test_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = carbot_interfaces__srv__Test_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
carbot_interfaces__srv__Test_Request__Sequence__destroy(carbot_interfaces__srv__Test_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    carbot_interfaces__srv__Test_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
carbot_interfaces__srv__Test_Request__Sequence__are_equal(const carbot_interfaces__srv__Test_Request__Sequence * lhs, const carbot_interfaces__srv__Test_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!carbot_interfaces__srv__Test_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
carbot_interfaces__srv__Test_Request__Sequence__copy(
  const carbot_interfaces__srv__Test_Request__Sequence * input,
  carbot_interfaces__srv__Test_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(carbot_interfaces__srv__Test_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    carbot_interfaces__srv__Test_Request * data =
      (carbot_interfaces__srv__Test_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!carbot_interfaces__srv__Test_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          carbot_interfaces__srv__Test_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!carbot_interfaces__srv__Test_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
carbot_interfaces__srv__Test_Response__init(carbot_interfaces__srv__Test_Response * msg)
{
  if (!msg) {
    return false;
  }
  // num1
  // num2
  return true;
}

void
carbot_interfaces__srv__Test_Response__fini(carbot_interfaces__srv__Test_Response * msg)
{
  if (!msg) {
    return;
  }
  // num1
  // num2
}

bool
carbot_interfaces__srv__Test_Response__are_equal(const carbot_interfaces__srv__Test_Response * lhs, const carbot_interfaces__srv__Test_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num1
  if (lhs->num1 != rhs->num1) {
    return false;
  }
  // num2
  if (lhs->num2 != rhs->num2) {
    return false;
  }
  return true;
}

bool
carbot_interfaces__srv__Test_Response__copy(
  const carbot_interfaces__srv__Test_Response * input,
  carbot_interfaces__srv__Test_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // num1
  output->num1 = input->num1;
  // num2
  output->num2 = input->num2;
  return true;
}

carbot_interfaces__srv__Test_Response *
carbot_interfaces__srv__Test_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carbot_interfaces__srv__Test_Response * msg = (carbot_interfaces__srv__Test_Response *)allocator.allocate(sizeof(carbot_interfaces__srv__Test_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(carbot_interfaces__srv__Test_Response));
  bool success = carbot_interfaces__srv__Test_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
carbot_interfaces__srv__Test_Response__destroy(carbot_interfaces__srv__Test_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    carbot_interfaces__srv__Test_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
carbot_interfaces__srv__Test_Response__Sequence__init(carbot_interfaces__srv__Test_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carbot_interfaces__srv__Test_Response * data = NULL;

  if (size) {
    data = (carbot_interfaces__srv__Test_Response *)allocator.zero_allocate(size, sizeof(carbot_interfaces__srv__Test_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = carbot_interfaces__srv__Test_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        carbot_interfaces__srv__Test_Response__fini(&data[i - 1]);
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
carbot_interfaces__srv__Test_Response__Sequence__fini(carbot_interfaces__srv__Test_Response__Sequence * array)
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
      carbot_interfaces__srv__Test_Response__fini(&array->data[i]);
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

carbot_interfaces__srv__Test_Response__Sequence *
carbot_interfaces__srv__Test_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carbot_interfaces__srv__Test_Response__Sequence * array = (carbot_interfaces__srv__Test_Response__Sequence *)allocator.allocate(sizeof(carbot_interfaces__srv__Test_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = carbot_interfaces__srv__Test_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
carbot_interfaces__srv__Test_Response__Sequence__destroy(carbot_interfaces__srv__Test_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    carbot_interfaces__srv__Test_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
carbot_interfaces__srv__Test_Response__Sequence__are_equal(const carbot_interfaces__srv__Test_Response__Sequence * lhs, const carbot_interfaces__srv__Test_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!carbot_interfaces__srv__Test_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
carbot_interfaces__srv__Test_Response__Sequence__copy(
  const carbot_interfaces__srv__Test_Response__Sequence * input,
  carbot_interfaces__srv__Test_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(carbot_interfaces__srv__Test_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    carbot_interfaces__srv__Test_Response * data =
      (carbot_interfaces__srv__Test_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!carbot_interfaces__srv__Test_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          carbot_interfaces__srv__Test_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!carbot_interfaces__srv__Test_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
