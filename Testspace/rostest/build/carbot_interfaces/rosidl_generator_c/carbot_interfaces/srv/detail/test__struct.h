// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from carbot_interfaces:srv/Test.idl
// generated code does not contain a copyright notice

#ifndef CARBOT_INTERFACES__SRV__DETAIL__TEST__STRUCT_H_
#define CARBOT_INTERFACES__SRV__DETAIL__TEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Test in the package carbot_interfaces.
typedef struct carbot_interfaces__srv__Test_Request
{
  bool get_num;
} carbot_interfaces__srv__Test_Request;

// Struct for a sequence of carbot_interfaces__srv__Test_Request.
typedef struct carbot_interfaces__srv__Test_Request__Sequence
{
  carbot_interfaces__srv__Test_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carbot_interfaces__srv__Test_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Test in the package carbot_interfaces.
typedef struct carbot_interfaces__srv__Test_Response
{
  double num1;
  double num2;
} carbot_interfaces__srv__Test_Response;

// Struct for a sequence of carbot_interfaces__srv__Test_Response.
typedef struct carbot_interfaces__srv__Test_Response__Sequence
{
  carbot_interfaces__srv__Test_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carbot_interfaces__srv__Test_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CARBOT_INTERFACES__SRV__DETAIL__TEST__STRUCT_H_
