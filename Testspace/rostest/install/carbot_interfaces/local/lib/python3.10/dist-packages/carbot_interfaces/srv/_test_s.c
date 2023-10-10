// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from carbot_interfaces:srv/Test.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "carbot_interfaces/srv/detail/test__struct.h"
#include "carbot_interfaces/srv/detail/test__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool carbot_interfaces__srv__test__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("carbot_interfaces.srv._test.Test_Request", full_classname_dest, 40) == 0);
  }
  carbot_interfaces__srv__Test_Request * ros_message = _ros_message;
  {  // get_num
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_num");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->get_num = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * carbot_interfaces__srv__test__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Test_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("carbot_interfaces.srv._test");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Test_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  carbot_interfaces__srv__Test_Request * ros_message = (carbot_interfaces__srv__Test_Request *)raw_ros_message;
  {  // get_num
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->get_num ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_num", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "carbot_interfaces/srv/detail/test__struct.h"
// already included above
// #include "carbot_interfaces/srv/detail/test__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool carbot_interfaces__srv__test__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("carbot_interfaces.srv._test.Test_Response", full_classname_dest, 41) == 0);
  }
  carbot_interfaces__srv__Test_Response * ros_message = _ros_message;
  {  // num1
    PyObject * field = PyObject_GetAttrString(_pymsg, "num1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->num1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // num2
    PyObject * field = PyObject_GetAttrString(_pymsg, "num2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->num2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * carbot_interfaces__srv__test__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Test_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("carbot_interfaces.srv._test");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Test_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  carbot_interfaces__srv__Test_Response * ros_message = (carbot_interfaces__srv__Test_Response *)raw_ros_message;
  {  // num1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->num1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->num2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
