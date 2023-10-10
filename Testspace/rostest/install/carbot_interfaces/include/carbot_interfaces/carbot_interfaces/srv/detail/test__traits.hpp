// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carbot_interfaces:srv/Test.idl
// generated code does not contain a copyright notice

#ifndef CARBOT_INTERFACES__SRV__DETAIL__TEST__TRAITS_HPP_
#define CARBOT_INTERFACES__SRV__DETAIL__TEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carbot_interfaces/srv/detail/test__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carbot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Test_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: get_num
  {
    out << "get_num: ";
    rosidl_generator_traits::value_to_yaml(msg.get_num, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Test_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: get_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_num: ";
    rosidl_generator_traits::value_to_yaml(msg.get_num, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Test_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace carbot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use carbot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carbot_interfaces::srv::Test_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  carbot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carbot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const carbot_interfaces::srv::Test_Request & msg)
{
  return carbot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carbot_interfaces::srv::Test_Request>()
{
  return "carbot_interfaces::srv::Test_Request";
}

template<>
inline const char * name<carbot_interfaces::srv::Test_Request>()
{
  return "carbot_interfaces/srv/Test_Request";
}

template<>
struct has_fixed_size<carbot_interfaces::srv::Test_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carbot_interfaces::srv::Test_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carbot_interfaces::srv::Test_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace carbot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Test_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: num1
  {
    out << "num1: ";
    rosidl_generator_traits::value_to_yaml(msg.num1, out);
    out << ", ";
  }

  // member: num2
  {
    out << "num2: ";
    rosidl_generator_traits::value_to_yaml(msg.num2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Test_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num1: ";
    rosidl_generator_traits::value_to_yaml(msg.num1, out);
    out << "\n";
  }

  // member: num2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num2: ";
    rosidl_generator_traits::value_to_yaml(msg.num2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Test_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace carbot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use carbot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carbot_interfaces::srv::Test_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  carbot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carbot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const carbot_interfaces::srv::Test_Response & msg)
{
  return carbot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carbot_interfaces::srv::Test_Response>()
{
  return "carbot_interfaces::srv::Test_Response";
}

template<>
inline const char * name<carbot_interfaces::srv::Test_Response>()
{
  return "carbot_interfaces/srv/Test_Response";
}

template<>
struct has_fixed_size<carbot_interfaces::srv::Test_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carbot_interfaces::srv::Test_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carbot_interfaces::srv::Test_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<carbot_interfaces::srv::Test>()
{
  return "carbot_interfaces::srv::Test";
}

template<>
inline const char * name<carbot_interfaces::srv::Test>()
{
  return "carbot_interfaces/srv/Test";
}

template<>
struct has_fixed_size<carbot_interfaces::srv::Test>
  : std::integral_constant<
    bool,
    has_fixed_size<carbot_interfaces::srv::Test_Request>::value &&
    has_fixed_size<carbot_interfaces::srv::Test_Response>::value
  >
{
};

template<>
struct has_bounded_size<carbot_interfaces::srv::Test>
  : std::integral_constant<
    bool,
    has_bounded_size<carbot_interfaces::srv::Test_Request>::value &&
    has_bounded_size<carbot_interfaces::srv::Test_Response>::value
  >
{
};

template<>
struct is_service<carbot_interfaces::srv::Test>
  : std::true_type
{
};

template<>
struct is_service_request<carbot_interfaces::srv::Test_Request>
  : std::true_type
{
};

template<>
struct is_service_response<carbot_interfaces::srv::Test_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CARBOT_INTERFACES__SRV__DETAIL__TEST__TRAITS_HPP_
