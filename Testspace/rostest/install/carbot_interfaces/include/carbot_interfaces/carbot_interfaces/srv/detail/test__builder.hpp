// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from carbot_interfaces:srv/Test.idl
// generated code does not contain a copyright notice

#ifndef CARBOT_INTERFACES__SRV__DETAIL__TEST__BUILDER_HPP_
#define CARBOT_INTERFACES__SRV__DETAIL__TEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "carbot_interfaces/srv/detail/test__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace carbot_interfaces
{

namespace srv
{

namespace builder
{

class Init_Test_Request_get_num
{
public:
  Init_Test_Request_get_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::carbot_interfaces::srv::Test_Request get_num(::carbot_interfaces::srv::Test_Request::_get_num_type arg)
  {
    msg_.get_num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::carbot_interfaces::srv::Test_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::carbot_interfaces::srv::Test_Request>()
{
  return carbot_interfaces::srv::builder::Init_Test_Request_get_num();
}

}  // namespace carbot_interfaces


namespace carbot_interfaces
{

namespace srv
{

namespace builder
{

class Init_Test_Response_num2
{
public:
  explicit Init_Test_Response_num2(::carbot_interfaces::srv::Test_Response & msg)
  : msg_(msg)
  {}
  ::carbot_interfaces::srv::Test_Response num2(::carbot_interfaces::srv::Test_Response::_num2_type arg)
  {
    msg_.num2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::carbot_interfaces::srv::Test_Response msg_;
};

class Init_Test_Response_num1
{
public:
  Init_Test_Response_num1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_Response_num2 num1(::carbot_interfaces::srv::Test_Response::_num1_type arg)
  {
    msg_.num1 = std::move(arg);
    return Init_Test_Response_num2(msg_);
  }

private:
  ::carbot_interfaces::srv::Test_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::carbot_interfaces::srv::Test_Response>()
{
  return carbot_interfaces::srv::builder::Init_Test_Response_num1();
}

}  // namespace carbot_interfaces

#endif  // CARBOT_INTERFACES__SRV__DETAIL__TEST__BUILDER_HPP_
