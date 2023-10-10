// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from carbot_interfaces:srv/Test.idl
// generated code does not contain a copyright notice

#ifndef CARBOT_INTERFACES__SRV__DETAIL__TEST__STRUCT_HPP_
#define CARBOT_INTERFACES__SRV__DETAIL__TEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__carbot_interfaces__srv__Test_Request __attribute__((deprecated))
#else
# define DEPRECATED__carbot_interfaces__srv__Test_Request __declspec(deprecated)
#endif

namespace carbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Test_Request_
{
  using Type = Test_Request_<ContainerAllocator>;

  explicit Test_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->get_num = false;
    }
  }

  explicit Test_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->get_num = false;
    }
  }

  // field types and members
  using _get_num_type =
    bool;
  _get_num_type get_num;

  // setters for named parameter idiom
  Type & set__get_num(
    const bool & _arg)
  {
    this->get_num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    carbot_interfaces::srv::Test_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const carbot_interfaces::srv::Test_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      carbot_interfaces::srv::Test_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      carbot_interfaces::srv::Test_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__carbot_interfaces__srv__Test_Request
    std::shared_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__carbot_interfaces__srv__Test_Request
    std::shared_ptr<carbot_interfaces::srv::Test_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Test_Request_ & other) const
  {
    if (this->get_num != other.get_num) {
      return false;
    }
    return true;
  }
  bool operator!=(const Test_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Test_Request_

// alias to use template instance with default allocator
using Test_Request =
  carbot_interfaces::srv::Test_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace carbot_interfaces


#ifndef _WIN32
# define DEPRECATED__carbot_interfaces__srv__Test_Response __attribute__((deprecated))
#else
# define DEPRECATED__carbot_interfaces__srv__Test_Response __declspec(deprecated)
#endif

namespace carbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Test_Response_
{
  using Type = Test_Response_<ContainerAllocator>;

  explicit Test_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num1 = 0.0;
      this->num2 = 0.0;
    }
  }

  explicit Test_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num1 = 0.0;
      this->num2 = 0.0;
    }
  }

  // field types and members
  using _num1_type =
    double;
  _num1_type num1;
  using _num2_type =
    double;
  _num2_type num2;

  // setters for named parameter idiom
  Type & set__num1(
    const double & _arg)
  {
    this->num1 = _arg;
    return *this;
  }
  Type & set__num2(
    const double & _arg)
  {
    this->num2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    carbot_interfaces::srv::Test_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const carbot_interfaces::srv::Test_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      carbot_interfaces::srv::Test_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      carbot_interfaces::srv::Test_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__carbot_interfaces__srv__Test_Response
    std::shared_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__carbot_interfaces__srv__Test_Response
    std::shared_ptr<carbot_interfaces::srv::Test_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Test_Response_ & other) const
  {
    if (this->num1 != other.num1) {
      return false;
    }
    if (this->num2 != other.num2) {
      return false;
    }
    return true;
  }
  bool operator!=(const Test_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Test_Response_

// alias to use template instance with default allocator
using Test_Response =
  carbot_interfaces::srv::Test_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace carbot_interfaces

namespace carbot_interfaces
{

namespace srv
{

struct Test
{
  using Request = carbot_interfaces::srv::Test_Request;
  using Response = carbot_interfaces::srv::Test_Response;
};

}  // namespace srv

}  // namespace carbot_interfaces

#endif  // CARBOT_INTERFACES__SRV__DETAIL__TEST__STRUCT_HPP_
