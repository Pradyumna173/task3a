// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ebot_docking:srv/PassingService.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__STRUCT_HPP_
#define EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ebot_docking__srv__PassingService_Request __attribute__((deprecated))
#else
# define DEPRECATED__ebot_docking__srv__PassingService_Request __declspec(deprecated)
#endif

namespace ebot_docking
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PassingService_Request_
{
  using Type = PassingService_Request_<ContainerAllocator>;

  explicit PassingService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit PassingService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    ebot_docking::srv::PassingService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebot_docking::srv::PassingService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::PassingService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::PassingService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebot_docking__srv__PassingService_Request
    std::shared_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebot_docking__srv__PassingService_Request
    std::shared_ptr<ebot_docking::srv::PassingService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PassingService_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const PassingService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PassingService_Request_

// alias to use template instance with default allocator
using PassingService_Request =
  ebot_docking::srv::PassingService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ebot_docking


#ifndef _WIN32
# define DEPRECATED__ebot_docking__srv__PassingService_Response __attribute__((deprecated))
#else
# define DEPRECATED__ebot_docking__srv__PassingService_Response __declspec(deprecated)
#endif

namespace ebot_docking
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PassingService_Response_
{
  using Type = PassingService_Response_<ContainerAllocator>;

  explicit PassingService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->conveyer = 0l;
    }
  }

  explicit PassingService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->conveyer = 0l;
    }
  }

  // field types and members
  using _conveyer_type =
    int32_t;
  _conveyer_type conveyer;

  // setters for named parameter idiom
  Type & set__conveyer(
    const int32_t & _arg)
  {
    this->conveyer = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ebot_docking::srv::PassingService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebot_docking::srv::PassingService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::PassingService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::PassingService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebot_docking__srv__PassingService_Response
    std::shared_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebot_docking__srv__PassingService_Response
    std::shared_ptr<ebot_docking::srv::PassingService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PassingService_Response_ & other) const
  {
    if (this->conveyer != other.conveyer) {
      return false;
    }
    return true;
  }
  bool operator!=(const PassingService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PassingService_Response_

// alias to use template instance with default allocator
using PassingService_Response =
  ebot_docking::srv::PassingService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ebot_docking

namespace ebot_docking
{

namespace srv
{

struct PassingService
{
  using Request = ebot_docking::srv::PassingService_Request;
  using Response = ebot_docking::srv::PassingService_Response;
};

}  // namespace srv

}  // namespace ebot_docking

#endif  // EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__STRUCT_HPP_
