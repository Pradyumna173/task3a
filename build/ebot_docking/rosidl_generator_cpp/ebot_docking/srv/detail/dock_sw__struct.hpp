// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ebot_docking:srv/DockSw.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__DOCK_SW__STRUCT_HPP_
#define EBOT_DOCKING__SRV__DETAIL__DOCK_SW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ebot_docking__srv__DockSw_Request __attribute__((deprecated))
#else
# define DEPRECATED__ebot_docking__srv__DockSw_Request __declspec(deprecated)
#endif

namespace ebot_docking
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DockSw_Request_
{
  using Type = DockSw_Request_<ContainerAllocator>;

  explicit DockSw_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = 0;
    }
  }

  explicit DockSw_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = 0;
    }
  }

  // field types and members
  using _target_type =
    uint8_t;
  _target_type target;

  // setters for named parameter idiom
  Type & set__target(
    const uint8_t & _arg)
  {
    this->target = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ebot_docking::srv::DockSw_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebot_docking::srv::DockSw_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::DockSw_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::DockSw_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebot_docking__srv__DockSw_Request
    std::shared_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebot_docking__srv__DockSw_Request
    std::shared_ptr<ebot_docking::srv::DockSw_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DockSw_Request_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    return true;
  }
  bool operator!=(const DockSw_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DockSw_Request_

// alias to use template instance with default allocator
using DockSw_Request =
  ebot_docking::srv::DockSw_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ebot_docking


#ifndef _WIN32
# define DEPRECATED__ebot_docking__srv__DockSw_Response __attribute__((deprecated))
#else
# define DEPRECATED__ebot_docking__srv__DockSw_Response __declspec(deprecated)
#endif

namespace ebot_docking
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DockSw_Response_
{
  using Type = DockSw_Response_<ContainerAllocator>;

  explicit DockSw_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit DockSw_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ebot_docking::srv::DockSw_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebot_docking::srv::DockSw_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::DockSw_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebot_docking::srv::DockSw_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebot_docking__srv__DockSw_Response
    std::shared_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebot_docking__srv__DockSw_Response
    std::shared_ptr<ebot_docking::srv::DockSw_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DockSw_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const DockSw_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DockSw_Response_

// alias to use template instance with default allocator
using DockSw_Response =
  ebot_docking::srv::DockSw_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ebot_docking

namespace ebot_docking
{

namespace srv
{

struct DockSw
{
  using Request = ebot_docking::srv::DockSw_Request;
  using Response = ebot_docking::srv::DockSw_Response;
};

}  // namespace srv

}  // namespace ebot_docking

#endif  // EBOT_DOCKING__SRV__DETAIL__DOCK_SW__STRUCT_HPP_
