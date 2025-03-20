// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from usb_servo:srv/ServoSw.idl
// generated code does not contain a copyright notice

#ifndef USB_SERVO__SRV__DETAIL__SERVO_SW__STRUCT_HPP_
#define USB_SERVO__SRV__DETAIL__SERVO_SW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__usb_servo__srv__ServoSw_Request __attribute__((deprecated))
#else
# define DEPRECATED__usb_servo__srv__ServoSw_Request __declspec(deprecated)
#endif

namespace usb_servo
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ServoSw_Request_
{
  using Type = ServoSw_Request_<ContainerAllocator>;

  explicit ServoSw_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servostate = false;
    }
  }

  explicit ServoSw_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servostate = false;
    }
  }

  // field types and members
  using _servostate_type =
    bool;
  _servostate_type servostate;

  // setters for named parameter idiom
  Type & set__servostate(
    const bool & _arg)
  {
    this->servostate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    usb_servo::srv::ServoSw_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const usb_servo::srv::ServoSw_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      usb_servo::srv::ServoSw_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      usb_servo::srv::ServoSw_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__usb_servo__srv__ServoSw_Request
    std::shared_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__usb_servo__srv__ServoSw_Request
    std::shared_ptr<usb_servo::srv::ServoSw_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoSw_Request_ & other) const
  {
    if (this->servostate != other.servostate) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoSw_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoSw_Request_

// alias to use template instance with default allocator
using ServoSw_Request =
  usb_servo::srv::ServoSw_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace usb_servo


#ifndef _WIN32
# define DEPRECATED__usb_servo__srv__ServoSw_Response __attribute__((deprecated))
#else
# define DEPRECATED__usb_servo__srv__ServoSw_Response __declspec(deprecated)
#endif

namespace usb_servo
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ServoSw_Response_
{
  using Type = ServoSw_Response_<ContainerAllocator>;

  explicit ServoSw_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0;
    }
  }

  explicit ServoSw_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0;
    }
  }

  // field types and members
  using _num_type =
    int16_t;
  _num_type num;

  // setters for named parameter idiom
  Type & set__num(
    const int16_t & _arg)
  {
    this->num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    usb_servo::srv::ServoSw_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const usb_servo::srv::ServoSw_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      usb_servo::srv::ServoSw_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      usb_servo::srv::ServoSw_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__usb_servo__srv__ServoSw_Response
    std::shared_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__usb_servo__srv__ServoSw_Response
    std::shared_ptr<usb_servo::srv::ServoSw_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoSw_Response_ & other) const
  {
    if (this->num != other.num) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoSw_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoSw_Response_

// alias to use template instance with default allocator
using ServoSw_Response =
  usb_servo::srv::ServoSw_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace usb_servo

namespace usb_servo
{

namespace srv
{

struct ServoSw
{
  using Request = usb_servo::srv::ServoSw_Request;
  using Response = usb_servo::srv::ServoSw_Response;
};

}  // namespace srv

}  // namespace usb_servo

#endif  // USB_SERVO__SRV__DETAIL__SERVO_SW__STRUCT_HPP_
