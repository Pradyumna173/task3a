// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ebot_docking:srv/PassingService.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__BUILDER_HPP_
#define EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ebot_docking/srv/detail/passing_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ebot_docking
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebot_docking::srv::PassingService_Request>()
{
  return ::ebot_docking::srv::PassingService_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ebot_docking


namespace ebot_docking
{

namespace srv
{

namespace builder
{

class Init_PassingService_Response_conveyer
{
public:
  Init_PassingService_Response_conveyer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ebot_docking::srv::PassingService_Response conveyer(::ebot_docking::srv::PassingService_Response::_conveyer_type arg)
  {
    msg_.conveyer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ebot_docking::srv::PassingService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebot_docking::srv::PassingService_Response>()
{
  return ebot_docking::srv::builder::Init_PassingService_Response_conveyer();
}

}  // namespace ebot_docking

#endif  // EBOT_DOCKING__SRV__DETAIL__PASSING_SERVICE__BUILDER_HPP_
