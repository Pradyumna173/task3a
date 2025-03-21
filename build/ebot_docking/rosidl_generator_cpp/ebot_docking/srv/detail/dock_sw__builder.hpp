// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ebot_docking:srv/DockSw.idl
// generated code does not contain a copyright notice

#ifndef EBOT_DOCKING__SRV__DETAIL__DOCK_SW__BUILDER_HPP_
#define EBOT_DOCKING__SRV__DETAIL__DOCK_SW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ebot_docking/srv/detail/dock_sw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ebot_docking
{

namespace srv
{

namespace builder
{

class Init_DockSw_Request_target
{
public:
  Init_DockSw_Request_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ebot_docking::srv::DockSw_Request target(::ebot_docking::srv::DockSw_Request::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ebot_docking::srv::DockSw_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebot_docking::srv::DockSw_Request>()
{
  return ebot_docking::srv::builder::Init_DockSw_Request_target();
}

}  // namespace ebot_docking


namespace ebot_docking
{

namespace srv
{

namespace builder
{

class Init_DockSw_Response_success
{
public:
  Init_DockSw_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ebot_docking::srv::DockSw_Response success(::ebot_docking::srv::DockSw_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ebot_docking::srv::DockSw_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebot_docking::srv::DockSw_Response>()
{
  return ebot_docking::srv::builder::Init_DockSw_Response_success();
}

}  // namespace ebot_docking

#endif  // EBOT_DOCKING__SRV__DETAIL__DOCK_SW__BUILDER_HPP_
