// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
#define TURTLEBOT3_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_interfaces/srv/detail/find_closest_wall__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_interfaces::srv::FindClosestWall_Request>()
{
  return ::turtlebot3_interfaces::srv::FindClosestWall_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace turtlebot3_interfaces


namespace turtlebot3_interfaces
{

namespace srv
{

namespace builder
{

class Init_FindClosestWall_Response_success
{
public:
  Init_FindClosestWall_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_interfaces::srv::FindClosestWall_Response success(::turtlebot3_interfaces::srv::FindClosestWall_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_interfaces::srv::FindClosestWall_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_interfaces::srv::FindClosestWall_Response>()
{
  return turtlebot3_interfaces::srv::builder::Init_FindClosestWall_Response_success();
}

}  // namespace turtlebot3_interfaces

#endif  // TURTLEBOT3_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
