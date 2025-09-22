// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_msgs/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_msgs
{

namespace msg
{

namespace builder
{

class Init_Goal_flag
{
public:
  explicit Init_Goal_flag(::turtlebot3_msgs::msg::Goal & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::msg::Goal flag(::turtlebot3_msgs::msg::Goal::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::msg::Goal msg_;
};

class Init_Goal_pose
{
public:
  Init_Goal_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Goal_flag pose(::turtlebot3_msgs::msg::Goal::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Goal_flag(msg_);
  }

private:
  ::turtlebot3_msgs::msg::Goal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::msg::Goal>()
{
  return turtlebot3_msgs::msg::builder::Init_Goal_pose();
}

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_
