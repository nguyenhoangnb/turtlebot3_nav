// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__TRAITS_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_msgs/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    rosidl_generator_traits::value_to_yaml(msg.pose, out);
    out << ", ";
  }

  // member: flag
  {
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose: ";
    rosidl_generator_traits::value_to_yaml(msg.pose, out);
    out << "\n";
  }

  // member: flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace turtlebot3_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_msgs::msg::Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_msgs::msg::Goal & msg)
{
  return turtlebot3_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_msgs::msg::Goal>()
{
  return "turtlebot3_msgs::msg::Goal";
}

template<>
inline const char * name<turtlebot3_msgs::msg::Goal>()
{
  return "turtlebot3_msgs/msg/Goal";
}

template<>
struct has_fixed_size<turtlebot3_msgs::msg::Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_msgs::msg::Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_msgs::msg::Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__TRAITS_HPP_
