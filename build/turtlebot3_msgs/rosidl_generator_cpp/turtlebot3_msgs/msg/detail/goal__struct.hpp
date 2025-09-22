// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__STRUCT_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__msg__Goal __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__msg__Goal __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Goal_
{
  using Type = Goal_<ContainerAllocator>;

  explicit Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pose = 0l;
      this->flag = false;
    }
  }

  explicit Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pose = 0l;
      this->flag = false;
    }
  }

  // field types and members
  using _pose_type =
    int32_t;
  _pose_type pose;
  using _flag_type =
    bool;
  _flag_type flag;

  // setters for named parameter idiom
  Type & set__pose(
    const int32_t & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__flag(
    const bool & _arg)
  {
    this->flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::msg::Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::msg::Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::msg::Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::msg::Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__msg__Goal
    std::shared_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__msg__Goal
    std::shared_ptr<turtlebot3_msgs::msg::Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Goal_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->flag != other.flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Goal_

// alias to use template instance with default allocator
using Goal =
  turtlebot3_msgs::msg::Goal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__STRUCT_HPP_
