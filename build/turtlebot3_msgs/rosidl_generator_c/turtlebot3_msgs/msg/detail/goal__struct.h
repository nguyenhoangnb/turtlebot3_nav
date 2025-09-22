// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__STRUCT_H_
#define TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Goal in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__msg__Goal
{
  int32_t pose;
  bool flag;
} turtlebot3_msgs__msg__Goal;

// Struct for a sequence of turtlebot3_msgs__msg__Goal.
typedef struct turtlebot3_msgs__msg__Goal__Sequence
{
  turtlebot3_msgs__msg__Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__msg__Goal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__GOAL__STRUCT_H_
