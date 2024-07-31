// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_interfaces:srv/LapStartingTime.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__STRUCT_H_
#define TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/LapStartingTime in the package turtlebot3_interfaces.
typedef struct turtlebot3_interfaces__srv__LapStartingTime_Request
{
  uint8_t structure_needs_at_least_one_member;
} turtlebot3_interfaces__srv__LapStartingTime_Request;

// Struct for a sequence of turtlebot3_interfaces__srv__LapStartingTime_Request.
typedef struct turtlebot3_interfaces__srv__LapStartingTime_Request__Sequence
{
  turtlebot3_interfaces__srv__LapStartingTime_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_interfaces__srv__LapStartingTime_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/LapStartingTime in the package turtlebot3_interfaces.
typedef struct turtlebot3_interfaces__srv__LapStartingTime_Response
{
  bool success;
} turtlebot3_interfaces__srv__LapStartingTime_Response;

// Struct for a sequence of turtlebot3_interfaces__srv__LapStartingTime_Response.
typedef struct turtlebot3_interfaces__srv__LapStartingTime_Response__Sequence
{
  turtlebot3_interfaces__srv__LapStartingTime_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_interfaces__srv__LapStartingTime_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__STRUCT_H_
