// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_interfaces:action/MeasureLapTime.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__TRAITS_HPP_
#define TURTLEBOT3_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_interfaces/action/detail/measure_lap_time__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_Goal & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_Goal & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_Goal>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_Goal";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_Goal>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_Goal";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: total_time
  {
    out << "total_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: total_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_Result & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_Result>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_Result";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_Result>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_Result";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: elapsed_time
  {
    out << "elapsed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: elapsed_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elapsed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_Feedback & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_Feedback>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_Feedback";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_Feedback>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_Feedback";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "turtlebot3_interfaces/action/detail/measure_lap_time__traits.hpp"

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_SendGoal_Request";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_SendGoal_Response";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_SendGoal>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_SendGoal";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_SendGoal>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_SendGoal";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>::value &&
    has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>::value &&
    has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_interfaces::action::MeasureLapTime_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_interfaces::action::MeasureLapTime_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_GetResult_Request";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "turtlebot3_interfaces/action/detail/measure_lap_time__traits.hpp"

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_GetResult_Response";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_Result>::value> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_Result>::value> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_GetResult>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_GetResult";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_GetResult>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_GetResult";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>::value &&
    has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>::value &&
    has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_interfaces::action::MeasureLapTime_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_interfaces::action::MeasureLapTime_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "turtlebot3_interfaces/action/detail/measure_lap_time__traits.hpp"

namespace turtlebot3_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const MeasureLapTime_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasureLapTime_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasureLapTime_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage & msg)
{
  return turtlebot3_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage>()
{
  return "turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage";
}

template<>
inline const char * name<turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage>()
{
  return "turtlebot3_interfaces/action/MeasureLapTime_FeedbackMessage";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<turtlebot3_interfaces::action::MeasureLapTime_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<turtlebot3_interfaces::action::MeasureLapTime_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<turtlebot3_interfaces::action::MeasureLapTime_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<turtlebot3_interfaces::action::MeasureLapTime>
  : std::true_type
{
};

template<>
struct is_action_goal<turtlebot3_interfaces::action::MeasureLapTime_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<turtlebot3_interfaces::action::MeasureLapTime_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<turtlebot3_interfaces::action::MeasureLapTime_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // TURTLEBOT3_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__TRAITS_HPP_
