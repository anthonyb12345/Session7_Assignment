// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_interfaces:srv/LapStartingTime.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__TRAITS_HPP_
#define TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_interfaces/srv/detail/lap_starting_time__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LapStartingTime_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LapStartingTime_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LapStartingTime_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::srv::LapStartingTime_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::srv::LapStartingTime_Request & msg)
{
  return turtlebot3_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::srv::LapStartingTime_Request>()
{
  return "turtlebot3_interfaces::srv::LapStartingTime_Request";
}

template<>
inline const char * name<turtlebot3_interfaces::srv::LapStartingTime_Request>()
{
  return "turtlebot3_interfaces/srv/LapStartingTime_Request";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::srv::LapStartingTime_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::srv::LapStartingTime_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_interfaces::srv::LapStartingTime_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LapStartingTime_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LapStartingTime_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LapStartingTime_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_interfaces::srv::LapStartingTime_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_interfaces::srv::LapStartingTime_Response & msg)
{
  return turtlebot3_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_interfaces::srv::LapStartingTime_Response>()
{
  return "turtlebot3_interfaces::srv::LapStartingTime_Response";
}

template<>
inline const char * name<turtlebot3_interfaces::srv::LapStartingTime_Response>()
{
  return "turtlebot3_interfaces/srv/LapStartingTime_Response";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::srv::LapStartingTime_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_interfaces::srv::LapStartingTime_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_interfaces::srv::LapStartingTime_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_interfaces::srv::LapStartingTime>()
{
  return "turtlebot3_interfaces::srv::LapStartingTime";
}

template<>
inline const char * name<turtlebot3_interfaces::srv::LapStartingTime>()
{
  return "turtlebot3_interfaces/srv/LapStartingTime";
}

template<>
struct has_fixed_size<turtlebot3_interfaces::srv::LapStartingTime>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_interfaces::srv::LapStartingTime_Request>::value &&
    has_fixed_size<turtlebot3_interfaces::srv::LapStartingTime_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_interfaces::srv::LapStartingTime>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_interfaces::srv::LapStartingTime_Request>::value &&
    has_bounded_size<turtlebot3_interfaces::srv::LapStartingTime_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_interfaces::srv::LapStartingTime>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_interfaces::srv::LapStartingTime_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_interfaces::srv::LapStartingTime_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__TRAITS_HPP_
