// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_localization:srv/FromLL.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_localization/srv/detail/from_ll__rosidl_typesupport_introspection_c.h"
#include "robot_localization/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_localization/srv/detail/from_ll__functions.h"
#include "robot_localization/srv/detail/from_ll__struct.h"


// Include directives for member types
// Member `ll_point`
#include "geographic_msgs/msg/geo_point.h"
// Member `ll_point`
#include "geographic_msgs/msg/detail/geo_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_localization__srv__FromLL_Request__init(message_memory);
}

void FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_fini_function(void * message_memory)
{
  robot_localization__srv__FromLL_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_member_array[1] = {
  {
    "ll_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_localization__srv__FromLL_Request, ll_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_members = {
  "robot_localization__srv",  // message namespace
  "FromLL_Request",  // message name
  1,  // number of fields
  sizeof(robot_localization__srv__FromLL_Request),
  FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_member_array,  // message members
  FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_type_support_handle = {
  0,
  &FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_localization
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_localization, srv, FromLL_Request)() {
  FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geographic_msgs, msg, GeoPoint)();
  if (!FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_type_support_handle.typesupport_identifier) {
    FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FromLL_Request__rosidl_typesupport_introspection_c__FromLL_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_localization/srv/detail/from_ll__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_localization/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_localization/srv/detail/from_ll__functions.h"
// already included above
// #include "robot_localization/srv/detail/from_ll__struct.h"


// Include directives for member types
// Member `map_point`
#include "geometry_msgs/msg/point.h"
// Member `map_point`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_localization__srv__FromLL_Response__init(message_memory);
}

void FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_fini_function(void * message_memory)
{
  robot_localization__srv__FromLL_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_member_array[1] = {
  {
    "map_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_localization__srv__FromLL_Response, map_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_members = {
  "robot_localization__srv",  // message namespace
  "FromLL_Response",  // message name
  1,  // number of fields
  sizeof(robot_localization__srv__FromLL_Response),
  FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_member_array,  // message members
  FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_type_support_handle = {
  0,
  &FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_localization
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_localization, srv, FromLL_Response)() {
  FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_type_support_handle.typesupport_identifier) {
    FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FromLL_Response__rosidl_typesupport_introspection_c__FromLL_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_localization/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_localization/srv/detail/from_ll__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_service_members = {
  "robot_localization__srv",  // service namespace
  "FromLL",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_Request_message_type_support_handle,
  NULL  // response message
  // robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_Response_message_type_support_handle
};

static rosidl_service_type_support_t robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_service_type_support_handle = {
  0,
  &robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_localization, srv, FromLL_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_localization, srv, FromLL_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_localization
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_localization, srv, FromLL)() {
  if (!robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_service_type_support_handle.typesupport_identifier) {
    robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_localization, srv, FromLL_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_localization, srv, FromLL_Response)()->data;
  }

  return &robot_localization__srv__detail__from_ll__rosidl_typesupport_introspection_c__FromLL_service_type_support_handle;
}
