cmake_minimum_required(VERSION 3.5)
project(marker_arm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(nav_msgs)

find_package(OpenCV REQUIRED)


include_directories(include)

include_directories(${OpenCV_INCLUDE_DIRS})


# Build
add_executable(marker_arm_control_node src/marker_arm_control_node.cpp)
ament_target_dependencies(marker_arm_control_node
  dynamixel_sdk
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)

add_executable(marker_arm_perception_node src/marker_arm_perception_node.cpp)
ament_target_dependencies(marker_arm_perception_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nav_msgs
  OpenCV
)

# target_link_libraries(marker_arm_perception_node
#   ${OpenCV_LIBRARIES}
# )


# Install
install(TARGETS
  marker_arm_control_node
  marker_arm_perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf params rviz bt
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
