cmake_minimum_required(VERSION 3.8)
project(activity_control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

add_library(mission_core SHARED
  src/core/aux_controller.cpp
  src/core/image_cache.cpp
  src/core/sensor_cache.cpp
  src/core/waypoint_navigator.cpp
)
target_include_directories(mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mission_core
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
if(MSVC)
  target_compile_options(mission_core PRIVATE /W4)
else()
  target_compile_options(mission_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(fixed_waypoint_task_node
  src/tasks/fixed_waypoint_task_node.cpp
)
target_link_libraries(fixed_waypoint_task_node mission_core)
ament_target_dependencies(fixed_waypoint_task_node
  rclcpp
  std_msgs
)
if(MSVC)
  target_compile_options(fixed_waypoint_task_node PRIVATE /W4)
else()
  target_compile_options(fixed_waypoint_task_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS mission_core
  EXPORT export_mission_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  fixed_waypoint_task_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
ament_export_include_directories(include)
ament_export_targets(export_mission_core HAS_LIBRARY_TARGET)

ament_package()
