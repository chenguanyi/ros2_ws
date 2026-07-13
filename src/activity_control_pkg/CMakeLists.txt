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
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

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

add_executable(wildlife_patrol_task_node
  src/tasks/wildlife_patrol_task_node.cpp
)
target_link_libraries(wildlife_patrol_task_node mission_core)
ament_target_dependencies(wildlife_patrol_task_node
  rclcpp
  std_msgs
)
if(MSVC)
  target_compile_options(wildlife_patrol_task_node PRIVATE /W4)
else()
  target_compile_options(wildlife_patrol_task_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(diansai_first_task_node
  src/tasks/diansai_first/diansai_first_task_node.cpp
)
target_include_directories(diansai_first_task_node PRIVATE
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(diansai_first_task_node
  mission_core
  ${OpenCV_LIBS}
  ${JSONCPP_LIBRARIES}
)
ament_target_dependencies(diansai_first_task_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)
if(MSVC)
  target_compile_options(diansai_first_task_node PRIVATE /W4)
else()
  target_compile_options(diansai_first_task_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(plant_protection_task_node
  src/tasks/plant_protection_task_node.cpp
)
target_link_libraries(plant_protection_task_node mission_core)
ament_target_dependencies(plant_protection_task_node
  rclcpp
  std_msgs
  sensor_msgs
)
if(MSVC)
  target_compile_options(plant_protection_task_node PRIVATE /W4)
else()
  target_compile_options(plant_protection_task_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(warehouse_inventory_task_node
  src/tasks/warehouse_inventory_task_node.cpp
)
target_include_directories(warehouse_inventory_task_node PRIVATE
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(warehouse_inventory_task_node
  mission_core
  ${JSONCPP_LIBRARIES}
)
ament_target_dependencies(warehouse_inventory_task_node
  rclcpp
  std_msgs
)
if(MSVC)
  target_compile_options(warehouse_inventory_task_node PRIVATE /W4)
else()
  target_compile_options(warehouse_inventory_task_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(barcode_reader_node
  src/tasks/barcode_reader_node.cpp
)
target_include_directories(barcode_reader_node PRIVATE
  ${ZBAR_INCLUDE_DIRS}
)
target_link_libraries(barcode_reader_node
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES}
)
ament_target_dependencies(barcode_reader_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)
if(MSVC)
  target_compile_options(barcode_reader_node PRIVATE /W4)
else()
  target_compile_options(barcode_reader_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(side_camera_viewer_node
  src/tasks/side_camera_viewer_node.cpp
)
target_link_libraries(side_camera_viewer_node
  ${OpenCV_LIBS}
)
ament_target_dependencies(side_camera_viewer_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)
if(MSVC)
  target_compile_options(side_camera_viewer_node PRIVATE /W4)
else()
  target_compile_options(side_camera_viewer_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS mission_core
  EXPORT export_mission_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  fixed_waypoint_task_node
  wildlife_patrol_task_node
  diansai_first_task_node
  plant_protection_task_node
  warehouse_inventory_task_node
  barcode_reader_node
  side_camera_viewer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/diansai_first/alignment_check_support.py
  scripts/diansai_first/relay_http_bridge.py
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
