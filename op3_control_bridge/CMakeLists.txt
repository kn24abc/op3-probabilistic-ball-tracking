cmake_minimum_required(VERSION 3.8)
project(op3_control_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(op3_vision_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(game_controller_hl_interfaces REQUIRED)
find_package(op3_head_scan REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# cmd_vel_echo
add_executable(cmd_vel_echo src/cmd_vel_echo.cpp)

ament_target_dependencies(cmd_vel_echo 
  rclcpp
  geometry_msgs
)

# cmd_vel_node
add_executable(cmd_vel_node src/cmd_vel_to_walk_param.cpp)

ament_target_dependencies(cmd_vel_node 
  rclcpp
  geometry_msgs
  std_msgs
  op3_walking_module_msgs
)

# fall_detector_node
add_executable(fall_detector_node src/fall_detection.cpp)

ament_target_dependencies(fall_detector_node
  rclcpp
  sensor_msgs
  robotis_math
)

target_include_directories(fall_detector_node PUBLIC
  ${robotis_math_INCLUDE_DIRS}
)

# controller_manager_node
add_executable(control_bridge_node
  src/control_bridge_node.cpp
  src/ball_tracker.cpp
  src/ball_follower.cpp
  src/ball_dribbler.cpp
  src/ball_searcher.cpp
  src/gamestate_listener.cpp
  src/goal_bearing_tracker.cpp
)

ament_target_dependencies(control_bridge_node
  rclcpp
  yaml-cpp
  ament_index_cpp
  std_msgs
  geometry_msgs
  robotis_controller_msgs
  op3_walking_module_msgs
  op3_action_module_msgs
  robotis_math
  op3_vision_msgs
  game_controller_hl_interfaces
  op3_head_scan
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(control_bridge_node
  yaml-cpp
  op3_head_scan::head_scan
)

target_include_directories(control_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${robotis_math_INCLUDE_DIRS}
)

install(TARGETS
  cmd_vel_echo
  cmd_vel_node
  fall_detector_node
  control_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
