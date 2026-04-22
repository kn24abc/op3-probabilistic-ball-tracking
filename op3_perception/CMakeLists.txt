cmake_minimum_required(VERSION 3.8)
project(op3_perception)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(op3_vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ball_particle_filter
    test/test_ball_particle_filter.cpp)
endif()

add_library(op3_perception_bearing_transformer src/bearing_transformer.cpp)

target_include_directories(op3_perception_bearing_transformer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(op3_perception_bearing_transformer
  rclcpp
  geometry_msgs
  op3_vision_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(ball_filter_node src/ball_filter_node.cpp)

ament_target_dependencies(ball_filter_node
  rclcpp
  vision_msgs
  geometry_msgs
  op3_vision_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(ball_filter_node
  op3_perception_bearing_transformer
)

add_executable(goalpost_filter_node src/goalpost_filter_node.cpp)

ament_target_dependencies(goalpost_filter_node
  rclcpp
  op3_vision_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(goalpost_filter_node
  op3_perception_bearing_transformer
)

add_executable(ball_particle_filter_node src/ball_particle_filter_node.cpp)

ament_target_dependencies(ball_particle_filter_node
  rclcpp
  geometry_msgs
  op3_vision_msgs
  op3_walking_module_msgs
  sensor_msgs
)

add_executable(ball_reacquisition_logger_node src/ball_reacquisition_logger_node.cpp)

ament_target_dependencies(ball_reacquisition_logger_node
  rclcpp
  std_msgs
)

add_executable(vision_overlay_node src/vision_overlay_node.cpp)

ament_target_dependencies(vision_overlay_node
  rclcpp
  sensor_msgs
  op3_vision_msgs
)

target_link_libraries(vision_overlay_node
  ${OpenCV_LIBS}
)

install(TARGETS
  op3_perception_bearing_transformer
  ball_filter_node
  ball_particle_filter_node
  ball_reacquisition_logger_node
  goalpost_filter_node
  vision_overlay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(op3_perception_bearing_transformer)

ament_package()
