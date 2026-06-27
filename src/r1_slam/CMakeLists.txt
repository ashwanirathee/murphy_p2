cmake_minimum_required(VERSION 3.8)
project(r1_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(monocular_slam_node
  src/monocular_slam_node.cpp
)

target_include_directories(monocular_slam_node
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(monocular_slam_node
  cv_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(monocular_slam_node
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS monocular_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
