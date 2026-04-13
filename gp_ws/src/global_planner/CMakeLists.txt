cmake_minimum_required(VERSION 3.8)
project(global_planner)

# ── Compiler settings ──────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Find dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ── gps_nav_node ───────────────────────────────────────────────────────────────
add_executable(gps_nav_node
  src/gps_nav_node.cpp
)
ament_target_dependencies(gps_nav_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# ── goal_publisher_node ────────────────────────────────────────────────────────
add_executable(goal_publisher_node
  src/goal_publisher_node.cpp
)
ament_target_dependencies(goal_publisher_node
  rclcpp
  sensor_msgs
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  gps_nav_node
  goal_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Lint (optional) ────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
