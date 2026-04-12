cmake_minimum_required(VERSION 3.8)
project(cashier_system)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bill.msg"
  "srv/GetStatus.srv"
)

# Executables
add_executable(bill_generator src/bill_generator.cpp)
add_executable(inventory_manager src/inventory_manager.cpp)
add_executable(status_viewer src/status_viewer.cpp)

ament_target_dependencies(bill_generator rclcpp)
ament_target_dependencies(inventory_manager rclcpp)
ament_target_dependencies(status_viewer rclcpp)

# Link interfaces to executables
rosidl_target_interfaces(bill_generator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(inventory_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(status_viewer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  bill_generator
  inventory_manager
  status_viewer
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
