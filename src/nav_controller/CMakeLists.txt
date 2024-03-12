cmake_minimum_required(VERSION 3.8)
project(nav_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_module_interfaces REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

# Install all scripts
install(PROGRAMS
  scripts/nav_node.py
  scripts/show_commands.py
  scripts/clicked_point_listener.py
  scripts/add_waypoint.py
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# install the resources directory
install(DIRECTORY
  nav_controller/resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
