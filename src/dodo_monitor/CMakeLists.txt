cmake_minimum_required(VERSION 3.8)
project(dodo_monitor)

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
find_package(diagnostic_msgs REQUIRED)
find_package(dodo_msgs REQUIRED)

# Include our own directories
include_directories(include)

# Standard executable (monitor_node)
add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node rclcpp diagnostic_msgs dodo_msgs)

# Install executables
install(TARGETS
  monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()