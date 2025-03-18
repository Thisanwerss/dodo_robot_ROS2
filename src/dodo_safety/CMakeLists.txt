cmake_minimum_required(VERSION 3.8)
project(dodo_safety)

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
find_package(dodo_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node rclcpp std_msgs dodo_msgs)

# Install executables
install(TARGETS
  safety_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()