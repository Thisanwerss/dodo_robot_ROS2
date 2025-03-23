cmake_minimum_required(VERSION 3.8)
project(dodo_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dodo_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# State Manager Node
add_executable(state_manager_node
  src/state_manager_node.cpp
  src/robot_state_machine.cpp
)

# Parameter Debugging Node
add_executable(parameter_debugging_node
  src/parameter_debugging_node.cpp
)

# Include directories
include_directories(include)

# Specify libraries to link executable targets against
target_link_libraries(state_manager_node
  nlohmann_json::nlohmann_json
)

target_link_libraries(parameter_debugging_node
  nlohmann_json::nlohmann_json
)

# Dependencies
ament_target_dependencies(state_manager_node
  rclcpp
  std_msgs
  sensor_msgs
  dodo_msgs
)

ament_target_dependencies(parameter_debugging_node
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  state_manager_node
  parameter_debugging_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()