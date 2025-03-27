cmake_minimum_required(VERSION 3.8)
project(dodo_monitor)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Enable Qt Meta Object Compiler (MOC) ===
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# === Find Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dodo_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# === Include directories ===
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# === Trigger MOC manually for monitor_gui.hpp ===
qt5_wrap_cpp(MOC_SOURCES
  include/dodo_monitor/monitor_gui.hpp
)

# === monitor_node executable ===
add_executable(monitor_node
  src/monitor_node.cpp
)

ament_target_dependencies(monitor_node
  rclcpp
  diagnostic_msgs
  dodo_msgs
)

# === gui_node executable ===
add_executable(gui_node
  src/gui_node.cpp
  src/monitor_gui.cpp
  src/monitor_core.cpp
  ${MOC_SOURCES}
)

target_link_libraries(gui_node
  Qt5::Widgets
)

ament_target_dependencies(gui_node
  rclcpp
  diagnostic_msgs
  dodo_msgs
)

# === Install executables ===
install(TARGETS
  monitor_node
  gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Install other project files ===
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

