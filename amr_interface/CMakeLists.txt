cmake_minimum_required(VERSION 3.8)
project(amr_interface)

add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(san_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# Common dependency variable
set(COMMON_DEPENDENCIES
  rclcpp rclcpp_action std_msgs sensor_msgs nav_msgs nav2_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs std_srvs san_msgs yaml-cpp visualization_msgs
)

# All sources for the shared library
set(AMR_INTERFACE_SRCS
  src/interface.cpp
  src/path_status_manager.cpp
  src/execution.cpp
  src/parameter_updater.cpp
  src/map_manager.cpp
  src/node_manager.cpp
)

# Create a single shared library
add_library(amr_interface_lib SHARED ${AMR_INTERFACE_SRCS})
ament_target_dependencies(amr_interface_lib ${COMMON_DEPENDENCIES})

# Executable
add_executable(interface_node src/node/interface_node.cpp)
target_link_libraries(interface_node amr_interface_lib yaml-cpp)
ament_target_dependencies(interface_node ${COMMON_DEPENDENCIES})

# Install targets
install(TARGETS interface_node amr_interface_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY param launch
  DESTINATION share/${PROJECT_NAME})

ament_package()