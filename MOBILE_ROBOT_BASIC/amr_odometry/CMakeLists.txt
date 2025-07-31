cmake_minimum_required(VERSION 3.8)
project(amr_odometry)

add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Common dependency variable
set(COMMON_DEPENDENCIES
  rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs geometry_msgs sensor_msgs
)

# All sources for the shared library
set(AMR_ODOMETRY_SRCS
  src/odometry_publisher.cpp
  # src/odometry.cpp  # Add more source files here if needed
)

# Create a single shared library
add_library(amr_odometry_lib SHARED ${AMR_ODOMETRY_SRCS})
ament_target_dependencies(amr_odometry_lib ${COMMON_DEPENDENCIES})
target_link_libraries(amr_odometry_lib pthread)

# Executable
add_executable(amr_odometry src/odometry_publisher.cpp)
target_link_libraries(amr_odometry amr_odometry_lib pthread)
ament_target_dependencies(amr_odometry ${COMMON_DEPENDENCIES})

# Install targets
install(TARGETS amr_odometry amr_odometry_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY param launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
