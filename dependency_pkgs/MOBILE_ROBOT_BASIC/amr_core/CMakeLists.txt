cmake_minimum_required(VERSION 3.8)
project(amr_core)

add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(std_srvs REQUIRED)
find_package(san_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(amr_serial REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Common dependency variable
set(COMMON_DEPENDENCIES
  rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs
  tf2 tf2_geometry_msgs tf2_ros diagnostic_msgs can_msgs
  san_msgs amr_serial
)

# All sources for the shared library
set(AMR_CORE_SRCS
  src/amr_core.cpp
  src/velocity_profile.cpp
  src/ekf_odometry.cpp
  src/can.cpp
  src/kinematics.cpp
  src/mode_manager.cpp
)

# Create a single shared library
add_library(amr_core_lib SHARED ${AMR_CORE_SRCS})
ament_target_dependencies(amr_core_lib ${COMMON_DEPENDENCIES})
target_link_libraries(amr_core_lib pthread Boost::thread ${EIGEN3_LIBRARIES})

# Executables
add_executable(${PROJECT_NAME}_node src/amr_core.cpp src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node amr_core_lib pthread Boost::thread ${EIGEN3_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_node ${COMMON_DEPENDENCIES})

add_executable(teleop_keyboard src/teleop_keyboard.cpp)
target_link_libraries(teleop_keyboard amr_core_lib pthread)
ament_target_dependencies(teleop_keyboard rclcpp std_msgs geometry_msgs std_srvs san_msgs)

# Install targets
install(TARGETS ${PROJECT_NAME}_node teleop_keyboard amr_core_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY param launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
