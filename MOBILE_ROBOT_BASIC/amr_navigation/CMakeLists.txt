cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# All sources for the shared library
set(AMR_NAVIGATION_SRCS
  src/periodic_costmap_clearer.cpp
)

# Create a single shared library
add_library(amr_navigation_lib SHARED ${AMR_NAVIGATION_SRCS})
ament_target_dependencies(amr_navigation_lib rclcpp std_srvs nav2_msgs)

target_include_directories(amr_navigation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executable for periodic costmap clearer
add_executable(periodic_costmap_clearer_node src/periodic_costmap_clearer_node.cpp)
target_link_libraries(periodic_costmap_clearer_node amr_navigation_lib)
ament_target_dependencies(periodic_costmap_clearer_node rclcpp std_srvs nav2_msgs)


# Install the executable to the lib directory
install(TARGETS
  periodic_costmap_clearer_node
  amr_navigation_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz params maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
