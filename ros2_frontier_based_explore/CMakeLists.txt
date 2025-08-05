cmake_minimum_required(VERSION 3.8)
project(ros2_frontier_based_explore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# List of source files for the library
set(LIB_SOURCES
  src/utils.cpp
  src/costmap.cpp
  src/frontier_search.cpp
)

# Check if all source files exist
set(ALL_FILES_EXIST TRUE)
foreach(SRC_FILE ${LIB_SOURCES})
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_FILE}")
    set(ALL_FILES_EXIST FALSE)
    message(WARNING "File ${SRC_FILE} does not exist.")
  endif()
endforeach()

# Add the library only if all source files exist
if(ALL_FILES_EXIST)
  add_library(explore_lib SHARED ${LIB_SOURCES})
  target_include_directories(explore_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(explore_lib 
    rclcpp 
    nav2_costmap_2d 
    nav_msgs 
    map_msgs 
    geometry_msgs 
    visualization_msgs)

  # Set library output directory
  set_target_properties(explore_lib PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

  # Install the library to the 'lib' directory
  install(TARGETS explore_lib
    LIBRARY DESTINATION lib)
else()
  message(WARNING "Skipping the creation of explore_lib because one or more source files are missing.")
endif()

# Add the executable and directly link to the explore_lib
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
# find_library(EXPLORE_LIB explore_lib PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_executable(explorer src/ros2_frontier_based_explore/explorer.cpp src/ros2_frontier_based_explore/node.cpp)
target_link_libraries(explorer explore_lib)
ament_target_dependencies(explorer 
  rclcpp 
  nav2_costmap_2d 
  nav_msgs 
  map_msgs 
  geometry_msgs 
  visualization_msgs)

# Install the executable
install(TARGETS explorer
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

# Install other resources (launch files, rviz configs, params)
install(DIRECTORY launch rviz params
  DESTINATION share/${PROJECT_NAME})

ament_package()
