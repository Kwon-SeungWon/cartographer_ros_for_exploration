cmake_minimum_required(VERSION 3.8)
project(amr_serial)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joystick.msg"
  "srv/GetMode.srv"
  "srv/ReadGPIO.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and param files
install(DIRECTORY launch param scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/serial_all.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
