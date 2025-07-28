cmake_minimum_required(VERSION 3.5)
project(amr_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(
  ${urdf_INCLUDE_DIRS}
  ${xacro_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

# Install files
install(DIRECTORY urdf meshes launch
    DESTINATION share/${PROJECT_NAME}
)

# Package registration
ament_package()
