cmake_minimum_required(VERSION 3.8)
project(caselab_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(san_msgs REQUIRED)


set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/demo_panel.hpp
  include/camswitch_panel.hpp
  include/ammrTask_panel.hpp
)

add_library(caselab_rviz_plugin 
src/demo_panel.cpp
src/camswitch_panel.cpp
src/ammrTask_panel.cpp
${MOC_FILES})
target_include_directories(caselab_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(caselab_rviz_plugin
  pluginlib
  rviz_common
  san_msgs
)

target_link_libraries(
  ${PROJECT_NAME} 
  ament_index_cpp::ament_index_cpp
  )

install(TARGETS caselab_rviz_plugin
        EXPORT export_caselab_rviz_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/DemoPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)

install(DIRECTORY task/
  DESTINATION share/${PROJECT_NAME}/task
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_caselab_rviz_plugin)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()