cmake_minimum_required(VERSION 3.8)
project(san_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Declare the message files

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StateMachine.msg"
  "msg/BatteryStatus.msg"
  "msg/Emergency.msg"
  "msg/Qrdetection.msg"
  "msg/Behavior.msg"
  "msg/Task.msg"
  "msg/MotorStatus.msg"
  "srv/TaskCommand.srv"
  "srv/SlamMode.srv"
  "srv/BuildMap.srv"
  "srv/ManualManipulatorControl.srv"
  "srv/GetWaypoints.srv"
  "srv/PauseTask.srv"
  "action/SetState.action"
  "action/MarkerDocking.action"
  "action/TaskCommandAction.action"
  # "srv/Mode.srv"
  # "srv/MapList.srv"
  # "srv/SetGoal.srv"
  # "srv/MapName.srv"
  # "srv/ReLocalization.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
