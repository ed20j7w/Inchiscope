cmake_minimum_required(VERSION 3.8)
project(inchiscope_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PistonState.msg"
  "msg/PressureState.msg"
  "msg/PistonStateArray.msg"
  "msg/PressureStateArray.msg"
  "msg/PistonCommand.msg"
  "msg/ValveCommand.msg"
  "msg/HomeCommand.msg"
  "msg/PistonRangeCommand.msg"
  "msg/PistonSpeedCommand.msg"
  "msg/RegulatorCommand.msg"
  "msg/RegulatorRangeCommand.msg"
  "msg/AbPidCommand.msg"
  "msg/InchiscopeState.msg"
  "action/SetAbDiameter.action"
  "action/MovePba.action"
  "action/TransitionMode.action"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
