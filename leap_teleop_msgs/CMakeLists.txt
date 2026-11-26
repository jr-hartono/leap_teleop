cmake_minimum_required(VERSION 3.8)
project(leap_teleop_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeadmanState.msg"
  "msg/Hand.msg"
  "msg/HandArray.msg"
  "msg/TeleopHand.msg"
  "msg/TeleopState.msg"
  "msg/TeleopDelta.msg"
  "msg/TeleopGripper.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_auto_package()
