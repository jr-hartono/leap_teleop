cmake_minimum_required(VERSION 3.8)
project(leap_teleop_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

find_package(generate_parameter_library REQUIRED)
generate_parameter_library(leap_moveit_servo_bridge_parameters
  config/leap_moveit_servo_bridge_parameters.yaml)

install(FILES config/leap_moveit_servo_bridge_parameters.yaml
  DESTINATION share/${PROJECT_NAME}/parameter_definitions)

ament_auto_add_executable(leap_moveit_servo_bridge
  src/leap_moveit_servo_bridge.cpp
)
target_link_libraries(leap_moveit_servo_bridge Eigen3::Eigen leap_moveit_servo_bridge_parameters)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
