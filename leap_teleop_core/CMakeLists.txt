cmake_minimum_required(VERSION 3.8)
project(leap_teleop_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
ament_export_dependencies(Eigen3)

find_package(generate_parameter_library REQUIRED)
generate_parameter_library(leap_driver_parameters config/leap_driver_parameters.yaml)
generate_parameter_library(leap_teleop_deadman_parameters
  config/leap_teleop_deadman_parameters.yaml)
generate_parameter_library(leap_teleop_mapper_parameters
  config/leap_teleop_mapper_parameters.yaml)

install(FILES
  config/leap_driver_parameters.yaml
  config/leap_teleop_deadman_parameters.yaml
  config/leap_teleop_mapper_parameters.yaml
  DESTINATION share/${PROJECT_NAME}/parameter_definitions
)

ament_auto_add_library(leap_hand_mapping SHARED
  src/frames.cpp
  src/filters.cpp
  src/hand_mapper.cpp
  src/delta_builder.cpp
  src/gripper_mapper.cpp
  src/mapper_fsm.cpp
)
target_link_libraries(leap_hand_mapping Eigen3::Eigen)

ament_auto_add_executable(leap_teleop_deadman
  src/leap_teleop_deadman.cpp
  src/keyboard_deadman.cpp
)
target_link_libraries(leap_teleop_deadman leap_teleop_deadman_parameters)

ament_auto_add_executable(leap_teleop_mapper
  src/leap_teleop_mapper.cpp
)
target_link_libraries(leap_teleop_mapper leap_hand_mapping leap_teleop_mapper_parameters)

find_package(LeapSDK QUIET)
if(LeapSDK_FOUND)
  ament_auto_add_executable(leap_driver src/leap_driver.cpp)
  target_link_libraries(leap_driver leap_hand_mapping leap_driver_parameters LeapSDK::LeapC)
else()
  message(STATUS "LeapSDK not found; skipping leap_driver target")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frames test/test_frames.cpp)
  target_link_libraries(test_frames leap_hand_mapping)

  ament_add_gtest(test_filters test/test_filters.cpp)
  target_link_libraries(test_filters leap_hand_mapping)

  ament_add_gtest(test_hand_mapper test/test_hand_mapper.cpp)
  target_link_libraries(test_hand_mapper leap_hand_mapping)

  ament_add_gtest(test_delta_builder test/test_delta_builder.cpp)
  target_link_libraries(test_delta_builder leap_hand_mapping)

  ament_add_gtest(test_gripper_mapper test/test_gripper_mapper.cpp)
  target_link_libraries(test_gripper_mapper leap_hand_mapping)

  ament_add_gtest(test_mapper_fsm test/test_mapper_fsm.cpp)
  target_link_libraries(test_mapper_fsm leap_hand_mapping)

  ament_add_gtest(test_keyboard_deadman test/test_keyboard_deadman.cpp src/keyboard_deadman.cpp)
  target_link_libraries(test_keyboard_deadman leap_hand_mapping)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
