cmake_minimum_required(VERSION 3.8)
project(leap_teleop_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
ament_export_dependencies(Eigen3)

ament_auto_add_library(leap_hand_mapping SHARED
  src/frames.cpp
  src/filters.cpp
  src/hand_mapper.cpp
  src/delta_builder.cpp
  src/supervisor_fsm.cpp
)
target_link_libraries(leap_hand_mapping Eigen3::Eigen)

ament_auto_add_executable(leap_teleop_engage
  src/leap_teleop_engage.cpp
  src/keyboard_deadman.cpp
)

ament_auto_add_executable(leap_teleop_supervisor
  src/leap_teleop_supervisor.cpp
)
target_link_libraries(leap_teleop_supervisor leap_hand_mapping)

find_package(LeapSDK QUIET)
if(LeapSDK_FOUND)
  ament_auto_add_executable(leap_tracker src/leap_tracker.cpp)
  target_link_libraries(leap_tracker leap_hand_mapping LeapSDK::LeapC)
else()
  message(STATUS "LeapSDK not found; skipping leap_tracker target")
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

  ament_add_gtest(test_supervisor_fsm test/test_supervisor_fsm.cpp)
  target_link_libraries(test_supervisor_fsm leap_hand_mapping)

  ament_add_gtest(test_parameters test/test_parameters.cpp)
  target_link_libraries(test_parameters leap_hand_mapping)
  ament_target_dependencies(test_parameters rclcpp)

  ament_add_gtest(test_keyboard_deadman test/test_keyboard_deadman.cpp)
  target_link_libraries(test_keyboard_deadman leap_hand_mapping)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
