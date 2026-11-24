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
)
target_link_libraries(leap_hand_mapping Eigen3::Eigen)

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

endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
