cmake_minimum_required(VERSION 3.8)
project(leap_teleop_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

find_package(generate_parameter_library REQUIRED)
generate_parameter_library(leap_teleop_viz_parameters config/leap_teleop_viz_parameters.yaml)

install(FILES config/leap_teleop_viz_parameters.yaml
  DESTINATION share/${PROJECT_NAME}/parameter_definitions)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_auto_add_library(leap_teleop_viz_markers SHARED src/markers.cpp)
target_link_libraries(leap_teleop_viz_markers Eigen3::Eigen)

ament_auto_add_executable(leap_teleop_viz src/leap_teleop_viz.cpp)
target_link_libraries(leap_teleop_viz leap_teleop_viz_markers leap_teleop_viz_parameters)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_markers test/test_markers.cpp)
  target_link_libraries(test_markers leap_teleop_viz_markers)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
