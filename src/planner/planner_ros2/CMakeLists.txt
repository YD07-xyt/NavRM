cmake_minimum_required(VERSION 3.8)
project(planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(OPT_LIB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_optimization)
set(RC_ESDF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../planner_map/RC-ESDF-2D)

add_subdirectory(${OPT_LIB_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/trajectory_optimization_build)
add_subdirectory(${RC_ESDF_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/rc_esdf_build)

ament_auto_add_executable(${PROJECT_NAME} src/planner_node.cpp
src/planner_manage.cpp
src/planner_map.cpp
)


target_link_libraries(${PROJECT_NAME} trajectory_optimization
rc_esdf)

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${OPT_LIB_SOURCE_DIR}/include
  ${RC_ESDF_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
