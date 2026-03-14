cmake_minimum_required(VERSION 3.10)
project(planner_ros2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


set(OPT_LIB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_optimization)
set(PLANNER_MAP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../planner_map/)
set(PATH_SEARCHING_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../path_searching)

#add_subdirectory(${OPT_LIB_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/trajectory_optimization_build)
#add_subdirectory(${PLANNER_MAP_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/planner_map_build)
#add_subdirectory(${PATH_SEARCHING_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/path_searching_build)

ament_auto_add_library(planner_ros2_interface STATIC
  src/visualizer/visualizer.cpp
  src/plan_manager/plan_manager.cpp
)


ament_auto_add_executable(${PROJECT_NAME}
  src/global_planning.cpp
)

target_include_directories(planner_ros2_interface PUBLIC 
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_link_libraries(${PROJECT_NAME} planner_ros2_interface)

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
