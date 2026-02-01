cmake_minimum_required(VERSION 3.8)
project(planning_env)
set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)


ament_auto_find_build_dependencies()


add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../tool/log  ${CMAKE_CURRENT_BINARY_DIR}/tool/rmlog)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME} rm_log)


target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${Boost_LIBRARIES}
  $<INSTALL_INTERFACE:include>
)

#注册节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN planner::PlannerEnvNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#test
if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
