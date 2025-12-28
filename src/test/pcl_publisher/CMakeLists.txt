cmake_minimum_required(VERSION 3.8)
project(pcl_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)

ament_auto_find_build_dependencies()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${Boost_LIBRARIES}
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN test::PclPublisherNode
  EXECUTABLE ${PROJECT_NAME}_node
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
)