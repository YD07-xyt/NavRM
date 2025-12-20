cmake_minimum_required(VERSION 3.10)
project(gicp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED COMPONENTS date_time)


# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_node::SmallGicp
  EXECUTABLE ${PROJECT_NAME}_node
)

target_include_directories(gicp
  PUBLIC
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
    DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

