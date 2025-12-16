cmake_minimum_required(VERSION 3.8)
project(aim_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "msg/Game.msg"
)


ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/aim_driver_node.cpp
)
 
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  geometry_msgs
)


rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "aim_driver::AimDriver"
  EXECUTABLE ${PROJECT_NAME}_exec
)

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)