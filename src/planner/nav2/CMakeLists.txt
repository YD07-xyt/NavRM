cmake_minimum_required(VERSION 3.10)
project(nav2)

## Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_compile_options(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake REQUIRED)

# 添加所有必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
)

###########
## Build ##
###########

include_directories(include)



add_library(nav2 SHARED src/nav2_node.cpp)


ament_target_dependencies(nav2 PUBLIC ${dependencies})

target_include_directories(nav2
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# target_link_libraries(global_planner PUBLIC
#     a_star
#     b_spline_smooth
# )

# 禁用 boost
target_compile_definitions(nav2 PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#############
## Install ##
#############

# 安装库
install(TARGETS nav2
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    # RUNTIME DESTINATION 通常用于可执行文件，不是库文件
    # 如果确实有可执行文件，使用：
    # RUNTIME DESTINATION bin
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
    DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(nav2)
ament_export_dependencies(${dependencies})
ament_package()
