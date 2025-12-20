cmake_minimum_required(VERSION 3.10)
project(map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED COMPONENTS date_time)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core)
find_package(grid_map_ros)
find_package(grid_map_msgs)
find_package(grid_map_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs)
find_package(nav_msgs)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_filters
  pcl_ros
  pcl_conversions
  sensor_msgs
  nav_msgs
  tf2_ros
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


add_library(map SHARED
src/map.cpp)

ament_target_dependencies(map PUBLIC ${dependencies})

target_include_directories(map
  PUBLIC
  ${Boost_LIBRARIES}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(global_map SHARED
src/global_map.cpp)

ament_target_dependencies(global_map PUBLIC ${dependencies})

target_include_directories(global_map
  PUBLIC
  ${Boost_LIBRARIES}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


add_library(local_map SHARED
src/local_map.cpp)

ament_target_dependencies(local_map PUBLIC ${dependencies})

target_include_directories(local_map
  PUBLIC
  ${Boost_LIBRARIES}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 安装两个库
install(TARGETS global_map  local_map
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    # RUNTIME DESTINATION 通常用于可执行文件，不是库文件
    # 如果确实有可执行文件，使用：
    # RUNTIME DESTINATION bin
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

ament_export_include_directories(include)
ament_export_libraries(map)
ament_export_dependencies(${dependencies})
ament_package()
