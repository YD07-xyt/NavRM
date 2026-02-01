cmake_minimum_required(VERSION 3.22.1)
project(fiesta)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

# 修改：更灵活地查找PCL
#find_package(PCL REQUIRED COMPONENTS common kdtree filters)
find_package(PCL 1.7 REQUIRED COMPONENTS common kdtree filters)
# 查找其他包
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

# 查找ROS2相关包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
# 自动查找ROS2依赖
ament_auto_find_build_dependencies()

# 设置包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${spdlog_INCLUDE_DIRS}
)

# 设置链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 设置编译标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -pthread")

# OpenMP支持
find_package(OpenMP)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# 添加可执行文件
add_executable(map_node
    include/parameters.h
    src/parameters.cpp
    include/timing.h
    src/timing.cc
    include/raycast.h
    src/raycast.cpp
    include/ESDFMap.h
    src/ESDFMap.cpp
    test/main.cpp
)

# 为目标添加ROS2依赖
ament_target_dependencies(map_node
    rclcpp
    sensor_msgs
    nav_msgs
    visualization_msgs
    geometry_msgs
    cv_bridge
    pcl_conversions
)

# 修改：正确链接PCL库
target_link_libraries(map_node
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    spdlog::spdlog
    pcl_common
    pcl_kdtree
    pcl_filters
)

target_compile_definitions(map_node PRIVATE EIGEN_MAX_ALIGN_BYTES=32)

install(TARGETS map_node
  DESTINATION lib/${PROJECT_NAME}
)

# 添加测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 自动安装和打包
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)