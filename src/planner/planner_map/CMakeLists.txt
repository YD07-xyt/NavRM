cmake_minimum_required(VERSION 3.20)
project(planner_map)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

# 添加编译和链接选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)
find_package(glog REQUIRED)



ament_auto_add_library(planner_map STATIC
    src/planner_map.cpp
    src/rc_esdf.cpp
    src/raycast.cpp
    src/grid_map.cpp
)

target_include_directories(planner_map PUBLIC include)

target_link_libraries(planner_map ${PCL_LIBRARIES} glog::glog)

#target_link_libraries(planner_map_node planner_map)

add_executable(planner_map_test
    main.cpp 
)

target_link_libraries(planner_map_test planner_map  ${PCL_LIBRARIES})

ament_auto_package( 
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)