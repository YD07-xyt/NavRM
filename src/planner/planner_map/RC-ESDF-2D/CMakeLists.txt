cmake_minimum_required(VERSION 3.20)
project(planner_map)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
   include    # 包含当前目录 (为了找到 rc_esdf.h)
    ${EIGEN3_INCLUDE_DIRS}    # 包含 Eigen 头文件
    ${PCL_INCLUDE_DIRS}
)

add_library(rc_esdf 
    src/rc_esdf.cpp
    src/planner_map.cpp
)
add_executable( planner_map
    main.cpp 
)
target_link_libraries(planner_map rc_esdf ${OpenCV_LIBS} ${PCL_LIBRARIES})