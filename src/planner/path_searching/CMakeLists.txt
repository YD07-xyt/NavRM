cmake_minimum_required(VERSION 3.22)

project(path_searching)

set(CMAKE_CXX_STANDARD 23)
find_package(Eigen3 REQUIRED)

set(RC_ESDF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../planner_map/RC-ESDF-2D)


add_library(${PROJECT_NAME} STATIC 
src/a_star.cpp
src/jps_planner/jps_planner.cpp
src/jps_planner/graph_search.cpp)

target_include_directories(${PROJECT_NAME} 
    PUBLIC 
    include  
    ${RC_ESDF_SOURCE_DIR}/include                   
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen rc_esdf)