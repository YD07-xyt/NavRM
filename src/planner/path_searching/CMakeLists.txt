cmake_minimum_required(VERSION 3.22)

project(path_searching)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

set(RM_LOG_PATH "build/rm_log/librm_log.a") 
set(RM_LOG_INCLUDE_DIR "src/tool/rm_log/include")

include_directories(${RM_LOG_INCLUDE_DIR})

ament_auto_add_library(${PROJECT_NAME} STATIC
    src/path_searching_node.cpp
    src/a_star.cpp
    src/jps_planner/jps_planner.cpp
    src/jps_planner/graph_search.cpp
)


target_include_directories(${PROJECT_NAME} 
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include               
)

target_link_libraries(${PROJECT_NAME} ${RM_LOG_PATH} Eigen3::Eigen planner_map)

ament_auto_package(  
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
)