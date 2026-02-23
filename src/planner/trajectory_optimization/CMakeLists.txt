cmake_minimum_required(VERSION 3.22)

project(trajectory_optimization)


find_package(Eigen3 REQUIRED)

set(SOURCE 
    src/traj.cpp)

add_library(trajectory_optimization STATIC ${SOURCE})

target_link_libraries(trajectory_optimization PUBLIC Eigen3::Eigen)

target_include_directories(trajectory_optimization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
