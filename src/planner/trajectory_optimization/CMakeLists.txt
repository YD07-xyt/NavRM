cmake_minimum_required(VERSION 3.10)

project(trajectory_optimization)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(Eigen3 REQUIRED)

set(SOURCE 
    src/optimizer.cpp)


ament_auto_add_library(trajectory_optimization STATIC
    ${SOURCE}
)

target_link_libraries(trajectory_optimization  Eigen3::Eigen)

target_include_directories(trajectory_optimization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
)