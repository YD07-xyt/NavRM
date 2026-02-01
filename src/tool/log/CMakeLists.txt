cmake_minimum_required(VERSION 3.22)

project(rm_log)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(spdlog REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(rm_log STATIC
    src/rm_log.cpp)

target_include_directories(rm_log PUBLIC include)

target_link_libraries(rm_log PRIVATE spdlog::spdlog_header_only)
