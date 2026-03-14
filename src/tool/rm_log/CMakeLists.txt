cmake_minimum_required(VERSION 3.10)

project(rm_log)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(spdlog REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include )

add_library(rm_log src/rm_log.cpp)

target_include_directories(rm_log PUBLIC include)

target_link_libraries(rm_log PRIVATE spdlog::spdlog_header_only)

add_executable(rm_log_test test/test.cpp)

target_link_libraries(rm_log_test PRIVATE rm_log)