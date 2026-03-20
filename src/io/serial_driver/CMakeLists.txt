cmake_minimum_required(VERSION 3.10)

project(serial_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

set(BOOST_ROOT "/usr/local/boost_1_85")
set(Boost_NO_SYSTEM_PATHS ON)  # 不搜索系统 Boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)

find_package(Boost 1.85.0 REQUIRED COMPONENTS system thread)

find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../tool/rm_log/include)

# 确保找到了Boost
if(Boost_FOUND)
    message(STATUS "Boost found. Include dirs: ${Boost_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Boost not found! Please install Boost.")
endif()


add_library(serial_driver src/serial_driver.cpp
src/crc.cpp)


target_include_directories(serial_driver PUBLIC  
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)


target_link_libraries(serial_driver PRIVATE
        Boost::boost          # 仅头文件依赖
        Boost::system         # system库
        Boost::thread       # 如果需要线程支持
        glog::glog
)


add_executable(test_serial test/test_serial.cpp)

target_link_libraries(test_serial PRIVATE serial_driver)