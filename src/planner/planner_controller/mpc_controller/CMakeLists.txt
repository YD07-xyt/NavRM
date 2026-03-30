cmake_minimum_required(VERSION 3.10)
project(mpc_controller)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 编译选项
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)

# 自动查找所有依赖（从 package.xml 读取）
ament_auto_find_build_dependencies()


find_package(OsqpEigen REQUIRED)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)

# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  /usr/local/include/osqp
)

# 创建库和可执行文件
ament_auto_add_library(mpc_controller SHARED
  src/mpc.cpp
  src/mpc_node.cpp
)

ament_auto_add_executable(mpc
  src/mpc_node.cpp
  src/mpc.cpp
)

# 链接第三方库
target_link_libraries(mpc_controller
  OsqpEigen::OsqpEigen
  ${PCL_LIBRARIES}
  glog::glog
)

target_link_libraries(mpc
  mpc_controller
  OsqpEigen::OsqpEigen
  ${PCL_LIBRARIES}
)

# 自动处理安装和导出
ament_auto_package(
  INSTALL_TO_SHARE
    config
)