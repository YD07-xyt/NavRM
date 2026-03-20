cmake_minimum_required(VERSION 3.10)
project(rm_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(${PROJECT_NAME} STATIC
    #src/decision_node.cpp
    #src/decision_manager.cpp
    src/behavior/base.cpp
    src/behavior/topics2blackboard.cpp
    src/behavior/nav2pose.cpp
    src/behavior/bt2serial.cpp
    src/behavior/custume_types.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC include)


target_link_libraries(${PROJECT_NAME})


ament_auto_add_executable(decision_node src/global_decision.cpp)



target_link_libraries(decision_node ${PROJECT_NAME}  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(  
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    config
)
