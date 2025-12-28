
## ros2 组件化

package.xml中
```bash
  <depend>rclcpp_components</depend>
```

cpp/hpp中
```c++
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(test::PclPublisherNode)
```

cmake
```bash
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
```