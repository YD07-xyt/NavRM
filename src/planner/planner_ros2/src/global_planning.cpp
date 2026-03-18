#include "plan_manager/plan_manager.hpp"
#include"visualizer/visualizer.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <signal.h>

void MySigintHandler(int sig) {
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char *argv[]) {
  
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr global_node = rclcpp::Node::make_shared("global_planner");
  auto plan_manager_node = std::make_shared<PlanManager>(global_node);
  signal(SIGINT,MySigintHandler);  
  rclcpp::spin(global_node);
  return 0;
}