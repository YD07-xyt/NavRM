#include "plan_manager/plan_manager.hpp"
#include"visualizer/visualizer.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <signal.h>

void MySigintHandler(int sig) {
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char *argv[]) {
  // ros::init(argc, argv, "planmanager");
  // ros::NodeHandle nh("~");
  
  rclcpp::init(argc, argv);
  auto plan_manager_node = std::make_shared<PlanManager>(rclcpp::NodeOptions());
  
  signal(SIGINT,MySigintHandler);  
  rclcpp::spin(plan_manager_node);
  return 0;
}