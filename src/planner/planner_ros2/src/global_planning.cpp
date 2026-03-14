#include "plan_manager/plan_manager.hpp"
#include"visualizer/visualizer.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <signal.h>

void MySigintHandler(int sig) {
   // ros::shutdown();
    // exit(0);
}

int main(){
  // ros::init(argc, argv, "planmanager");
  // ros::NodeHandle nh("~");

  // PlanManager planmanager(nh);
  // signal(SIGINT,MySigintHandler);  
  
  // ros::spin();
  return 0;
}