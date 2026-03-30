#include "mpc_controller/mpc.h"
#include <signal.h>

void MySigintHandler(int sig) {
    rclcpp::shutdown();
    // exit(0);
}

int main(int argc, char** argv){ 
  rclcpp::init(argc, argv);
  rclcpp::Node mpc_node("mpc_node");

  MpcController mpccontroller(mpc_node.shared_from_this());
  signal(SIGINT,MySigintHandler);  
  rclcpp::spin(mpc_node.shared_from_this());

  return 0;
}