#include"behavior/bt2serial.hpp"
#include "behavior/nav2pose.hpp"
#include"behavior/topics2blackboard.hpp"
#include"behavior/custume_types.hpp"

#include<rclcpp/rclcpp.hpp>
#include "behavior/custume_types.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

#include <fstream>
//TODO: config 获取参数
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node =  std::make_shared<rclcpp::Node>("global_decision");
    
    std::string tree_node_model_export_path = "/home";
    std::string tree_xml_file = "/home";
    int tick_period_milliseconds= 100;
    int groot_port=5556;
    auto timeout = std::chrono::milliseconds(tick_period_milliseconds);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<decision::Nav2Pose>("Nav2Pose", node);
    factory.registerNodeType<decision::Topics2Blackboard>("Topics2Blackboard", node);
    factory.registerNodeType<decision::BT2Serial>("Bt2Serial", node);
    RCLCPP_INFO(node->get_logger(), "Loaded all custom nodes");

    // Visualize custom types in the Blackboard
    //TODO： 
    // BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>(BT::PoseStampedToJson);
    // BT::RegisterJsonDefinition<geometry_msgs::msg::PointStamped>(BT::PointStampedToJson);


    // generate xml file
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    //save to file
    std::ofstream file(tree_node_model_export_path);
    file << xml_models;
    file.close();
    RCLCPP_INFO(node->get_logger(), "Generated XML file");
    
    auto tree = factory.createTreeFromFile(tree_xml_file.c_str());
    std::shared_ptr<BT::Groot2Publisher> groot2publisher_ptr_;
    groot2publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree, groot_port);

    unsigned long int tick_count = 0;
    while(rclcpp::ok()){
        tick_count++;
        RCLCPP_INFO(node->get_logger(), "----------Tick %lu---------", tick_count);
        rclcpp::spin_some(node);
        tree.tickOnce();
        tree.sleep(timeout);
    }
    return 0;
}