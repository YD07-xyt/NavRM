#pragma once 

#ifndef BEHAVIOR_TOPICS2BLACKBOARD_HPP
#define BEHAVIOR_TOPICS2BLACKBOARD_HPP
#include <rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include<optional>

#include"rm_interfaces/msg/robot_status.hpp"
#include"rm_interfaces/msg/game_status.hpp"

#include"rm_interfaces/msg/bt_data.hpp"

namespace decision {
    class Topics2Blackboard : public  BT::SyncActionNode{
        public:
        
            Topics2Blackboard(const std::string &name, const BT::NodeConfig &config, 
                    std::shared_ptr<rclcpp::Node> node);
            ~Topics2Blackboard() = default ;
            BT::NodeStatus tick() override;
            static BT::PortsList providedPorts();
        private:
            rclcpp::Node::SharedPtr node_;
            
            rclcpp::Subscription<rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
            rclcpp::Subscription<rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
            
            void game_status_callback(const rm_interfaces::msg::GameStatus::SharedPtr msg);
            void robot_status_callback(const rm_interfaces::msg::RobotStatus::SharedPtr msg);
        private:
            std::optional<rm_interfaces::msg::GameStatus> game_status_;
            std::optional<rm_interfaces::msg::RobotStatus> robot_status_;
        private:
            std::optional<rm_interfaces::msg::BtData> bt_data_;
            void bt_data_callback();
            void init_bt_data();
    };
}
#endif