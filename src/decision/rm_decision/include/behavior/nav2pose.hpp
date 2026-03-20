#pragma once

#ifndef Nav2Pose_H
#define Nav2Pose_H
#include<behaviortree_cpp/bt_factory.h>
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<rclcpp_action/rclcpp_action.hpp>
#include<nav2_msgs/action/navigate_to_pose.hpp>
#include<nav_msgs/msg/odometry.hpp>

namespace decision {
    class Nav2Pose : public  BT::StatefulActionNode{
        public:
            Nav2Pose(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

            // this function is invoked once at the beginning.
            BT::NodeStatus onStart() override;

            // If onStart() returned RUNNING, we will keep calling
            // this method until it return something different from RUNNING
            BT::NodeStatus onRunning() override;

            // callback to execute if the action was aborted by another node
            void onHalted() override;

            static BT::PortsList providedPorts();
        
        private:
            std::shared_ptr<rclcpp::Node> node_;
            //--未使用
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
            //--
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
            nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
            int send_goal_timeout_;
        private:
    };
}
#endif