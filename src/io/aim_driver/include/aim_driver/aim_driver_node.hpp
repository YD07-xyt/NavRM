#pragma  once


#ifndef AIMDDRIVER_HPP
#define AIMDDRIVER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// 1. 添加自定义消息的头文件
#include "aim_driver/msg/control.hpp" 
#include "aim_driver/msg/game.hpp"

namespace aim_driver{
    struct Nav2Aim{
        float line_vel_x;
        float line_vel_y;
        float angle_vel_z;

    };
    class AimDriver: public rclcpp::Node{
        public:
            explicit AimDriver(const rclcpp::NodeOptions &options);
            std::vector<Nav2Aim> sum_Nav2Aim;
            Nav2Aim nav_to_aim;
        private:
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr action_sub_;
            rclcpp::Publisher<aim_driver::msg::Control>::SharedPtr send_pub_;
            rclcpp::Subscription<aim_driver::msg::Game>::SharedPtr game_sub_; 
            void send_data(const geometry_msgs::msg::Twist::SharedPtr msg);
            void get_game_rules(const aim_driver::msg::Game msg);
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aim_driver::AimDriver)

#endif
