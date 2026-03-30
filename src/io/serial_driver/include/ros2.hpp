#pragma once

#ifndef ROS2_HPP
#define ROS2_HPP

#include <rclcpp/rclcpp.hpp>

#include"rm_interfaces/msg/game_status.hpp"
#include"rm_interfaces/msg/robot_status.hpp"
#include"rm_interfaces/msg/bt_data.hpp"
namespace serial {
    class SerialRos2{
        public:
            SerialRos2();
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<rm_interfaces::msg::GameStatus> game_status_pub_;
            rclcpp::Publisher<rm_interfaces::msg::RobotStatus> robot_status_pub_;
            rclcpp::Subscription<rm_interfaces::msg::BtData> bt_data_sub_;
        private:
            void publish_game_status();
            void publish_robot_status();
            void sub_bt_data_callback();
    };
}

#endif