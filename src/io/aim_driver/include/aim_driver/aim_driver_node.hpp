#pragma  once
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// 1. 添加自定义消息的头文件
#include "aim_driver/msg/control.hpp" 

namespace aim_driver{
    struct Nav2Aim{
        float line_vel_x;
        float line_vel_y;
        float angle_vel_z;

    };
    class AimDriver: public rclcpp::Node{
        public:
            explicit AimDriver(const rclcpp::NodeOptions &options);
            ~AimDriver() override;
            std::vector<Nav2Aim> sum_Nav2Aim;
            Nav2Aim nav_to_aim;
        private:
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr action_sub_;
            rclcpp::Publisher<aim_driver::msg::Control>::SharedPtr send_pub_;
            void send_data(const geometry_msgs::msg::Twist::SharedPtr msg);
    };
}