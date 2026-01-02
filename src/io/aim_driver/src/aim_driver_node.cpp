#include <cstdio>

#include "aim_driver/aim_driver_node.hpp"

// 1. 添加自定义消息的头文件

namespace aim_driver {
    AimDriver::AimDriver(const rclcpp::NodeOptions& options)
        : Node("aim_driver_node", options)
    {
        RCLCPP_INFO(get_logger(), "Start AimDriver!");
        action_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",
                rclcpp::SensorDataQoS(),
                std::bind(&AimDriver::send_data, this, std::placeholders::_1));
        game_sub_ = this->create_subscription<aim_driver::msg::Game>(
            "/aim/game",
            rclcpp::SensorDataQoS(),
            std::bind(&AimDriver::get_game_rules, this, std::placeholders::_1));
        // 2. 使用完整的命名空间
        send_pub_ = this->create_publisher<aim_driver::msg::Control>(
            "/sentry_to_aim_data",
            10);
    }

    void AimDriver::send_data(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Start Send data from nav");

        try {
            auto control = std::make_unique<aim_driver::msg::Control>();
            this->nav_to_aim.line_vel_x = static_cast<float>(msg->linear.x);
            this->nav_to_aim.line_vel_y = static_cast<float>(msg->linear.y);
            this->nav_to_aim.angle_vel_z = static_cast<float>(msg->angular.z);
            this->sum_Nav2Aim.push_back(this->nav_to_aim);

            control->line_vel_x = static_cast<float>(msg->linear.x);
            control->line_vel_y = static_cast<float>(msg->linear.y);
            control->angle_vel_z = static_cast<float>(msg->angular.z);

            send_pub_->publish(std::move(control));

            RCLCPP_INFO(get_logger(), "Published custom control packet!");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(),
                "Failed to process and publish control: %s",
                e.what());
        }
        catch (...) {
            RCLCPP_ERROR(get_logger(),
                "Failed to process and publish control (Unknown Error)");
        }
    }

    void AimDriver::get_game_rules(const aim_driver::msg::Game msg){
      RCLCPP_INFO(get_logger(),"angle_vel_z: %f",msg.angle_vel_z);
    }
}// namespace aim_driver