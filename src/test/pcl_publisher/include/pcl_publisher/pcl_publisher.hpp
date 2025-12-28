#pragma once


#ifndef PCLPUBLISHERNODE_HPP
#define PCLPUBLISHERNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace test {
    class PclPublisherNode:public rclcpp::Node{
        public:
            explicit PclPublisherNode(const rclcpp::NodeOptions& options);
        private: 
            void read_pcd();
            void  timer_callback();
            rclcpp::TimerBase::SharedPtr timer_;
            sensor_msgs::msg::PointCloud2 ros2_pcl_cloud; 
            std::string pcd_path;
            std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> test_lidar_pub_;
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(test::PclPublisherNode)

#endif