#pragma once

#ifndef PLANNERENVNODE_HPP
#define PLANNERENVNODE_HPP

// ros
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/rclcpp.hpp>

#include "planning_env/config.hpp"
#include "planning_env/global_env.hpp"
#include "planning_env/local_env.hpp"

// grid_map
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

//pcl
#include <pcl_conversions/pcl_conversions.h>

namespace planner {
    inline sensor_msgs::msg::PointCloud2 pcl_to_ros(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& frame_id = "map")
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;

        pcl::toROSMsg(*cloud, cloud_msg);

        // 设置消息头
        cloud_msg.header.stamp = rclcpp::Clock().now();
        cloud_msg.header.frame_id = frame_id;

        return cloud_msg;
    }


    class PlannerEnvNode : public rclcpp::Node
    {
    public:
        explicit PlannerEnvNode(const rclcpp::NodeOptions& options);

    private:
        void init_global_map_params();
        void init_local_map_params();
        void publish_global_map();
        void pcl_cloud_callback(sensor_msgs::msg::PointCloud2);

    private:
        GlobalEnvConfig global_map_config;
        LocalEnvConfig local_map_config;
        GlobalEnv global_env_;
        LocalEnv local_env_;
        grid_map::GridMap global_map;
        grid_map::GridMap local_map;

    private:
        rclcpp::TimerBase::SharedPtr map_publish_timer_;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr
            global_map_pub_;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr
            local_map_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
            point_downsampled_global_map_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
            point_downsampled_;
    };


}// namespace planner
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::PlannerEnvNode)
#endif//PLANNERENVNODE_HPP