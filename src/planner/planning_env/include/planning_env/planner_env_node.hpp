#pragma once

#ifndef PLANNERENVNODE_HPP
#define PLANNERENVNODE_HPP

#include "planning_env/global_env.hpp"
#include "planning_env/local_env.hpp"
#include "planning_env/config.hpp"

// ros
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

// grid_map
#include <grid_map_msgs/msg/grid_map.hpp>

namespace planner {
    class PlannerEnvNode : public rclcpp::Node
    {
    public:
        explicit PlannerEnvNode(const rclcpp::NodeOptions& options);
    
    private:
        void init_params();
    private:
        GlobalEnvConfig global_map_config;
        LocalEnvConfig local_map_config;
        GlobalEnv global_env_;
        LocalEnv local_env_;
        rclcpp::Subscription<grid_map_msgs::msg::GridMap> global_map_sub_;
        rclcpp::Subscription<grid_map_msgs::msg::GridMap> local_map_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2> point_downsampled_;
    };


}// namespace planner
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::PlannerEnvNode)
#endif //PLANNERENVNODE_HPP