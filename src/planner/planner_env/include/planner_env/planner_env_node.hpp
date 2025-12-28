#pragma once

#ifndef PLANNERENVNODE_HPP
#define PLANNERENVNODE_HPP
#include <rclcpp/rclcpp.hpp>
namespace planner {
    struct PlannerEnvConfig{
        std::string map_frame;           //地图坐标系
        double resolution;               //栅格分辨率（米/格
        double map_size_x;               //地图X方向尺寸（米)
        double map_size_y;               //地图y方向尺寸（米)
        double obstacle_height_threshold;//障碍物高度阈值
        double ground_height_threshold;//地面高度阈值（用于区分地面/噪声）
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };
    class PlannerEnvNode : public rclcpp::Node
    {
    public:
        explicit PlannerEnvNode(const rclcpp::NodeOptions& options);
    private:
        void init_params();
    private:
        PlannerEnvConfig global_map_config;
        PlannerEnvConfig local_map_config;
    
    };


}// namespace planner
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::PlannerEnvNode)
#endif //PLANNERENVNODE_HPP