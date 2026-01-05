#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <string>
#include<vector>
namespace planner {

struct GridMapConfig{
        std::string map_frame;           //地图坐标系
        // int layer;                         // 层数
        // std::vector<std::string> layer_names; // 层的名字 
        double map_postion_x;              // 原点x
        double map_postion_y;              // 原点y
        double resolution;               //栅格分辨率（米/格
        double map_size_x;               //地图X方向尺寸（米)
        double map_size_y;               //地图y方向尺寸（米)
        double obstacle_height_threshold;//障碍物高度阈值
        double ground_height_threshold;     //地面高度阈值（用于区分地面/噪声）
};

struct LocalEnvConfig{
        GridMapConfig grid_map_config;
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };

    
struct GlobalEnvConfig{
        std::string pcd_file_path;       //pcd文件路径
        GridMapConfig grid_map_config;
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };

}
#endif //CONFIG_HPP