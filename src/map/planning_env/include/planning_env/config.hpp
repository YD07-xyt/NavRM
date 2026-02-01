#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <string>
#include <vector>
namespace planner {

    struct GridMapConfig {
        std::string map_frame;//地图坐标系
        double map_postion_x;            // 原点x
        double map_postion_y;            // 原点y
        double map_postion_z;            // 原点z
        double resolution;               //栅格分辨率（米/格
        double map_size_x;               //地图X方向尺寸（米)
        double map_size_y;               //地图y方向尺寸（米)
        double map_size_z;               //地图z方向尺寸（米)
        double min_obstacle_height;     //障碍物最小高度
        double max_obstacle_height;         //障碍物最大高度阈值
        double ground_height_threshold;//地面高度阈值（用于区分地面/噪声）
    };

    struct LocalEnvConfig {
        GridMapConfig grid_map_config;
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };


    struct GlobalEnvConfig {
        std::string pcd_file_path;//pcd文件路径
        bool auto_adjust_map = true;         // 是否自动调整地图尺寸
        GridMapConfig grid_map_config;
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };

}// namespace planner
#endif//CONFIG_HPP