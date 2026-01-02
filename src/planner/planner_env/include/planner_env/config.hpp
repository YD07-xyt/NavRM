#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <string>
namespace planner {

struct LocalEnvConfig{
        std::string map_frame;           //地图坐标系
        double resolution;               //栅格分辨率（米/格
        double map_size_x;               //地图X方向尺寸（米)
        double map_size_y;               //地图y方向尺寸（米)
        double obstacle_height_threshold;//障碍物高度阈值
        double ground_height_threshold;//地面高度阈值（用于区分地面/噪声）
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };
struct GlobalEnvConfig{
        std::string map_frame;           //地图坐标系
        double resolution;               //栅格分辨率（米/格
        double map_size_x;               //地图X方向尺寸（米)
        double map_size_y;               //地图y方向尺寸（米)
        double obstacle_height_threshold;//障碍物高度阈值
        double ground_height_threshold;//地面高度阈值（用于区分地面/噪声）
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };

}
#endif //CONFIG_HPP