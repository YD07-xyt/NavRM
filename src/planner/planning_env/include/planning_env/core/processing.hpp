#pragma once
#ifndef PROCESSING_HPP
#define PROCESSING_HPP

#include "grid_map_core/GridMap.hpp"
#include "planning_env/config.hpp"
#include "planning_env/esdf/esdf_map.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//spdlog
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
namespace planner {
    /*
    * @params: 输入点云 ，体素滤波分辨率
    * @return: 下采样的输出点云
    */
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filters(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
        float voxel_leaf_size);

    /*
    * @brief: 初始化2.5d map
    * @params: 输入的下采样点云 
    * @return: 输出2.5d map
    */
    inline grid_map::GridMap init_grid_map(GridMapConfig& grid_map_config);


    /*
    * @brief:  map 增加 elevation层
    * @params: 输入的下采样点云 , map 
    * @return: 输出map
    */
    inline grid_map::GridMap init_elevation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        grid_map::GridMap& grid_map);

    /*
    * @brief:  使用 FIESTA 增量更新 esdf层 
    * @params: 输入的下采样点云 , map 
    * @return: 输出map
    */
    inline grid_map::GridMap init_esdf_map_layer(grid_map::GridMap& grid_map,
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        GridMapConfig& grid_map_config);

    /*
    * @brief:  map 增加 障碍物层
    * @params: 输入的下采样点云 , map 
    * @return: 输出map
    */
    inline grid_map::GridMap init_obstacles_map_layer(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        grid_map::GridMap& grid_map,
        float min_obstacle_height,
        float max_obstacle_height);

    //TODO:
    /*
    * @brief:  map 增加 膨胀层
    * @params: map (含障碍物层)
    * @return: 输出map
    */
    inline grid_map::GridMap init_inflation_map_layer(
        grid_map::GridMap& grid_map);

    // TODO:增加 语义层
    /*
    * @brief:  map 增加 语义层
    * @params:  map 
    * @return: 输出map
    */
    inline grid_map::GridMap init_Semantics_map_layer(
        grid_map::GridMap& grid_map);


    //======================================================//
    //======================= tool =========================//
    //======================================================//

    //TODO:
    /*
    * @brief: grid_map 的输入层  膨胀 
    */
    inline void inflate_layer(grid_map::GridMap& grid_map,
        const std::string& sourceLayer,
        const std::string& targetLayer,
        double inflation_radius);

    /* 
    *  @return: x*x 
    */
    inline double squared(double x);

    /*
    *@brief: 双扫描法更新esdf
    */
    inline void dual_scan_update_esdf(grid_map::GridMap& grid_map,
        const std::string& inputLayer,
        const std::string& outputLayer);

    /*
    * @brief: fiesta_esdf_map to GridMap esdf layer 
    */
    inline void FiestaESDFMap_to_GridMap(fiesta::ESDFMap& fiesta_esdf_map,
        grid_map::GridMap& grid_map,
        const std::string& layer_name);
}// namespace planner


#endif