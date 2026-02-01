#pragma once
#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP
//grid_map
#include "grid_map_core/GridMap.hpp"
#include "planning_env/config.hpp"
//esdf
#include "planning_env/esdf/esdf_map.hpp"
//log
#include "rm_log.hpp"
//pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace planner::grid_env {

    /*
    * @brief: 初始化2.5d map
    * @params: 输入的下采样点云 
    * @return: 输出2.5d map
    */
    grid_map::GridMap init_grid_map(const GridMapConfig& grid_map_config);


    /*
    * @brief:  map 增加 elevation层
    * @params: 输入的下采样点云 , map 
    * @return: 输出map
    */
    void init_elevation_layer(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud,
        grid_map::GridMap& grid_map);

    /*
    * @brief:  使用 FIESTA 增量更新 esdf层 
    * @params: 输入的下采样点云 , map 
    * @return: 输出map
    */
    grid_map::GridMap init_esdf_map_layer(grid_map::GridMap& grid_map,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        const GridMapConfig& grid_map_config);

    /*
    * @brief:  map 增加 障碍物层
    * @params: 输入的下采样点云 , map 
    * @return: 输出map
    */
    void init_obstacles_map_layer(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        grid_map::GridMap& grid_map,
        const float min_obstacle_height,
        const float max_obstacle_height);

    //TODO:
    /*
    * @brief:  map 增加 膨胀层
    * @params: map (含障碍物层)
    * @return: 输出map
    */
    grid_map::GridMap init_inflation_map_layer(grid_map::GridMap& grid_map);

    // TODO:增加 语义层
    /*
    * @brief:  map 增加 语义层
    * @params:  map 
    * @return: 输出map
    */
    grid_map::GridMap init_Semantics_map_layer(grid_map::GridMap& grid_map);


    //======================================================//
    //======================= tool =========================//
    //======================================================//

    //TODO:
    /*
    * @brief: grid_map 的输入层  膨胀 
    */
    void inflate_layer(grid_map::GridMap& grid_map,
        const std::string& sourceLayer,
        const std::string& targetLayer,
        const double inflation_radius);

    /* 
    *  @return: x*x 
    */
    double squared(const double x);

    /*
    *@brief: 双扫描法更新esdf
    */
    void dual_scan_update_esdf(grid_map::GridMap& grid_map,
        const std::string& inputLayer,
        const std::string& outputLayer);

    /*
    * @brief: fiesta_esdf_map to GridMap esdf layer 
    */
    void FiestaESDFMap_to_GridMap(fiesta::ESDFMap& fiesta_esdf_map,
        grid_map::GridMap& grid_map,
        const std::string& layer_name);
}// namespace planner


#endif