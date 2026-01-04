#pragma once
#ifndef PROCESSING_HPP
#define PROCESSING_HPP

#include <pcl/filters/voxel_grid.h>
#include"grid_map_core/GridMap.hpp"
#include"planner_env/config.hpp"
namespace planner {
    /*
    * @params: 输入输出点云 ，体素滤波分辨率
    * @return: 下采样的输出点云
    */
    void point_cloud_filters(pcl::PCLPointCloud2::Ptr input_pcl,
        pcl::PCLPointCloud2 output_pcl,
        float voxel_leaf_size);
    /*
    * @brief: 点云转2.5d map
    * @params: 输入点云 输出2.5d map
    */
    void init_grid_map(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        grid_map::GridMap grid_map, GridMapConfig grid_map_config);
    
    void init_esdf_map();
}// namespace planner


#endif