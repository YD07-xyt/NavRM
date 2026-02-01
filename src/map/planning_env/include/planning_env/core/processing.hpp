#pragma once
#ifndef PROCESSING_HPP
#define PROCESSING_HPP

#include "grid_map_core/GridMap.hpp"
#include "planning_env/config.hpp"
#include "planning_env/esdf/esdf_map.hpp"
#include "rm_log.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace planner {
    /*
    * @params: 输入点云 ，体素滤波分辨率
    * @return: 下采样的输出点云
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const float voxel_leaf_size);

}// namespace planner


#endif