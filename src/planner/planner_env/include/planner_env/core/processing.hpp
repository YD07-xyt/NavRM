#pragma once
#ifndef PROCESSING_HPP
#define PROCESSING_HPP

#include <pcl/filters/voxel_grid.h>

namespace planner {
    /*
    * @params: 点云 ，体素滤波分辨率
    * @return: 下采样的点云
    */
    pcl::PCLPointCloud2 point_cloud_filters(pcl::PCLPointCloud2::Ptr pcl_cloud,
        float voxel_leaf_size);

}// namespace planner


#endif