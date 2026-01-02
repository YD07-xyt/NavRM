#include "planner_env/core/processing.hpp"

namespace planner {
 pcl::PCLPointCloud2 point_cloud_filters(pcl::PCLPointCloud2::Ptr pcl_cloud,float voxel_leaf_size){
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size); // 体素大小，单位m，值越大降采样越明显
    pcl::PCLPointCloud2 output_pcl;
    sor.filter(output_pcl);
    return output_pcl;
 }
}