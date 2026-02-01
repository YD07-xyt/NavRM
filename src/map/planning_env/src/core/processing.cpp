#include "planning_env/core/processing.hpp"

#include <algorithm>
#include <grid_map_core/iterators/SlidingWindowIterator.hpp>


namespace planner {


    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const float voxel_leaf_size)
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (input_cloud == nullptr) {
            spdlog::error("point_cloud_filters传入的input_cloud为空");
            output_cloud = input_cloud;
        }

        pcl::VoxelGrid<pcl::PointXYZ> sor;

        sor.setInputCloud(input_cloud);

        sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        sor.filter(*output_cloud);
        return output_cloud;
    }
}// namespace planner