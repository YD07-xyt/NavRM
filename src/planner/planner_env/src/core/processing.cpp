#include "planner_env/core/processing.hpp"

namespace planner {
    void point_cloud_filters(pcl::PCLPointCloud2::Ptr input_pcl,
        pcl::PCLPointCloud2 output_pcl,
        float voxel_leaf_size)
    {
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(input_pcl);
        sor.setLeafSize(voxel_leaf_size,
            voxel_leaf_size,
            voxel_leaf_size);// 体素大小，单位m，值越大降采样越明显
        sor.filter(output_pcl);
    }
    void init_grid_map(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        grid_map::GridMap grid_map,
        GridMapConfig grid_map_config)
    {
        grid_map.setFrameId(grid_map_config.map_frame);
        grid_map.setGeometry(grid_map::Length(grid_map_config.map_size_x,
                                 grid_map_config.map_size_y),
            grid_map_config.resolution,
            grid_map::Position(grid_map_config.map_postion_x,
                grid_map_config.map_size_y));// 地图尺寸、分辨率、原点
            grid_map.add("elevation");           // 高程层
            grid_map.add("Semantics");           // 语义层
            grid_map.add("obstacles");           // 障碍物层 (0:可通行, 1:障碍物)
            grid_map.add("inflation");           // 膨胀层
            grid_map.add("esdf");                //esdf层
            for (const auto& point : point_cloud->points) {
        grid_map::Position pos(point.x, point.y);
        if (grid_map.isInside(pos)) {
            grid_map.atPosition("elevation", pos) = point.z;
        }
    }
    }
}// namespace planner