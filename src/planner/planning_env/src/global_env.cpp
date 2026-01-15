#include "planning_env/global_env.hpp"

namespace planner {
    void GlobalEnv::init_config(GlobalEnvConfig& global_env_config)
    {
        this->global_env_config = global_env_config;
    }
    grid_map::GridMap GlobalEnv::get_global_map()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_after_filters;
        read_pcd(pcd_cloud, global_env_config.pcd_file_path);
        point_cloud_filters(pcd_cloud,
            pcd_cloud_after_filters,
            global_env_config.voxel_leaf_size);
        global_map = init_grid_map(global_env_config.grid_map_config);
        //global_map=init_esdf_map_layer(global_map,pcd_cloud,global_env_config.grid_map_config);
        global_map = init_obstacles_map_layer(pcd_cloud_after_filters,
            global_map,
            global_env_config.grid_map_config.min_obstacle_height,
            global_env_config.grid_map_config.max_obstacle_height);
        global_map = init_elevation(pcd_cloud_after_filters, global_map);

        return this->global_map;
    }
}// namespace planner