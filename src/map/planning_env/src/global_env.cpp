#include "planning_env/global_env.hpp"

namespace planner {
    void GlobalEnv::init_config(const GlobalEnvConfig& global_env_config)
    {
        this->global_env_config = global_env_config;
    }

    grid_map::GridMap GlobalEnv::get_global_map()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud =
            read_pcd(global_env_config.pcd_file_path);

        this->global_point_cloud =
            point_cloud_filters(pcd_cloud, global_env_config.voxel_leaf_size);

        global_map = grid_env::init_grid_map(global_env_config.grid_map_config);
        //global_map=init_esdf_map_layer(global_map,pcd_cloud,global_env_config.grid_map_config);

        grid_env::init_obstacles_map_layer(global_point_cloud,
            global_map,
            global_env_config.grid_map_config.min_obstacle_height,
            global_env_config.grid_map_config.max_obstacle_height);

        grid_env::init_elevation_layer(global_point_cloud, global_map);

        return this->global_map;
    }
}// namespace planner