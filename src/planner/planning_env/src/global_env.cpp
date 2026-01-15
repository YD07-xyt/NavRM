#include "planning_env/global_env.hpp"

namespace planner {
    GlobalEnv::GlobalEnv(GlobalEnvConfig global_env_config)
        : global_env_config(global_env_config)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud;
        read_pcd(pcd_cloud, global_env_config.pcd_file_path);
        global_map=init_grid_map(global_env_config.grid_map_config);
        global_map=init_esdf_map_layer(global_map,pcd_cloud,global_env_config.grid_map_config);
        global_map=init_obstacles_map_layer(pcd_cloud, global_map, global_env_config.min_obstacle_height, global_env_config.max_obstacle_height);


    }
    // void GlobalEnv::init_grid_map(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud,grid_map::GridMap grid_map){

    // }
}// namespace planner