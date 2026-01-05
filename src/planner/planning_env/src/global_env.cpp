#include "planning_env/global_env.hpp"

namespace planner {
    GlobalEnv::GlobalEnv(GlobalEnvConfig global_env_config)
        : global_env_config(global_env_config)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud;
        read_pcd(pcd_cloud, global_env_config.pcd_file_path);

    }
    // void GlobalEnv::init_grid_map(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud,grid_map::GridMap grid_map){

    // }
}// namespace planner