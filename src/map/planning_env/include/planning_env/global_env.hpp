#pragma once


#ifndef GLOBAL_ENV_HPP
#define GLOBAL_ENV_HPP
#include "planning_env/config.hpp"

#include "planning_env/core/io.hpp"

#include"planning_env/core/processing.hpp"

#include"planning_env/grid_map/grid_map.hpp"

#include "planning_env/voronoi/voronoi.hpp"

namespace planner {
    class GlobalEnv
    {
    public:
        grid_map::GridMap get_global_map();
        void init_config(const GlobalEnvConfig& global_env_config);
    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_point_cloud;
    private:
        grid_map::GridMap global_map;
        //fiesta::ESDFMap esdf_map;
        //voronoi::VoronoiMap voronoi_map;
        
        GlobalEnvConfig global_env_config;
    };
}// namespace planner
#endif