#pragma once

#ifndef GLOBAL_ENV_HPP
#define GLOBAL_ENV_HPP
#include "planning_env/config.hpp"
#include <grid_map_core/GridMap.hpp>
#include"planning_env/core/io.hpp"
//pcl

namespace planner {
    class GlobalEnv
    {
    public:
        explicit GlobalEnv(GlobalEnvConfig global_env_config);
        void get_global_env();
        void get_esdf_value();

    private:
        grid_map::GridMap global_map;
        GlobalEnvConfig global_env_config;
        //void 
    };
}// namespace planner
#endif