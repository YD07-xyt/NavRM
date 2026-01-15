#pragma once

#ifndef GLOBAL_ENV_HPP
#define GLOBAL_ENV_HPP
#include "planning_env/config.hpp"
#include "planning_env/core/io.hpp"
#include"planning_env/core/processing.hpp"
//pcl

namespace planner {
    class GlobalEnv
    {
    public:
        explicit GlobalEnv(GlobalEnvConfig global_env_config);
        void get_esdf_value();
        
    private:
        grid_map::GridMap global_map;
        GlobalEnvConfig global_env_config;
        
        //void
    };
}// namespace planner
#endif