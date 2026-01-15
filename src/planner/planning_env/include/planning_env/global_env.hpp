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
        grid_map::GridMap get_global_map();
        void init_config(GlobalEnvConfig& global_env_config);
    private:
        grid_map::GridMap global_map;
        GlobalEnvConfig global_env_config;
    };
}// namespace planner
#endif