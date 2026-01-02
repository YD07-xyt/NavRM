#pragma once

#ifndef GLOBAL_ENV_HPP
#define GLOBAL_ENV_HPP
#include "planner_env/config.hpp"
#include <grid_map_core/GridMap.hpp>

//pcl

namespace planner {
    class GlobalEnv
    {
    public:
        explicit GlobalEnv(GlobalEnvConfig global_env_config);
        void get_glovbal_env();
        void get_esdf_value();

    private:
        grid_map::GridMap global_map;
    };
}// namespace planner
#endif