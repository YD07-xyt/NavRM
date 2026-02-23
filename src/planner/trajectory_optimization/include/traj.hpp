#pragma once

#ifndef TRAJECTORY_OPTIMIZATION_HPP
#define TRAJECTORY_OPTIMIZATION_HPP
#include"minco/minco.hpp"

namespace traj_smooth {
    class TrajectoryOptimization
    {
    public:
        
    private:
        minco::MINCO_S3NU minco_optimizer;
        minco::BandedSystem;
    };
}// namespace traj_smooth

#endif