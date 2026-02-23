#pragma once


#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP
#include "Eigen/Core"
#include "config.hpp"
#include <expected>
#include <mdspan>
//pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace planner {
    enum GridState {
        FREE = 0,      // 空闲（可通过）
        OCCUPIED = 100,// 占用（障碍物）
        UNKNOWN = -1   // 未知区域
    };

    class OccupancyGridMap
    {
    public:
        OccupancyGridMap();
        void updateOccupancy(Eigen::Vector3d odom_pos);

    public:
        double x_upper_ = -std::numeric_limits<double>::infinity(),
               y_upper_ = -std::numeric_limits<double>::infinity();
        double x_lower_ = std::numeric_limits<double>::infinity(),
               y_lower_ = std::numeric_limits<double>::infinity();

    private:
        
    private:
        bool occ_need_update_;
        OccupancyGridMapConfig map_params;
        double X_SIZE_,Y_SIZE_,XY_SIZE_;
        int GLX_SIZE_, GLY_SIZE_,GLXY_SIZE_;

        double inv_grid_interval_= 1/map_params.resolution;
    private:
        bool isInGlobalMap(const Eigen::Vector2d& pt);
        void raycastProcess(
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud,Eigen::Vector3d odom_pos);
        // 从给定位置指向目标点的射线与地图边界的交点，返回地图边界上的最近点
        Eigen::Vector2d closetPointInMap(const Eigen::Vector2d& pt,
            const Eigen::Vector2d& pos);
        Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d& pt)
    };

    enum class PointError {

    };
}// namespace planner

#endif