#pragma once
#ifndef IO_HPP
#define IO_HPP

#include <pcl/io/pcd_io.h>
#include <string>

#include"rm_log.hpp"
namespace planner {
    inline void read_stl();//TODO:
    /*
    *   @params 输出点云 ， pcd的文件路径
    */
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr read_pcd(std::string pcd_file_path)
    {   
        
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1) {
            spdlog::warn("Couldn't read file your_file.pcd");
        }
        else {
            spdlog::info("Success read file your_file.pcd");
        }
        return cloud;
    };
}// namespace planner

#endif