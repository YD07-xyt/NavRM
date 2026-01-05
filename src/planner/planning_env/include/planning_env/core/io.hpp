#pragma once
#ifndef IO_HPP
#define IO_HPP

#include <pcl/io/pcd_io.h>
#include <string>
namespace planner {
    inline void read_stl();
    /*
    *   @params 输出点云 ， pcd的文件路径
    */ 
    inline void read_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string pcd_file_path);
}// namespace planner

#endif