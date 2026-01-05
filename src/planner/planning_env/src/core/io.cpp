#include "planning_env/core/io.hpp"


namespace planner {
    inline void read_pcd( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string pcd_file_path)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) ==
            -1) {
            PCL_ERROR("Couldn't read file your_file.pcd\n");
        } else {
            PCL_INFO("Success read file your_file.pcd\n");
        }
    }
}// namespace planner