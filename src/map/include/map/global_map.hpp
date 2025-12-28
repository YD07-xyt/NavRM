#pragma once
#ifndef MAP_GLOBAL_MAP_HPP
#define MAP_GLOBAL_MAP_HPP
#include "map/map.hpp"

namespace map {
class GlobalMap: public Map , public rclcpp::Node {
public:
    explicit GlobalMap(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void init_global_map();

private:
    grid_map::GridMap get_global_map(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
    grid_map::GridMap get_esdf_layer(grid_map::GridMap map);
    bool grid_map_to_pgm(grid_map::GridMap map);
    bool pgm_to_grid_map(grid_map::GridMap map);
    // 继承
    grid_map::GridMap fill_nan_heights(grid_map::GridMap) override;
    MapConfig init_parameter() override;
    void read_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
    void point_cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) override;    
    
    /*data*/
    grid_map::GridMap global_map;
    MapConfig global_map_config;
    std::string pcd_file_path;
};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map::GlobalMap)

#endif  // MAP_GLOBAL_MAP_HPP