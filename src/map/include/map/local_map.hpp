#pragma once 
#ifndef MAP_LOCAL_MAP_HPP
#define MAP_LOCAL_MAP_HPP
#include "map/map.hpp"
namespace map {
class LocalMap: public Map ,  public rclcpp::Node{
public:
   explicit LocalMap(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void init_local_map();

private:
    grid_map::GridMap get_local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
    grid_map::GridMap get_esdf_layer(grid_map::GridMap map);

    //
    grid_map::GridMap fill_nan_heights(grid_map::GridMap) override;
    MapConfig init_parameter() override;
    void point_cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) override;    
    grid_map::GridMap local_map;
    MapConfig Local_map_config;
};
} // namespace map


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map::LocalMap)
#endif  // MAP_LOCAL_MAP_HPP