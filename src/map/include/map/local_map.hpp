#include "map/map.hpp"

namespace map {
class LocalMap: public Map , rclcpp::Node{
public:
    LocalMap();
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