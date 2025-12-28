#pragma once
//包含ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

//包含PCL相关头文件
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//包含Eigen
#include <Eigen/Core>

//包含grid_map相关头文件
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
namespace map {
    struct MapConfig {
        std::string map_frame;           //地图坐标系
        double resolution;               //栅格分辨率（米/格
        double map_size_x;               //地图X方向尺寸（米)
        double map_size_y;               //地图y方向尺寸（米)
        double obstacle_height_threshold;//障碍物高度阈值
        double ground_height_threshold;//地面高度阈值（用于区分地面/噪声）
        double voxel_leaf_size; //体素滤波分辨率（降采样）
        double max_point_height;//最大有效高度（过滤异常点)
    };

    /**   
 * @class 总的map
 * @brief 接受点云生成地图
 */
    class Map
    {
    protected:
        /** 
     * @brief  :ros2获取yaml文件的参数，
     * @return :mapconfig
    */
        virtual MapConfig init_parameter() = 0;
        /** 
     * @brief ：接受点云后预处理，获取map（订阅点云会回调函数）
     * @param :传入（ros2的msg::pointcloud2或pcl::pointcloud）的ptr
     */
        virtual void point_cloud_callback(
            sensor_msgs::msg::PointCloud2::SharedPtr msg) = 0;
        /**
    * @brief: 点云预处理
    */
        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_point_cloud(
            pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) = 0;
        /** 
    * @brief: 填充无数据区域的高度（用周围地面高度插值）
    */
        virtual grid_map::GridMap fill_nan_heights(grid_map::GridMap) = 0;
        /** 
    * @param point_cloud_sub 订阅点云
    * @param map_pub 发布map
    */
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
            point_cloud_sub;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub;
    };
}// namespace map