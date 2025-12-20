#pragma once


// small_gicp
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
//pcl
#include<pcl/io/pcd_io.h>

namespace relocation {
    struct GicpConfig {
        std::string pcl_path;
        int num_threads;
        int num_neighbors;
        float leaf_size;
        float max_dist_sq;
    };
    class SmallGicp : public rclcpp::Node
    {
    public:
        explicit SmallGicp();
        void relocation_gicp(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw_target,
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw_source);

    private:
        GicpConfig init_parameter();
        pcl::PointCloud<pcl::PointXYZ>::Ptr read_pcd();
        void pcd_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg);
        void publish_transform();
        
        /*data*/
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
        GicpConfig small_gicp_config;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr align_pcd_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reloc_pose_pub_;
        
    };
}// namespace relocation
