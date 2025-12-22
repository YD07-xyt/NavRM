#pragma once


// small_gicp
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

//ros
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
//pcl
#include <pcl/io/pcd_io.h>
namespace relocation {
    struct GicpConfig {
        std::string map_frame;
        std::string odom_frame;
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
        small_gicp::RegistrationResult relocation_gicp(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw_source);

    private:
        GicpConfig init_parameter();

        pcl::PointCloud<pcl::PointXYZ>::Ptr read_pcd();

        void pcd_callback(
            const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg);

        void publish_transform(Eigen::Isometry3d result_t,
            rclcpp::Time last_scan_time);

        /*data*/
        GicpConfig small_gicp_config;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        Eigen::Isometry3d result_t;

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcd;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcd;
        pcl::PointCloud<pcl::PointCovariance>::Ptr target;
        pcl::PointCloud<pcl::PointCovariance>::Ptr source;

        std::shared_ptr<
            small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
            target_tree;
        std::shared_ptr<
            small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
            source_tree;
        std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor,
            small_gicp::ParallelReductionOMP>>
            registration;
        //Eigen::Isometry3d  revious_result_t;
    };
}// namespace relocation
