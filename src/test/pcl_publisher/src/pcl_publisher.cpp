#include "pcl_publisher/pcl_publisher.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace test {
    PclPublisherNode::PclPublisherNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("pcl_publisher_node")
    {
        test_lidar_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar",
                10);
        this->pcd_path = "src/test/pcl_publisher/pcd/test.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto test = pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *pcd_cloud);
        if (test == -1) {
            RCLCPP_ERROR(this->get_logger(),
                "无法读取PCD文件:%s",
                pcd_path.c_str());
        }
        pcl::toROSMsg(*pcd_cloud, ros2_pcl_cloud);

        ros2_pcl_cloud.header.frame_id = "map";
        ros2_pcl_cloud.header.stamp = this->get_clock()->now();
        double publish_period = 0.5;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_period),
            [this] { timer_callback(); });
        test_lidar_pub_->publish(ros2_pcl_cloud);
    }
    void PclPublisherNode::timer_callback()
    {
        // 更新消息时间戳（每次发布都使用当前时间，避免时间戳固定）
        ros2_pcl_cloud.header.stamp = this->get_clock()->now();

        test_lidar_pub_->publish(ros2_pcl_cloud);
    }
}// namespace test