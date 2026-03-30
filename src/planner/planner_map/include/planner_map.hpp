#pragma once

#ifndef PLANNER_MAP_HPP
#define PLANNER_MAP_HPP

#include "config.hpp"
#include "grid_map.hpp"
#include "rc_esdf.h"
#include"../../../tool/rm_log/include/glog.hpp"


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include "tf2/utils.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  
#include<nav_msgs/msg/odometry.hpp>

namespace planner::map {

    class Map{
        public:
            Map(const OccupancyGridMapConfig &init_map_params,const rclcpp::Node::SharedPtr node);
            OccupancyGridMap occupancy_grid_map;
            //ESDFMap esdf_map;

        public:
            static std::shared_ptr<tf2_ros::Buffer> tf_buffer_ ;
            rclcpp::Node::SharedPtr node_;
            
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gridmap_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ESDF_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_gradESDF_;
            
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::TimerBase::SharedPtr occtimer_;
            rclcpp::TimerBase::SharedPtr esdftimer_;
            rclcpp::TimerBase::SharedPtr vistimer_;
             pcl::PointCloud<pcl::PointXYZ> cloud_;
        public:
            void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void Occupancycallback(){
                occupancy_grid_map.updateOccupancycallback();
            };
            void ESDFcallback(){
                occupancy_grid_map.updateESDFCallback();
            };

            void publish_gridmap();
            void publish_ESDF();
            void publish_ESDFGrad(){
                //TODO:
                return ;
            };
            void visCallback(){
                if(occupancy_grid_map.has_map_){
                        publish_gridmap();
                        // exit(0);
                }
                if(occupancy_grid_map.has_esdf_){
                        publish_ESDF();
                        // publish_ESDFGrad();
                }
            }
        private:

    };


    class ESDFMap
    {
    public:
        explicit ESDFMap(ESDFMapConfig params,
            std::vector<Eigen::Vector2d>& obs_points_body);
        std::tuple<bool, double, Eigen::Vector2d> query(
            const Eigen::Vector2d& obs_points_body);
        std::tuple<bool, double, Eigen::Vector2d> query(int obs_points_body_x,int obs_points_body_y);
    public:
        ESDFMapConfig params;
        std::vector<Eigen::Vector2d> obs_points_body_;

    private:
        rc_esdf::RcEsdfMap esdf_map;
        std::vector<Eigen::Vector2d> footprint;

    public:
        void update_esdf(std::vector<Eigen::Vector2d> footprint);
        void init_esdf_map(ESDFMapConfig param,
            std::vector<Eigen::Vector2d> footprint); 
    };

}// namespace planner

#endif