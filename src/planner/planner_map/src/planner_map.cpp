#include "../include/planner_map.hpp"
#include <chrono>
#include<functional>

namespace planner {
    namespace map {
 
 
    ESDFMap::ESDFMap(ESDFMapConfig params,
        std::vector<Eigen::Vector2d>& obs_points_body)
        : params(params),obs_points_body_(obs_points_body)
    {

        ESDFMap::init_esdf_map(ESDFMap::params, ESDFMap::footprint);
    };
    void ESDFMap::init_esdf_map(ESDFMapConfig param,
        std::vector<Eigen::Vector2d> footprint)
    {
        ESDFMap::esdf_map.initialize(param.width_m,
            param.height_m,
            param.resolution);
        ESDFMap::esdf_map.generateFromPolygon(footprint);
    };


    void ESDFMap::update_esdf(std::vector<Eigen::Vector2d> footprint)
    {
        ESDFMap::esdf_map.generateFromPolygon(footprint);
    }


    std::tuple<bool, double, Eigen::Vector2d> ESDFMap::query(
        const Eigen::Vector2d& obs_points_body)
    {
        bool in_box;
        double dist;
        Eigen::Vector2d grad;
        // 核心查询函数
        in_box = ESDFMap::esdf_map.query(obs_points_body, dist, grad);
        return std::make_tuple(in_box, dist, grad);
    }
    std::tuple<bool, double, Eigen::Vector2d> ESDFMap::query(int obs_points_body_x,
        int obs_points_body_y)
    {
        bool in_box;
        double dist;
        Eigen::Vector2d grad;
        // 核心查询函数
        in_box = ESDFMap::esdf_map.query(
            Eigen::Vector2d(obs_points_body_x, obs_points_body_y),
            dist,
            grad);
        return std::make_tuple(in_box, dist, grad);
    };

    Map::Map(const OccupancyGridMapConfig &init_map_params,const rclcpp::Node::SharedPtr node){
        occupancy_grid_map.map_params = init_map_params;
        
        occtimer_ = node->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&Map::Occupancycallback, this));
        
        esdftimer_ = node->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&Map::ESDFcallback, this));

        vistimer_ = node->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Map::visCallback, this));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock(),tf2::durationFromSec(20.0));
        
    }

    void Map::publish_gridmap(){
        pcl::PointCloud<pcl::PointXYZI> cloud_vis;
        sensor_msgs::msg::PointCloud2 map_vis;
        for(int idx = 1;idx < occupancy_grid_map.GLXY_SIZE_;idx++){
            
            // if(gridmap_[idx]==Unoccupied){
            //   Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
            //   pcl::PointXYZI pt;
            //   pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
            //   pt.intensity = 8.0;
            //   cloud_vis.points.push_back(pt);
            // }

            if(occupancy_grid_map.gridmap_[idx]==GridState::OCCUPIED){
                Eigen::Vector2d corrd = occupancy_grid_map.gridIndex2coordd(occupancy_grid_map.vectornum2gridIndex(idx));
                pcl::PointXYZI pt;
                pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
                pt.intensity = 0.0;
                cloud_vis.points.push_back(pt);
            }
            
            if(occupancy_grid_map.gridmap_[idx]==GridState::UNKNOWN){
                Eigen::Vector2d corrd = occupancy_grid_map.gridIndex2coordd(occupancy_grid_map.vectornum2gridIndex(idx));
                pcl::PointXYZI pt;
                pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
                pt.intensity = 8.0;
                cloud_vis.points.push_back(pt);
            }

        }

        pcl::PointXYZI pt;
        pt.x = 100.0; pt.y = 100.0; pt.z = 0.1;
        pt.intensity = 10.0;
        cloud_vis.points.push_back(pt);

        cloud_vis.width = cloud_vis.points.size();
        cloud_vis.height = 1;
        cloud_vis.is_dense = true;
        pcl::toROSMsg(cloud_vis, map_vis);
        map_vis.header.frame_id = "world";
        pub_gridmap_->publish(map_vis);
}

void Map::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  static tf2_ros::TransformListener tf_listener(*tf_buffer_);

  // Get transformation information
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
      transformStamped = tf_buffer_->lookupTransform("world", msg->header.frame_id,
                                                  msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  }catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      return;
  }

  occupancy_grid_map.odom_pos.head(2) = Eigen::Vector2d(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
  occupancy_grid_map.odom_pos[2] = tf2::getYaw(transformStamped.transform.rotation);

  // Perform coordinate transformation
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transformStamped);

  // Convert to PCL format
  pcl::fromROSMsg(transformed_cloud, *occupancy_grid_map.point_cloud);
  
  occupancy_grid_map.occ_need_update_ = true;
}

    }
} // namespace planner