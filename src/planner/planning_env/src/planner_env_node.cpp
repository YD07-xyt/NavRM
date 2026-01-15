#include "planning_env/planner_env_node.hpp"

namespace planner {
    PlannerEnvNode::PlannerEnvNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("planner_env_node")
    {
        init_global_map_params();
        init_local_map_params();
        global_env_.init_config(global_map_config);
        grid_map::GridMap global_map = global_env_.get_global_map();
        global_map_sub_ =
            this->create_publisher<grid_map_msgs::msg::GridMap>("/global_map",
                10);
        point_downsampled_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar",
                10,
                [this](const sensor_msgs::msg::PointCloud2 msg) {
                    this->pcl_cloud_callback(std::forward<decltype(msg)>(msg));
                });
    }
    void PlannerEnvNode::init_global_map_params()
    {
        this->declare_parameter<std::string>("global_map.map_frame", "map");
        this->declare_parameter<double>("global_map.resolution", 0.1);
        this->declare_parameter<double>("global_map.map_postion_x", 0.0);
        this->declare_parameter<double>("global_map.map_postion_y", 0.0);
        this->declare_parameter<double>("global_map.map_postion_z", 0.0);
        this->declare_parameter<double>("global_map.map_size_x", 10.0);
        this->declare_parameter<double>("global_map.map_size_y", 10.0);
        this->declare_parameter<double>("global_map.map_size_z", 10.0);
        this->declare_parameter<double>("global_map.max_obstacle_height", 0.1);
        this->declare_parameter<double>("global_map.min_obstacle_height", 0.1);
        this->declare_parameter<double>("global_map.ground_height_threshold",
            0.05);
        this->declare_parameter<double>("global_map.voxel_leaf_size", 0.05);
        this->declare_parameter<double>("global_map.max_point_height", 2.0);
        this->declare_parameter<std::string>("pcd",
            "src/bringup/pcd/rmuc_2025.pcd");
        this->global_map_config.grid_map_config.map_frame =
            this->get_parameter("global_map.map_frame").as_string();
        this->global_map_config.grid_map_config.resolution =
            this->get_parameter("global_map.resolution").as_double();
        this->global_map_config.grid_map_config.map_postion_z =
            this->get_parameter("global_map.map_postion_x").as_double();
        this->global_map_config.grid_map_config.map_postion_y =
            this->get_parameter("global_map.map_postion_y").as_double();
        this->global_map_config.grid_map_config.map_postion_z =
            this->get_parameter("global_map.map_postion_z").as_double();
        this->global_map_config.grid_map_config.map_size_x =
            this->get_parameter("global_map.map_size_x").as_double();
        this->global_map_config.grid_map_config.map_size_y =
            this->get_parameter("global_map.map_size_y").as_double();
        this->global_map_config.grid_map_config.map_size_z =
            this->get_parameter("global_map.map_size_z").as_double();
        this->global_map_config.grid_map_config.max_obstacle_height =
            this->get_parameter("global_map.max_obstacle_height").as_double();
        this->global_map_config.grid_map_config.min_obstacle_height =
            this->get_parameter("global_map.min_obstacle_height").as_double();
        this->global_map_config.grid_map_config.ground_height_threshold =
            this->get_parameter("global_map.ground_height_threshold")
                .as_double();
        this->global_map_config.voxel_leaf_size =
            this->get_parameter("global_map.voxel_leaf_size").as_double();
        this->global_map_config.max_point_height =
            this->get_parameter("global_map.max_point_height").as_double();
        this->global_map_config.pcd_file_path =
            this->get_parameter("pcd").as_string();
    }
    void PlannerEnvNode::init_local_map_params()
    {
        this->declare_parameter<std::string>("local_map.map_frame", "map");
        this->declare_parameter<double>("local_map.resolution", 0.1);
        this->declare_parameter<double>("local_map.map_postion_x", 0.0);
        this->declare_parameter<double>("local_map.map_postion_y", 0.0);
        this->declare_parameter<double>("local_map.map_postion_z", 0.0);
        this->declare_parameter<double>("local_map.map_size_x", 10.0);
        this->declare_parameter<double>("local_map.map_size_y", 10.0);
        this->declare_parameter<double>("local_map.map_size_z", 10.0);
        this->declare_parameter<double>("local_map.max_obstacle_height", 0.1);
        this->declare_parameter<double>("local_map.min_obstacle_height", 0.1);
        this->declare_parameter<double>("local_map.ground_height_threshold",
            0.05);
        this->declare_parameter<double>("local_map.voxel_leaf_size", 0.05);
        this->declare_parameter<double>("local_map.max_point_height", 2.0);
        this->local_map_config.grid_map_config.map_frame =
            this->get_parameter("local_map.map_frame").as_string();
        this->local_map_config.grid_map_config.resolution =
            this->get_parameter("local_map.resolution").as_double();
        this->local_map_config.grid_map_config.map_postion_z =
            this->get_parameter("local_map.map_postion_x").as_double();
        this->local_map_config.grid_map_config.map_postion_y =
            this->get_parameter("local_map.map_postion_y").as_double();
        this->local_map_config.grid_map_config.map_postion_z =
            this->get_parameter("local_map.map_postion_z").as_double();
        this->local_map_config.grid_map_config.map_size_x =
            this->get_parameter("local_map.map_size_x").as_double();
        this->local_map_config.grid_map_config.map_size_y =
            this->get_parameter("local_map.map_size_y").as_double();
        this->local_map_config.grid_map_config.map_size_z =
            this->get_parameter("local_map.map_size_z").as_double();
        this->local_map_config.grid_map_config.max_obstacle_height =
            this->get_parameter("local_map.max_obstacle_height").as_double();
        this->local_map_config.grid_map_config.min_obstacle_height =
            this->get_parameter("local_map.min_obstacle_height").as_double();
        this->local_map_config.grid_map_config.ground_height_threshold =
            this->get_parameter("local_map.ground_height_threshold")
                .as_double();
        this->local_map_config.voxel_leaf_size =
            this->get_parameter("local_map.voxel_leaf_size").as_double();
        this->local_map_config.max_point_height =
            this->get_parameter("local_map.max_point_height").as_double();
    }
}// namespace planner