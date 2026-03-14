#include "visualizer/visualizer.hpp"

Visualizer::Visualizer(rclcpp::Node::SharedPtr node):node_(node){
    kinoastarPubPCL = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/visualizer/kinoastarPathPCL", 10);
    kinoastarPubPath = node_->create_publisher<nav_msgs::msg::Path>("/visualizer/kinoastarPath", 10);
    finalnodePubMarker = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/finalnode", 10);
    mincoPathPath = node_->create_publisher<nav_msgs::msg::Path>("/visualizer/mincoPath", 10);
    mincoPointMarker = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/mincoPoint", 10);
}