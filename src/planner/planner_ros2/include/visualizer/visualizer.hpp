#pragma once

#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_


#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/transform_datatypes.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include<rclcpp/rclcpp.hpp>


#include "minco/trajectory.hpp"

// debug
#include <fstream>

class Visualizer
{
  private:

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kinoastarPubPCL;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr kinoastarPubPath;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr finalnodePubMarker;
    // ros::Publisher meshPub;
    // ros::Publisher edgePub;
    rclcpp::Node::SharedPtr node_;
  public:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mincoPathPath;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mincoPointMarker;
    
    explicit Visualizer(rclcpp::Node::SharedPtr node);

    ~Visualizer(){};

    // Only the end point
    void finalnodePub(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg){
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.ns = "finalnode";
      //TODO
      marker.lifetime =  rclcpp::Duration::from_seconds(1.5);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.3;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 0.3;
      marker.color.r = rand() / double(RAND_MAX);
      marker.color.g = rand() / double(RAND_MAX);
      marker.color.b = rand() / double(RAND_MAX);
      marker.header.stamp = node_->now();
      marker.id = 0;
      marker.pose.position.x = msg->pose.position.x;
      marker.pose.position.y = msg->pose.position.y;
      marker.pose.position.z = 0.15;
      marker.pose.orientation.w = msg->pose.orientation.w;
      marker.pose.orientation.x = msg->pose.orientation.x;
      marker.pose.orientation.y = msg->pose.orientation.y;
      marker.pose.orientation.z = msg->pose.orientation.z;
      finalnodePubMarker->publish(marker);
    }

    // the start point and end point
    void finalnodePub(const Eigen::Vector3d &init_point, const Eigen::Vector3d &final_point){
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.ns = "init_point";
      //TODO
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.15;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;
      marker.color.a = 0.8;
      // marker.color.r = rand() / double(RAND_MAX);
      // marker.color.g = rand() / double(RAND_MAX);
      // marker.color.b = rand() / double(RAND_MAX);
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.header.stamp = node_->now();
      marker.id = 0;
      marker.pose.position.x = init_point.x();
      marker.pose.position.y = init_point.y();
      marker.pose.position.z = 0.02;    
      tf2::Quaternion q;
      q.setRPY(0, 0, init_point.z());  // roll=0, pitch=0, yaw=init_point.z()
      marker.pose.orientation = tf2::toMsg(q);  
      finalnodePubMarker->publish(marker);

      marker.ns = "final_point";
      // marker.color.r = rand() / double(RAND_MAX);
      // marker.color.g = rand() / double(RAND_MAX);
      // marker.color.b = rand() / double(RAND_MAX);
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.pose.position.x = final_point.x();
      marker.pose.position.y = final_point.y();
      marker.pose.position.z = 0.02;      
      q.setRPY(0, 0, final_point.z());  // roll=0, pitch=0, yaw=init_point.z()
      marker.pose.orientation = tf2::toMsg(q);  
      finalnodePubMarker->publish(marker);
    }

    void initnodePub(const Eigen::Vector3d &point){
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.ns = "initnode";
      //TODO
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.2;
      marker.scale.y = 0.04;
      marker.scale.z = 0.04;
      marker.color.a = 0.7;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.header.stamp = node_->now();
      marker.id = 0;
      marker.pose.position.x = point.x();
      marker.pose.position.y = point.y();
      marker.pose.position.z = 0.02;      
      tf2::Quaternion q;
      q.setRPY(0, 0, point.z());  
      marker.pose.orientation = tf2::toMsg(q);  

      finalnodePubMarker->publish(marker);
    }

    void mincoPathPub(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta){
      double ini_x = start_state_XYTheta.x();
      double ini_y = start_state_XYTheta.y();

      double s1;
      int K = 50;
      int SamNumEachPart = 2 * K;
      double sumT = 0.0;

      int TrajNum = final_traj.getPieceNum();
      Eigen::VectorXd pieceTime = final_traj.getDurations();

      std::vector<Eigen::VectorXd> VecIntegralX(TrajNum);
      std::vector<Eigen::VectorXd> VecIntegralY(TrajNum);
      std::vector<Eigen::VectorXd> VecYaw(TrajNum);
      std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum+1);
      VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

      for(int i=0; i<TrajNum; i++){
        double step = pieceTime[i] / K;
        double halfstep = step / 2.0;
        double CoeffIntegral = pieceTime[i] / K / 6.0;

        Eigen::VectorXd IntegralX(K);IntegralX.setZero();
        Eigen::VectorXd IntegralY(K);IntegralY.setZero();
        Eigen::VectorXd Yaw(K);Yaw.setZero();
        s1 = 0.0;
        for(int j=0; j<=SamNumEachPart; j++){
          if(j%2 == 0){
            Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
            Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
            s1 += halfstep;
            if(j!=0){
              IntegralX[j/2-1] += CoeffIntegral * currVel.y() * cos(currPos.x());
              IntegralY[j/2-1] += CoeffIntegral * currVel.y() * sin(currPos.x());
              Yaw[j/2-1] = currPos.x();
            }
            if(j!=SamNumEachPart){
              IntegralX[j/2] += CoeffIntegral * currVel.y() * cos(currPos.x());
              IntegralY[j/2] += CoeffIntegral * currVel.y() * sin(currPos.x());
            }
          }
          else{
            Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
            Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
            s1 += halfstep;
            IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
            IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
          }
        }
        VecIntegralX[i] = IntegralX;
        VecIntegralY[i] = IntegralY;
        VecYaw[i] = Yaw;
        // VecTrajFinalXY[i+1] = Eigen::Vector2d(IntegralX[IntegralX.size()-1], IntegralY[IntegralX.size()-1]);
        sumT += pieceTime[i];
      }

      nav_msgs::msg::Path path;
      path.header.frame_id = "world";
      path.header.stamp = node_->now();
      Eigen::Vector2d pos(ini_x, ini_y);
      for(u_int i=0; i<VecIntegralX.size(); i++){
        for(u_int j=0; j<VecIntegralX[i].size(); j++){
          pos.x() += VecIntegralX[i][j];
          pos.y() += VecIntegralY[i][j];
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = "world";
          pose.header.stamp = node_->now();
          pose.pose.position.x = pos.x();
          pose.pose.position.y = pos.y();
          pose.pose.position.z = 0.15;
          tf2::Quaternion q;
          q.setRPY(0, 0, VecYaw[i][j]);  // roll=0, pitch=0, yaw=init_point.z()
          pose.pose.orientation = tf2::toMsg(q);  
          path.poses.push_back(pose);
        }
      }
      mincoPathPath->publish(path);
      RCLCPP_INFO(node_->get_logger(),"\033[40;33m iter real finStateXY:%f  %f  \033[0m", pos.x(), pos.y());
    }

};



#endif