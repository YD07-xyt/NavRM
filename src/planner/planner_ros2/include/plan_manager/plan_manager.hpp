#pragma once 

#ifndef _PLAN_MANAGER_HPP_
#define _PLAN_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include "planner_map.hpp"
#include "jps_planner/jps_planner.h"
#include "optimizer.h"
#include"visualizer/visualizer.hpp"
#include"yaml_io.hpp"
#include"../../../../tool/rm_log/include/glog.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "tf2/transform_datatypes.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.hpp"  // 工具函数，如getYaw等
#include "std_msgs/msg/bool.hpp"
#include <thread>
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>


enum StateMachine {
  INIT,           // 初始化状态
  IDLE,           // 空闲状态，等待目标
  PLANNING,       // 正在规划路径
  REPLAN,         // 重新规划路径（遇到障碍物或规划失败）
  GOINGTOGOAL,    // 执行路径，向目标移动
  EMERGENCY_STOP, // 紧急停止（检测到障碍物）
};


class PlanManager
{
  private:

    std::shared_ptr<planner::map::Map> sdfmap_;
    std::shared_ptr<MSPlanner> msplanner_;
    std::shared_ptr<JPS::JPSPlanner> jps_planner_;

    std::shared_ptr<Visualizer> visualizer_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
    
    rclcpp::TimerBase::SharedPtr main_thread_timer_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    // ros::Publisher mpc_polynome_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr record_pub_;
    rclcpp::Time current_time_;

    Eigen::Vector3d current_state_XYTheta_;
    Eigen::Vector3d current_state_VAJ_;
    Eigen::Vector3d current_state_OAJ_;

    //规划开始时间
    double plan_start_time_;

    //规划开始状态 (x,y,theta)
    Eigen::Vector3d plan_start_state_XYTheta;
    //速度、加速度、加加速度
    Eigen::Vector3d plan_start_state_VAJ;
    //位置、姿态、角速度？(可能是Orientation, Angular, Jerk)
    Eigen::Vector3d plan_start_state_OAJ;

    // 目标状态 (x,y,theta)
    Eigen::Vector3d goal_state_;    
    
    // 轨迹开始时间 用于replan?
    rclcpp::Time Traj_start_time_;
    // 轨迹总时间
    double Traj_total_time_;

    rclcpp::Time  loop_start_time_;

    //是否有odom
    bool have_geometry_;
    //是否有目标点
    bool have_goal_;
    //判断是否yaml文件参数固定最终点
    bool if_fix_final_;
    
    Eigen::Vector3d final_state_;

    double replan_time_;
    
    double max_replan_time_;

    double predicted_traj_start_time_;

    StateMachine state_machine_ = StateMachine::INIT;

  public:
  // parameters
    Config conf;
    MSPConfig msp_config;
    PenaltyWeights penaltyWt;
    PathpenaltyWeights PathpenaltyWt;
    PathLbfgsParams path_lbfgs_params;
    JPS::JPSPlannerParams jps_config;
    planner::OccupancyGridMapConfig occmap_config;

    ConfigReader config_reader_;
    void get_params();
    void get_params(const rclcpp::Node::SharedPtr node);
  public:
    explicit PlanManager(rclcpp::Node::SharedPtr node);

    ~PlanManager(){  

      sdfmap_->occupancy_grid_map.~OccupancyGridMap();
      visualizer_->~Visualizer();
    }

    void printStateMachine(){
      if(state_machine_ == INIT) {
        RCLCPP_INFO(node_->get_logger(),"state_machine_ == INIT");
      }
      if(state_machine_ == IDLE) {
        RCLCPP_INFO(node_->get_logger(),"state_machine_ == IDLE");
      }
      if(state_machine_ == PLANNING) 
      {
        RCLCPP_INFO(node_->get_logger(),"state_machine_ == PLANNING");
      }
      if(state_machine_ == REPLAN) {
        RCLCPP_INFO(node_->get_logger(),"state_machine_ == REPLAN");
      }
    }

    // void GeometryCallback(const carstatemsgs::CarState::ConstPtr &msg){
    //   have_geometry_ = true;
    //   current_state_XYTheta_ << msg->x, msg->y, msg->yaw;
    //   current_state_VAJ_ << msg->v, msg->a, msg->js;
    //   current_state_OAJ_ << msg->omega, msg->alpha, msg->jyaw;
    //   current_time_ = msg->Header.stamp;
    // }

    void GeometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);


    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


    //主要规划实现
     void MainThread();


    bool findJPSRoad();

    void MPCPathPub(const double& traj_start_time){
      Eigen::MatrixXd initstate = msplanner_->get_current_iniState();
      Eigen::MatrixXd finState = msplanner_->get_current_finState();
      Eigen::MatrixXd finalInnerpoints = msplanner_->get_current_Innerpoints();
      Eigen::VectorXd finalpieceTime = msplanner_->get_current_finalpieceTime();
      Eigen::Vector3d iniStateXYTheta = msplanner_->get_current_iniStateXYTheta();

      // carstatemsgs::Polynome polynome;
      // polynome.header.frame_id = "world";
      // polynome.header.stamp = this->now();
      // polynome.init_p.x = initstate.col(0).x();
      // polynome.init_p.y = initstate.col(0).y();
      // polynome.init_v.x = initstate.col(1).x();
      // polynome.init_v.y = initstate.col(1).y();
      // polynome.init_a.x = initstate.col(2).x();
      // polynome.init_a.y = initstate.col(2).y();
      // polynome.tail_p.x = finState.col(0).x();
      // polynome.tail_p.y = finState.col(0).y();
      // polynome.tail_v.x = finState.col(1).x();
      // polynome.tail_v.y = finState.col(1).y();
      // polynome.tail_a.x = finState.col(2).x();
      // polynome.tail_a.y = finState.col(2).y();

      // if(plan_start_time_ < 0) polynome.traj_start_time = this->now();
      // else polynome.traj_start_time = rclcpp::Time(plan_start_time_);

      // for(u_int i=0; i<finalInnerpoints.cols(); i++){
      //   geometry_msgs::msg::Vector3 point;
      //   point.x = finalInnerpoints.col(i).x();
      //   point.y = finalInnerpoints.col(i).y();
      //   point.z = 0.0;
      //   polynome.innerpoints.push_back(point);
      // }
      // for(u_int i=0; i<finalpieceTime.size(); i++){
      //   polynome.t_pts.push_back(finalpieceTime[i]);
      // }
      // polynome.start_position.x = iniStateXYTheta.x();
      // polynome.start_position.y = iniStateXYTheta.y();
      // polynome.start_position.z = iniStateXYTheta.z();

      // if(!msplanner_->if_standard_diff_){
      //   polynome.ICR.x = msplanner_->ICR_.x();
      //   polynome.ICR.y = msplanner_->ICR_.y();
      //   polynome.ICR.z = msplanner_->ICR_.z();
      // }
      
      // mpc_polynome_pub_->publish(polynome);


      
    }

};


#endif