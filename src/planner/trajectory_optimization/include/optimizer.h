#pragma once

#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include<Eigen/Core>

#include<rclcpp/rclcpp.hpp>
#include<visualization_msgs/msg/marker.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/transform_datatypes.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>



#include "../../planner_map/include/planner_map.hpp"
#include "../../path_searching/include/jps_planner/traj_representation.h"


#include "minco/trajectory.hpp"
#include "minco/minco.hpp"
#include "minco/lbfgs.hpp"


struct Config
{
    // kinematic constraints
    double max_vel_;
    double min_vel_;
    double max_acc_;
    double max_omega_;
    double max_domega_;
    double max_centripetal_acc_;

    bool if_directly_constrain_v_omega_;
    
    // Config(const ros::NodeHandle &nh_)
    // {
    //     nh_.param<double>(ros::this_node::getName()+ "/max_vel",max_vel_,5);
    //     nh_.param<double>(ros::this_node::getName()+ "/min_vel",min_vel_,-5);
    //     nh_.param<double>(ros::this_node::getName()+ "/max_acc",max_acc_,5);
    //     nh_.param<double>(ros::this_node::getName()+ "/max_domega",max_domega_,50);
    //     nh_.param<double>(ros::this_node::getName()+ "/max_centripetal_acc",max_centripetal_acc_,10000);
    //     nh_.param<double>(ros::this_node::getName()+ "/max_omega",max_omega_,1);
    //     nh_.param<bool>(ros::this_node::getName()+ "/if_directly_constrain_v_omega", if_directly_constrain_v_omega_, false);
    // }
};

struct PenaltyWeights{
    double time_weight;
    double time_weight_backup_for_replan;
    double acc_weight;
    double domega_weight;
    double collision_weight;
    double moment_weight;
    double mean_time_weight;
    double cen_acc_weight;
};

// For trajectory pre-processing
struct PathpenaltyWeights{
    double time_weight;
    double bigpath_sdf_weight;
    double mean_time_weight;
    double moment_weight;
    double acc_weight;
    double domega_weight;
};

// For trajectory pre-processing
struct PathLbfgsParams{
    lbfgs::lbfgs_parameter_t path_lbfgs_params;
    double normal_past;
    double shot_path_past;
    double shot_path_horizon;
};


struct MSPConfig{
    double mean_time_lowBound_;
    double mean_time_uppBound_;
    double smoothEps;// for smoothL1
    double safeDis_;
    double finalMinSafeDis;
    int finalSafeDisCheckNum;
    int safeReplanMaxTime;

    std::vector<double> _energyWeights;
    std::vector<double> _EqualLambda;
    std::vector<double> _EqualRho;
    std::vector<double> _EqualRhoMax;
    std::vector<double> _EqualGamma;
    std::vector<double> _EqualTolerance;
    std::vector<double> _CutEqualLambda;
    std::vector<double> _CutEqualRhoMax;
    std::vector<double> _CutEqualRho;
    std::vector<double> _CutEqualGamma;
    std::vector<double> _CutEqualTolerance;
    
    // sampling parameters
    int sparseResolution;
    double timeResolution;
    int mintrajNum;
        // Trajectory prediction resolution for get_the_predicted_state
    double trajPredictResolution;
    bool if_visual_optimization = false;
    
    bool hrz_limited_;
    double hrz_laser_range_dgr;

    //TODO:
    std::vector<double> checkpoint;

    double ICR_yl;
    double ICR_xv;
    double ICR_yr;

    bool if_standard_diff;
};

struct OptSharedData{

};


class MSPlanner
{
private:
    Config config_;
    std::shared_ptr<planner::map::Map> map_;
    rclcpp::Node::SharedPtr msp_planner_node_;
    rclcpp::Time time;
    // ros::NodeHandle nh_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mincoinitPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathmincoinitPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr CollisionpointPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processmincoinitPath;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pathmincoinitPoint;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mincoinitPoint;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr innerinitpositionsPoint;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr recordTextPub;


    // optimizer parameters
    double mean_time_lowBound_;
    double mean_time_uppBound_;
    double smoothEps;// for smoothL1
    PenaltyWeights penaltyWt;
    Eigen::Vector2d energyWeights;
    lbfgs::lbfgs_parameter_t lbfgs_params_;
    
    double finalMinSafeDis;
    int finalSafeDisCheckNum;
    int safeReplanMaxTime;

    Eigen::Vector3d iniStateXYTheta;
    Eigen::Vector3d finStateXYTheta;

    Eigen::Vector3d final_initStateXYTheta_;
    Eigen::Vector3d final_finStateXYTheta_;

    Eigen::VectorXd pieceTime;
    Eigen::MatrixXd Innerpoints;
    Eigen::MatrixXd iniState;
    Eigen::MatrixXd finState;
    // trajectory segments number
    int TrajNum;
    // if the traj is cutted
    bool ifCutTraj_;

    std::vector<Eigen::Vector3d> inner_init_positions;

    Eigen::MatrixXd finalInnerpoints;
    Eigen::VectorXd finalpieceTime;

    minco::MINCO_S3NU Minco;
    std::vector<Eigen::Vector3d> statelist;

    PathLbfgsParams path_lbfgs_params_;
    PathpenaltyWeights PathpenaltyWt;
 

    // sampling parameters
    
    int sparseResolution_6_;
    double timeResolution_;
    int mintrajNum_;

    int iter_num_;
    // store the gradient of the cost function
    Eigen::Matrix2Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX2d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;
    Eigen::Vector2d gradByTailStateS;
    Eigen::Vector2d FinalIntegralXYError;
    // for ALM
    Eigen::Vector2d FinalIntegralXYError_;
    // for debug, record the collision points
    std::vector<Eigen::Vector2d> collision_point;

    // unchanged auxiliary parameters in the loop
    int SamNumEachPart;
    // Simpson integration coefficients for each sampling point
    Eigen::VectorXd IntegralChainCoeff;

    // checkpoints for collision check
    std::vector<Eigen::Vector2d> check_point;
    double safeDis_, safeDis;

    // Whether to perform visualization
    bool ifprint = false;

    // Augmented Lagrangian
    Eigen::VectorXd init_EqualLambda_, init_EqualRho_, EqualRhoMax_, EqualGamma_;
    Eigen::VectorXd EqualLambda, EqualRho;
    Eigen::VectorXd EqualTolerance_;

    Eigen::VectorXd Cut_init_EqualLambda_, Cut_init_EqualRho_, Cut_EqualRhoMax_, Cut_EqualGamma_;
    Eigen::VectorXd Cut_EqualLambda, Cut_EqualRho;
    Eigen::VectorXd Cut_EqualTolerance_;

    bool hrz_limited_;
    double hrz_laser_range_dgr_;

    // Trajectory prediction resolution for get_the_predicted_state
    double trajPredictResolution_;

    bool if_visual_optimization_ = false;

private:
    int sparseResolution_;
    std::vector<Eigen::Vector2d> collision_point_;

public:

    // Results
    Trajectory<5, 2> final_traj_;
    // Results before collision check
    Trajectory<5, 2> optimizer_traj_;
    // Results before trajectory pre-processing
    Trajectory<5, 2> init_final_traj_;

    Eigen::Vector3d ICR_;
    bool if_standard_diff_;

    MSPlanner(const Config &conf, std::shared_ptr<planner::map::Map> map,
                MSPConfig msp_config,PenaltyWeights penaltyWt,
                PathpenaltyWeights PathpenaltyWt ,
                PathLbfgsParams path_lbfgs_params,const rclcpp::Node::SharedPtr &node);

    // Main function of the optimizer
    bool minco_plan(const FlatTrajData &flat_traj);
    // Obtain the initial state for planning
    bool get_state(const FlatTrajData &flat_traj);
    // Optimization
    bool optimizer();
    // Result check: whether a collision occurred
    bool check_final_collision(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta);

    template <typename EIGENVEC>
    inline void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    inline void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    static inline int earlyExit(void *instance,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                const double fx,
                                const double step,
                                const int k,
                                const int ls);

    static double costFunctionCallback(void *ptr,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g);

    // Gradient for partialGradByCoeffs and partialGradByTimes
    void attachPenaltyFunctional(double &cost);

    inline void positiveSmoothedL1(const double &x, double &f, double &df);

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                    const Eigen::VectorXd &gradT,
                                    EIGENVEC &gradTau);
    
    static double costFunctionCallbackPath(void *ptr,
                                           const Eigen::VectorXd &x,
                                           Eigen::VectorXd &g);

    void attachPenaltyFunctionalPath(double &cost);
    void mincoPathPub(const Trajectory<5, 2> &final_traj, 
                        const Eigen::Vector3d &start_state_XYTheta, 
                        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher);
    void mincoPointPub(const Trajectory<5,2> &final_traj, 
                        const Eigen::Vector3d &start_state_XYTheta, 
                        const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher, 
                        const Eigen::Vector3d &color);

    void Collision_point_Pub();

    Eigen::MatrixXd get_current_iniState(){
        return iniState;
    }
    Eigen::MatrixXd get_current_finState(){
        return finState;
    }
    Eigen::MatrixXd get_current_Innerpoints(){
        return finalInnerpoints;
    }
    Eigen::VectorXd get_current_finalpieceTime(){
        return finalpieceTime;
    }

    Eigen::Vector3d get_current_iniStateXYTheta(){
        return iniStateXYTheta;
    }

    void get_the_predicted_state(const double& time, Eigen::Vector3d& XYTheta, Eigen::Vector3d& VAJ, Eigen::Vector3d& OAJ);

    std::vector<Eigen::Vector3d> get_the_predicted_state_and_path(const double &start_time, const double &time, 
                                                                  const Eigen::Vector3d &start_XYTheta, 
                                                                  Eigen::Vector3d &XYTheta, bool &if_forward);

    inline double normlize_angle(double angle);
    
    void pub_inner_init_positions(const std::vector<Eigen::Vector3d> &inner_init_positions);
    // template<typename T>
    // bool readParam(const std::string &path, T &param){
    //     if(!nh_.hasParam(path)){
    //         ROS_ERROR("ERROR!! cannot read param %s", path.c_str());
    //         return false;
    //     }
    //     nh_.getParam(path, param);
    //     return true;
    // }

    // template<typename T>
    // bool readParam(const std::string &path, T &param, const T &Default){
    //     if(!nh_.hasParam(path)){
    //         param = Default;
    //         ROS_ERROR("ERROR!! cannot read param %s", path.c_str());
    //         return false;
    //     }
    //     nh_.getParam(path, param);
    //     return true;
    // }

    
};



#endif