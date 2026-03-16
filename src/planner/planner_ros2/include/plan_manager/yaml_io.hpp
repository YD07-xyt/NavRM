#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <memory>

#include "planner_map.hpp"
#include "jps_planner/jps_planner.h"
#include "optimizer.h"

// ROS2参数读取类
class ConfigReader {
public:
     ConfigReader(rclcpp::Node::SharedPtr node) : node_(node) {
        declareParameters();
        loadParameters();
    }

private:
    void declareParameters() {
        // Config parameters
        node_->declare_parameter<double>("max_vel", 5.0);
        node_->declare_parameter<double>("min_vel", -5.0);
        node_->declare_parameter<double>("max_acc", 5.0);
        node_->declare_parameter<double>("max_omega", 1.0);
        node_->declare_parameter<double>("max_domega", 50.0);
        node_->declare_parameter<double>("max_centripetal_acc", 10000.0);
        node_->declare_parameter<bool>("if_directly_constrain_v_omega", false);

        // MSPConfig parameters
        node_->declare_parameter<double>("mean_time_lowBound", 0.0);
        node_->declare_parameter<double>("mean_time_uppBound", 10.0);
        node_->declare_parameter<double>("smoothEps", 0.1);
        node_->declare_parameter<double>("safeDis_", 0.5);
        node_->declare_parameter<double>("finalMinSafeDis", 0.3);
        node_->declare_parameter<int>("finalSafeDisCheckNum", 5);
        node_->declare_parameter<int>("safeReplanMaxTime", 10);

        // Vector parameters
        node_->declare_parameter<std::vector<double>>("_energyWeights", std::vector<double>{1.0, 1.0, 1.0});
        node_->declare_parameter<std::vector<double>>("_EqualLambda", std::vector<double>{1.0, 1.0, 1.0});
               node_->declare_parameter<std::vector<double>>("_EqualRho", 
            std::vector<double>{1000.0, 1000.0});
        node_->declare_parameter<std::vector<double>>("_EqualRhoMax", 
            std::vector<double>{1.0e+10, 1.0e+10});
        node_->declare_parameter<std::vector<double>>("_EqualGamma", 
            std::vector<double>{90.0, 90.0});
        node_->declare_parameter<std::vector<double>>("_EqualTolerance", 
            std::vector<double>{0.01, 0.0});
        node_->declare_parameter<std::vector<double>>("_CutEqualLambda", 
            std::vector<double>{0.0, 0.0});
        node_->declare_parameter<std::vector<double>>("_CutEqualRhoMax", 
            std::vector<double>{1.0e+10, 1.0e+10});
        node_->declare_parameter<std::vector<double>>("_CutEqualRho", 
            std::vector<double>{1000.0, 1000.0});
        node_->declare_parameter<std::vector<double>>("_CutEqualGamma", 
            std::vector<double>{5.0, 5.0});
        node_->declare_parameter<std::vector<double>>("_CutEqualTolerance", 
            std::vector<double>{0.5, 0.0});

        // Sampling parameters
        node_->declare_parameter<int>("sparseResolution", 10);
        node_->declare_parameter<double>("timeResolution", 0.1);
        node_->declare_parameter<int>("mintrajNum", 5);
        node_->declare_parameter<double>("trajPredictResolution", 0.05);
        node_->declare_parameter<bool>("if_visual_optimization", false);
        node_->declare_parameter<bool>("hrz_limited", false);
        node_->declare_parameter<double>("hrz_laser_range_dgr", 360.0);

        // ICR parameters
        node_->declare_parameter<double>("ICR_yl", 0.0);
        node_->declare_parameter<double>("ICR_xv", 0.0);
        node_->declare_parameter<double>("ICR_yr", 0.0);
        node_->declare_parameter<bool>("if_standard_diff", true);

        // PenaltyWeights (nested parameters)
        node_->declare_parameter<double>("penalty_weights.time_weight", 1.0);
        node_->declare_parameter<double>("penalty_weights.time_weight_backup_for_replan", 1.0);
        node_->declare_parameter<double>("penalty_weights.acc_weight", 1.0);
        node_->declare_parameter<double>("penalty_weights.domega_weight", 1.0);
        node_->declare_parameter<double>("penalty_weights.collision_weight", 10.0);
        node_->declare_parameter<double>("penalty_weights.moment_weight", 1.0);
        node_->declare_parameter<double>("penalty_weights.mean_time_weight", 1.0);
        node_->declare_parameter<double>("penalty_weights.cen_acc_weight", 1.0);

        // PathpenaltyWeights
        node_->declare_parameter<double>("path_penalty_weights.time_weight", 1.0);
        node_->declare_parameter<double>("path_penalty_weights.bigpath_sdf_weight", 1.0);
        node_->declare_parameter<double>("path_penalty_weights.mean_time_weight", 1.0);
        node_->declare_parameter<double>("path_penalty_weights.moment_weight", 1.0);
        node_->declare_parameter<double>("path_penalty_weights.acc_weight", 1.0);
        node_->declare_parameter<double>("path_penalty_weights.domega_weight", 1.0);

        // PathLbfgsParams
        node_->declare_parameter<double>("path_lbfgs_params.normal_past", 5.0);
        node_->declare_parameter<double>("path_lbfgs_params.shot_path_past", 3.0);
        node_->declare_parameter<double>("path_lbfgs_params.shot_path_horizon", 10.0);

        // JPSPlannerParams
        node_->declare_parameter<double>("jps_planner_params.safe_dis_", 0.5);
        node_->declare_parameter<double>("jps_planner_params.max_jps_dis_", 5.0);
        node_->declare_parameter<double>("jps_planner_params.distance_weight_", 1.0);
        node_->declare_parameter<double>("jps_planner_params.yaw_weight_", 0.5);
        node_->declare_parameter<double>("jps_planner_params.trajCutLength_", 2.0);
        node_->declare_parameter<double>("jps_planner_params.max_vel_", 5.0);
        node_->declare_parameter<double>("jps_planner_params.max_acc_", 5.0);
        node_->declare_parameter<double>("jps_planner_params.max_omega_", 1.0);
        node_->declare_parameter<double>("jps_planner_params.max_domega_", 50.0);
        node_->declare_parameter<double>("jps_planner_params.sampletime_", 0.1);
        node_->declare_parameter<int>("jps_planner_params.mintrajNum_", 5);

        // OccupancyGridMapConfig
        node_->declare_parameter<double>("occupancy_grid_map.resolution", 0.05);
        node_->declare_parameter<double>("occupancy_grid_map.detection_range", 5.0);
        node_->declare_parameter<double>("occupancy_grid_map.global_x_lower", -10.0);
        node_->declare_parameter<double>("occupancy_grid_map.global_x_upper", 10.0);
        node_->declare_parameter<double>("occupancy_grid_map.global_y_lower", -10.0);
        node_->declare_parameter<double>("occupancy_grid_map.global_y_upper", 10.0);
        node_->declare_parameter<bool>("occupancy_grid_map.if_perspective", false);
        node_->declare_parameter<bool>("occupancy_grid_map.if_cirSupRaycast", true);
        node_->declare_parameter<bool>("occupancy_grid_map.hrz_limited", false);
        node_->declare_parameter<double>("occupancy_grid_map.hrz_laser_range_dgr", 360.0);
        node_->declare_parameter<double>("occupancy_grid_map.p_hit", 0.7);
        node_->declare_parameter<double>("occupancy_grid_map.p_miss", 0.3);
        node_->declare_parameter<double>("occupancy_grid_map.p_min", 0.1);
        node_->declare_parameter<double>("occupancy_grid_map.p_max", 0.9);
        node_->declare_parameter<double>("occupancy_grid_map.p_occ", 0.5);
    }

    void loadParameters() {
        // Load Config
        config_.max_vel_ = node_->get_parameter("max_vel").as_double();
        config_.min_vel_ = node_->get_parameter("min_vel").as_double();
        config_.max_acc_ = node_->get_parameter("max_acc").as_double();
        config_.max_omega_ = node_->get_parameter("max_omega").as_double();
        config_.max_domega_ = node_->get_parameter("max_domega").as_double();
        config_.max_centripetal_acc_ = node_->get_parameter("max_centripetal_acc").as_double();
        config_.if_directly_constrain_v_omega_ = node_->get_parameter("if_directly_constrain_v_omega").as_bool();

        // Load MSPConfig
        msp_config_.mean_time_lowBound_ = node_->get_parameter("mean_time_lowBound").as_double();
        msp_config_.mean_time_uppBound_ = node_->get_parameter("mean_time_uppBound").as_double();
        msp_config_.smoothEps = node_->get_parameter("smoothEps").as_double();
        msp_config_.safeDis_ = node_->get_parameter("safeDis_").as_double();
        msp_config_.finalMinSafeDis = node_->get_parameter("finalMinSafeDis").as_double();
        msp_config_.finalSafeDisCheckNum = node_->get_parameter("finalSafeDisCheckNum").as_int();
        msp_config_.safeReplanMaxTime = node_->get_parameter("safeReplanMaxTime").as_int();

        // Load vectors
        msp_config_._energyWeights = node_->get_parameter("_energyWeights").as_double_array();
        msp_config_._EqualLambda = node_->get_parameter("_EqualLambda").as_double_array();
        msp_config_._EqualRho = node_->get_parameter("_EqualRho").as_double_array();
        msp_config_._EqualRhoMax= node_->get_parameter("_EqualRhoMax").as_double_array();
        msp_config_._EqualGamma = node_->get_parameter("_EqualGamma").as_double_array();
        msp_config_._EqualTolerance = node_->get_parameter("_EqualTolerance").as_double_array();
        msp_config_._CutEqualLambda = node_->get_parameter("_CutEqualLambda").as_double_array();
        msp_config_._CutEqualRhoMax = node_->get_parameter("_CutEqualRhoMax").as_double_array();
        msp_config_._CutEqualRho = node_->get_parameter("_CutEqualRho").as_double_array();
        msp_config_._CutEqualGamma = node_->get_parameter("_CutEqualGamma").as_double_array();
        msp_config_._CutEqualTolerance = node_->get_parameter("_CutEqualTolerance").as_double_array();

        // Load sampling parameters
        msp_config_.sparseResolution = node_->get_parameter("sparseResolution").as_int();
        msp_config_.timeResolution = node_->get_parameter("timeResolution").as_double();
        msp_config_.mintrajNum = node_->get_parameter("mintrajNum").as_int();
        msp_config_.trajPredictResolution = node_->get_parameter("trajPredictResolution").as_double();
        msp_config_.if_visual_optimization = node_->get_parameter("if_visual_optimization").as_bool();
        msp_config_.hrz_limited_ = node_->get_parameter("hrz_limited").as_bool();
        msp_config_.hrz_laser_range_dgr = node_->get_parameter("hrz_laser_range_dgr").as_double();

        // Load ICR parameters
        msp_config_.ICR_yl = node_->get_parameter("ICR_yl").as_double();
        msp_config_.ICR_xv = node_->get_parameter("ICR_xv").as_double();
        msp_config_.ICR_yr = node_->get_parameter("ICR_yr").as_double();
        msp_config_.if_standard_diff = node_->get_parameter("if_standard_diff").as_bool();

        // Load PenaltyWeights
        penalty_weights_.time_weight = node_->get_parameter("penalty_weights.time_weight").as_double();
        penalty_weights_.time_weight_backup_for_replan = node_->get_parameter("penalty_weights.time_weight_backup_for_replan").as_double();
        penalty_weights_.acc_weight = node_->get_parameter("penalty_weights.acc_weight").as_double();
        penalty_weights_.domega_weight = node_->get_parameter("penalty_weights.domega_weight").as_double();
        penalty_weights_.collision_weight = node_->get_parameter("penalty_weights.collision_weight").as_double();
        penalty_weights_.moment_weight = node_->get_parameter("penalty_weights.moment_weight").as_double();
        penalty_weights_.mean_time_weight = node_->get_parameter("penalty_weights.mean_time_weight").as_double();
        penalty_weights_.cen_acc_weight = node_->get_parameter("penalty_weights.cen_acc_weight").as_double();

        // Load PathpenaltyWeights
        path_penalty_weights_.time_weight = node_->get_parameter("path_penalty_weights.time_weight").as_double();
        path_penalty_weights_.bigpath_sdf_weight = node_->get_parameter("path_penalty_weights.bigpath_sdf_weight").as_double();
        path_penalty_weights_.mean_time_weight = node_->get_parameter("path_penalty_weights.mean_time_weight").as_double();
        path_penalty_weights_.moment_weight = node_->get_parameter("path_penalty_weights.moment_weight").as_double();
        path_penalty_weights_.acc_weight = node_->get_parameter("path_penalty_weights.acc_weight").as_double();
        path_penalty_weights_.domega_weight = node_->get_parameter("path_penalty_weights.domega_weight").as_double();

        // Load PathLbfgsParams
        path_lbfgs_params_.normal_past = node_->get_parameter("path_lbfgs_params.normal_past").as_double();
        path_lbfgs_params_.shot_path_past = node_->get_parameter("path_lbfgs_params.shot_path_past").as_double();
        path_lbfgs_params_.shot_path_horizon = node_->get_parameter("path_lbfgs_params.shot_path_horizon").as_double();

        // Load JPSPlannerParams
        jps_params_.safe_dis_ = node_->get_parameter("jps_planner_params.safe_dis_").as_double();
        jps_params_.max_jps_dis_ = node_->get_parameter("jps_planner_params.max_jps_dis_").as_double();
        jps_params_.distance_weight_ = node_->get_parameter("jps_planner_params.distance_weight_").as_double();
        jps_params_.yaw_weight_ = node_->get_parameter("jps_planner_params.yaw_weight_").as_double();
        jps_params_.trajCutLength_ = node_->get_parameter("jps_planner_params.trajCutLength_").as_double();
        jps_params_.max_vel_ = node_->get_parameter("jps_planner_params.max_vel_").as_double();
        jps_params_.max_acc_ = node_->get_parameter("jps_planner_params.max_acc_").as_double();
        jps_params_.max_omega_ = node_->get_parameter("jps_planner_params.max_omega_").as_double();
        jps_params_.max_domega_ = node_->get_parameter("jps_planner_params.max_domega_").as_double();
        jps_params_.sampletime_ = node_->get_parameter("jps_planner_params.sampletime_").as_double();
        jps_params_.mintrajNum_ = node_->get_parameter("jps_planner_params.mintrajNum_").as_int();

        // Load OccupancyGridMapConfig
        occupancy_config_.resolution = node_->get_parameter("occupancy_grid_map.resolution").as_double();
        occupancy_config_.detection_range = node_->get_parameter("occupancy_grid_map.detection_range").as_double();
        occupancy_config_.global_x_lower = node_->get_parameter("occupancy_grid_map.global_x_lower").as_double();
        occupancy_config_.global_x_upper = node_->get_parameter("occupancy_grid_map.global_x_upper").as_double();
        occupancy_config_.global_y_lower = node_->get_parameter("occupancy_grid_map.global_y_lower").as_double();
        occupancy_config_.global_y_upper = node_->get_parameter("occupancy_grid_map.global_y_upper").as_double();
        occupancy_config_.if_perspective = node_->get_parameter("occupancy_grid_map.if_perspective").as_bool();
        occupancy_config_.if_cirSupRaycast = node_->get_parameter("occupancy_grid_map.if_cirSupRaycast").as_bool();
        occupancy_config_.hrz_limited = node_->get_parameter("occupancy_grid_map.hrz_limited").as_bool();
        occupancy_config_.hrz_laser_range_dgr = node_->get_parameter("occupancy_grid_map.hrz_laser_range_dgr").as_double();
        occupancy_config_.p_hit = node_->get_parameter("occupancy_grid_map.p_hit").as_double();
        occupancy_config_.p_miss = node_->get_parameter("occupancy_grid_map.p_miss").as_double();
        occupancy_config_.p_min = node_->get_parameter("occupancy_grid_map.p_min").as_double();
        occupancy_config_.p_max = node_->get_parameter("occupancy_grid_map.p_max").as_double();
        occupancy_config_.p_occ = node_->get_parameter("occupancy_grid_map.p_occ").as_double();
    }

public:
    // Getters for all configs
    const Config& getConfig() const { return config_; }
    const MSPConfig& getMSPConfig() const { return msp_config_; }
    const PenaltyWeights& getPenaltyWeights() const { return penalty_weights_; }
    const PathpenaltyWeights& getPathPenaltyWeights() const { return path_penalty_weights_; }
    const PathLbfgsParams& getPathLbfgsParams() const { return path_lbfgs_params_; }
    const JPS::JPSPlannerParams& getJPSPlannerParams() const { return jps_params_; }
    const planner::OccupancyGridMapConfig& getOccupancyGridMapConfig() const { return occupancy_config_; }

private:
    rclcpp::Node::SharedPtr node_;
    
    // Configuration structs
    Config config_;
    MSPConfig msp_config_;
    PenaltyWeights penalty_weights_;
    PathpenaltyWeights path_penalty_weights_;
    PathLbfgsParams path_lbfgs_params_;
    JPS::JPSPlannerParams jps_params_;
    planner::OccupancyGridMapConfig occupancy_config_;
};