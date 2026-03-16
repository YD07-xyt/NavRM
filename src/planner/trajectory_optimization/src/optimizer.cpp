#include "../include/optimizer.h"
#include <thread>
#include <chrono>
#include<Eigen/Core>



MSPlanner::MSPlanner(const Config &conf, std::shared_ptr<planner::map::Map> map,
    MSPConfig msp_config,PenaltyWeights penaltyWt,PathpenaltyWeights PathpenaltyWt ,
    PathLbfgsParams path_lbfgs_params,const rclcpp::Node::SharedPtr &node):config_(conf),
    penaltyWt(penaltyWt),
    PathpenaltyWt(PathpenaltyWt),path_lbfgs_params_(path_lbfgs_params){
    msp_planner_node_ = node;
    map_ = map;
    mincoinitPath = msp_planner_node_->create_publisher<nav_msgs::msg::Path>("/visualizer/mincoinitPath", 10);
    pathmincoinitPath = msp_planner_node_->create_publisher<nav_msgs::msg::Path>("/visualizer/pathmincoinitPath", 10);
    CollisionpointPub = msp_planner_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/visualizer/CollisionpointPub", 10);
    processmincoinitPath = msp_planner_node_->create_publisher<nav_msgs::msg::Path>("/visualizer/mincoPath", 10);

    mincoinitPoint = msp_planner_node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/mincoinitPoint", 10);
    pathmincoinitPoint = msp_planner_node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/pathmincoinitPoint", 10);
    innerinitpositionsPoint = msp_planner_node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/innerinitpositionsPoint",10);

    recordTextPub = msp_planner_node_->create_publisher<visualization_msgs::msg::Marker>("/planner/calculator_time",10);


    mean_time_lowBound_=msp_config.mean_time_lowBound_;
    mean_time_uppBound_=msp_config.mean_time_uppBound_;
    smoothEps=msp_config.smoothEps;
    safeDis_=msp_config.safeDis_;

    safeDis = safeDis_;

    finalMinSafeDis=msp_config.finalMinSafeDis;
    finalSafeDisCheckNum=msp_config.finalSafeDisCheckNum;
    safeReplanMaxTime=msp_config.safeReplanMaxTime;

    penaltyWt.time_weight = penaltyWt.time_weight_backup_for_replan;

    std::vector<double>  tmp_vec;
    tmp_vec=msp_config._energyWeights;
    for(u_int i=0; i < std::min(2, (int)tmp_vec.size()); i++){
        this->energyWeights[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._EqualLambda;
    init_EqualLambda_.resize(tmp_vec.size());

    for(u_int i=0; i<tmp_vec.size(); i++){
        init_EqualLambda_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._EqualRho;
    init_EqualRho_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        init_EqualRho_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._EqualRhoMax;
    EqualRhoMax_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        EqualRhoMax_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._EqualGamma;
    EqualGamma_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        EqualGamma_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._EqualTolerance;
    EqualTolerance_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        EqualTolerance_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._CutEqualLambda;
    Cut_init_EqualLambda_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_init_EqualLambda_[i] = tmp_vec[i];
    }
    
    tmp_vec=msp_config._CutEqualRho;
    Cut_init_EqualRho_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_init_EqualRho_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._CutEqualRhoMax;
    Cut_EqualRhoMax_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_EqualRhoMax_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._CutEqualGamma;
    Cut_EqualGamma_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_EqualGamma_[i] = tmp_vec[i];
    }

    tmp_vec=msp_config._CutEqualTolerance;
    Cut_EqualTolerance_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_EqualTolerance_[i] = tmp_vec[i];
    }

    sparseResolution_=msp_config.sparseResolution;
    timeResolution_=msp_config.timeResolution;
    mintrajNum_=msp_config.mintrajNum;

    trajPredictResolution_=msp_config.trajPredictResolution;
    if_visual_optimization_=msp_config.if_visual_optimization;

    hrz_limited_ = msp_config.hrz_limited_;

    if(hrz_limited_)  {
        hrz_laser_range_dgr_=msp_config.hrz_laser_range_dgr;

    }
    // Initialize some auxiliary variables
    SamNumEachPart = 2 * sparseResolution_;
    sparseResolution_6_ = sparseResolution_ * 6;

    IntegralChainCoeff.resize(SamNumEachPart + 1);
    IntegralChainCoeff.setZero();

    for(int i=0; i<sparseResolution_; i++){
        IntegralChainCoeff.block(2*i,0,3,1) += Eigen::Vector3d(1.0, 4.0, 1.0);
    }


    //TODO:
    check_point.clear();

    tmp_vec=msp_config.checkpoint;
    for(u_int i=0; i<tmp_vec.size()/2; i++){
        check_point.emplace_back(tmp_vec[i*2], tmp_vec[i*2+1]);
    }

    ICR_.x()=msp_config.ICR_yl;
    ICR_.y()=msp_config.ICR_yr;
    ICR_.z()=msp_config.ICR_xv;


    if_standard_diff_=msp_config.if_standard_diff;
}

bool MSPlanner::minco_plan(const FlatTrajData &flat_traj){
    //TODO:time
     rclcpp::Time minco_start = msp_planner_node_->now();
     rclcpp::Time current = msp_planner_node_->now();
    bool final_collision = false;
    int replan_num_for_coll = 0;

    double start_safe_dis = map_->occupancy_grid_map.getDistanceReal(flat_traj.start_state_XYTheta.head(2))*0.85;
    safeDis = std::min(start_safe_dis, safeDis_);
    
    for(; replan_num_for_coll < safeReplanMaxTime; replan_num_for_coll++){

        if(get_state(flat_traj)){
            RCLCPP_INFO(msp_planner_node_->get_logger(), "\033[40;36m get_state time:%f \033[0m", (msp_planner_node_->now()-current).seconds());
        }else{
            return false;
        }
        current = msp_planner_node_->now();
        if(optimizer()){
            RCLCPP_INFO(msp_planner_node_->get_logger(), "\033[41;37m minco optimizer time:%f \033[0m", (msp_planner_node_->now()-current).seconds());
        }else{
            return false;
        }
        Minco.getTrajectory(optimizer_traj_);
        final_collision = check_final_collision(optimizer_traj_, iniStateXYTheta);
        if(final_collision){
            penaltyWt.time_weight *= 0.75;
            // safeDis *= 1.2;
        }
        else{
            break;
        }
    }
    Collision_point_Pub();
    penaltyWt.time_weight = penaltyWt.time_weight_backup_for_replan;
    safeDis = safeDis_;
    if(replan_num_for_coll == safeReplanMaxTime){
        RCLCPP_ERROR(msp_planner_node_->get_logger(), "\n\n\n final traj Collision!!!!!!!!!!!\n\n\n\n");
        return false;
    }

    final_traj_ = optimizer_traj_;
    final_initStateXYTheta_ = iniStateXYTheta;
    final_finStateXYTheta_ = finStateXYTheta;
    Collision_point_Pub();

    RCLCPP_INFO(msp_planner_node_->get_logger(), "-------------------------------------------------------------------------------------------------------------------------------");
    
    RCLCPP_INFO(msp_planner_node_->get_logger(), "\033[41;37m all of back_end time:%f, with optimizer %d times. \033[0m", (msp_planner_node_->now()-minco_start).seconds(), replan_num_for_coll+1);

    return true;
}

bool MSPlanner::get_state(const FlatTrajData &flat_traj){
    ifCutTraj_ = flat_traj.if_cut;

    TrajNum = flat_traj.UnOccupied_traj_pts.size()+1;

    Innerpoints.resize(2,TrajNum-1);
    for(u_int i=0; i<flat_traj.UnOccupied_traj_pts.size(); i++){
        Innerpoints.col(i) = flat_traj.UnOccupied_traj_pts[i].head(2);
    }

    inner_init_positions = flat_traj.UnOccupied_positions;
    inner_init_positions.push_back(flat_traj.final_state_XYTheta);

    iniState = flat_traj.start_state;
    finState = flat_traj.final_state;

    pieceTime.resize(TrajNum); pieceTime.setOnes();
    pieceTime *= flat_traj.UnOccupied_initT;

    iniStateXYTheta = flat_traj.start_state_XYTheta;
    finStateXYTheta = flat_traj.final_state_XYTheta;

    pub_inner_init_positions(inner_init_positions);
    
    return true;
}

bool MSPlanner::optimizer(){
    // Initialize Lagrangian
    if(!ifCutTraj_){
        EqualLambda = init_EqualLambda_;
        EqualRho = init_EqualRho_;
    }
    else{
        EqualLambda = Cut_init_EqualLambda_;
        EqualRho = Cut_init_EqualRho_;
    }


    // 2*(N-1) intermediate points, 1 relaxed S, N times
    int variable_num_ = 3 * TrajNum - 1;
    //RCLCPP_INFO(msp_planner_node_->get_logger(), "iniStates: \n" , iniState);
    //RCLCPP_INFO(msp_planner_node_->get_logger(), "finStates: \n" , finState);
    RCLCPP_INFO(msp_planner_node_->get_logger(), "TrajNum: %d", TrajNum);
    Minco.setConditions(iniState, finState, TrajNum, energyWeights);

    //RCLCPP_INFO(msp_planner_node_->get_logger(), "init Innerpoints: \n" << Innerpoints);
    //RCLCPP_INFO(msp_planner_node_->get_logger(), "init pieceTime: " << pieceTime.transpose());

    Minco.setParameters(Innerpoints, pieceTime);   
    Minco.getTrajectory(init_final_traj_);
    mincoPathPub(init_final_traj_, iniStateXYTheta, mincoinitPath); 
    mincoPointPub(init_final_traj_, iniStateXYTheta, mincoinitPoint, Eigen::Vector3d(173, 127, 168));
    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    memcpy(x.data()+offset,Innerpoints.data(), Innerpoints.size() * sizeof(x[0]));
    offset += Innerpoints.size();
    x[offset] = finState(1,0);
    ++offset;
    Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, pieceTime.size());
    offset += pieceTime.size();
    RealT2VirtualT(pieceTime,Vt);

    double cost;
    int result;
    Eigen::VectorXd g;
    g.resize(x.size());
    iter_num_ = 0;

    auto start = std::chrono::high_resolution_clock::now();
    // Handle cases where the path is too short to converge
    if (fabs(finState(1, 0)) < path_lbfgs_params_.shot_path_horizon) {
        path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.shot_path_past;
    } else {
        path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.normal_past;
    }

    ifprint = false;
    result = lbfgs::lbfgs_optimize(x,
                                cost,
                                MSPlanner::costFunctionCallbackPath,
                                NULL,
                                NULL,
                                this,
                                path_lbfgs_params_.path_lbfgs_params);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    //Output computation time
    visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = msp_planner_node_->now();
        marker.ns = "pre_process";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 11.5;
        marker.pose.position.y = 7;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        double search_time = duration / 1000.0;
        std::ostringstream out;
        out << std::fixed << "Pre-process: \n"<< std::setprecision(2) << search_time<<" ms";
        marker.text = out.str();
        recordTextPub->publish(marker);

    ifprint = true;
    costFunctionCallbackPath(this,x,g);
    ifprint = false;

    //RCLCPP_INFO(msp_planner_node_->get_logger(), "Pre-processing optimizer:" << duration / 1000.0 << " ms");
    RCLCPP_INFO(msp_planner_node_->get_logger(), "Pre-processing finish! result:%d   finalcost:%f   iter_num_:%d", result, cost, iter_num_);
    offset = 0;
    Eigen::Map<Eigen::MatrixXd> PathP(x.data() + offset, 2, TrajNum - 1);
    offset += 2 * (TrajNum - 1);
    finalInnerpoints = PathP;
    finState(1, 0) = x[offset];
    ++offset;

    Eigen::Map<const Eigen::VectorXd> Patht(x.data() + offset, TrajNum);
    offset += TrajNum;
    VirtualT2RealT(Patht, finalpieceTime);
    Minco.setTConditions(finState);
    Minco.setParameters(finalInnerpoints, finalpieceTime);
    Minco.getTrajectory(final_traj_);
    mincoPathPub(final_traj_, iniStateXYTheta, pathmincoinitPath);
    mincoPointPub(final_traj_, iniStateXYTheta, pathmincoinitPoint, Eigen::Vector3d(114, 159, 207));

    // RCLCPP_INFO(msp_planner_node_->get_logger(), "Path final pieces time: " << finalpieceTime.transpose());
    // RCLCPP_INFO(msp_planner_node_->get_logger(), "Path final Innerpoints: \n" << finalInnerpoints);
    // RCLCPP_INFO(msp_planner_node_->get_logger(), "Path final finState: \n" << finState);

    // Formal optimization
    RCLCPP_INFO(msp_planner_node_->get_logger(), "---------------------------------------------------------------------init---------------------------------------------------------------");
    ifprint = true;
    costFunctionCallback(this, x, g);
    ifprint = false;
    RCLCPP_INFO(msp_planner_node_->get_logger(), "-------------------------------------------------------------------optimize---------------------------------------------------------------");
    iter_num_ = 0;
    
    rclcpp::Time current = msp_planner_node_->now();
    
    //TODO while ros::ok()

    while(1){
        result = lbfgs::lbfgs_optimize(x,
                                       cost,
                                       MSPlanner::costFunctionCallback,
                                       NULL,
                                       MSPlanner::earlyExit,
                                       this,
                                       lbfgs_params_);
        if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
            result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION){
            RCLCPP_INFO(msp_planner_node_->get_logger(), "optimizer finish! result:%d   finalcost:%f   iter_num_:%d ",result,cost,iter_num_);
        } 
        else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
            RCLCPP_WARN(msp_planner_node_->get_logger(), "Lbfgs: The line-search routine reaches the maximum number of evaluations.");
        }
        else{
            RCLCPP_WARN(msp_planner_node_->get_logger(), "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        }
        // ALM update
        if(!ifCutTraj_){

            //RCLCPP_INFO(msp_planner_node_->get_logger(), "EqualLambda: " << EqualLambda.transpose() << "  EqualRho: " << EqualRho.transpose() << "  current hx cost:" << FinalIntegralXYError.transpose() << "  XYError.norm():" << FinalIntegralXYError.norm());
            if(FinalIntegralXYError.norm() < EqualTolerance_[0]){
                break;
            }
            EqualLambda[0] += EqualRho[0] * FinalIntegralXYError.x();
            EqualLambda[1] += EqualRho[1] * FinalIntegralXYError.y();
            EqualRho[0] = std::min((1 + EqualGamma_[0]) * EqualRho[0], EqualRhoMax_[0]);
            EqualRho[1] = std::min((1 + EqualGamma_[1]) * EqualRho[1], EqualRhoMax_[1]);
        }
        else{
            //RCLCPP_INFO(msp_planner_node_->get_logger(), "EqualLambda: " << EqualLambda.transpose() << "  EqualRho: " << EqualRho.transpose() << "  current hx cost:" << FinalIntegralXYError.transpose() << "  XYError.norm():" << FinalIntegralXYError.norm());
            if(FinalIntegralXYError.norm() < Cut_EqualTolerance_[0]){
                break;
            }
            EqualLambda[0] += EqualRho[0] * FinalIntegralXYError.x();
            EqualLambda[1] += EqualRho[1] * FinalIntegralXYError.y();
            EqualRho[0] = std::min((1 + Cut_EqualGamma_[0]) * EqualRho[0], Cut_EqualRhoMax_[0]);
            EqualRho[1] = std::min((1 + Cut_EqualGamma_[1]) * EqualRho[1], Cut_EqualRhoMax_[1]);

        }

    }

    double mincotime = (msp_planner_node_->now()-current).seconds();
    RCLCPP_INFO(msp_planner_node_->get_logger(), "\033[40;36m minco optimizer time:%f \033[0m", mincotime);
    RCLCPP_INFO(msp_planner_node_->get_logger(), "--------------------------------------------------------------------final------------------------------------------------------");

    marker.header.frame_id = "world";
    marker.header.stamp = msp_planner_node_->now();
    marker.ns = "minco";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 11.5;
    marker.pose.position.y = 6;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    search_time = mincotime*1000.0;
    std::ostringstream out2;
    out2 << std::fixed <<"Optimization: \n"<< std::setprecision(2) << search_time << " ms";
    marker.text = out2.str();
    recordTextPub->publish(marker);

    //Output optimization infomation
    ifprint = true;
    costFunctionCallback(this,x,g);
    
    offset = 0;
    Eigen::Map<Eigen::MatrixXd> P(x.data()+offset, 2, TrajNum - 1);
    offset += 2 * (TrajNum - 1);
    finalInnerpoints = P;

    finState(1,0) = x[offset];
    ++offset;

    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, TrajNum);
    offset += TrajNum;
    VirtualT2RealT(t,finalpieceTime);
    Minco.setTConditions(finState);
    Minco.setParameters(finalInnerpoints, finalpieceTime);

    // std::cout<<"finalInnerpoints: \n"<<finalInnerpoints<<std::endl;
    // std::cout<<"finalpieceTime: \n"<<finalpieceTime.transpose()<<std::endl;

    RCLCPP_INFO(msp_planner_node_->get_logger(), "\n\n optimizer finish! result:%d   finalcost:%f   iter_num_:%d\n\n ",result,cost,iter_num_);

    return true;
}

bool MSPlanner::check_final_collision(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta){
    double ini_x = start_state_XYTheta.x();
    double ini_y = start_state_XYTheta.y();

    double s1;
    int sparseResolution = finalSafeDisCheckNum;
    int SamNumEachPart = 2 * sparseResolution;
    double sumT = 0.0;

    int TrajNum = final_traj.getPieceNum();
    Eigen::VectorXd pieceTime = final_traj.getDurations();

    std::vector<Eigen::VectorXd> VecIntegralX(TrajNum);
    std::vector<Eigen::VectorXd> VecIntegralY(TrajNum);
    std::vector<Eigen::VectorXd> VecYaw(TrajNum);
    std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum+1);
    VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

    for(int i=0; i<TrajNum; i++){
        double step = pieceTime[i] / sparseResolution;
        double halfstep = step / 2.0;
        double CoeffIntegral = pieceTime[i] / sparseResolution / 6.0;

        Eigen::VectorXd IntegralX(sparseResolution);IntegralX.setZero();
        Eigen::VectorXd IntegralY(sparseResolution);IntegralY.setZero();
        Eigen::VectorXd Yaw(sparseResolution);Yaw.setZero();
        s1 = 0.0;
        for(int j=0; j<=SamNumEachPart; j++){
            if(if_standard_diff_){
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
            else{
                Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                Eigen::Vector2d dsigma = final_traj.getVel(s1+sumT);
                double cosyaw = cos(currPos.x());
                double sinyaw = sin(currPos.x());
                s1 += halfstep;
                if(j%2 == 0){
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2-1] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }
                }
                else{
                    IntegralX[j/2] += 4.0 * CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                    IntegralY[j/2] += 4.0 * CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                }
            }
        }
        VecIntegralX[i] = IntegralX;
        VecIntegralY[i] = IntegralY;
        VecYaw[i] = Yaw;
        // VecTrajFinalXY[i+1] = Eigen::Vector2d(IntegralX[IntegralX.size()-1], IntegralY[IntegralX.size()-1]);
        sumT += pieceTime[i];
    }
    double min_distance = DBL_MAX;
    Eigen::Vector2d pos(ini_x, ini_y);
    for(u_int i=0; i<VecIntegralX.size(); i++){
        for(u_int j=0; j<VecIntegralX[i].size(); j++){
            pos.x() += VecIntegralX[i][j];
            pos.y() += VecIntegralY[i][j];
            double SDFvalue = map_->occupancy_grid_map.getDistWithGradBilinear(pos);
            if(SDFvalue < min_distance)
                min_distance = SDFvalue;
            if(SDFvalue < finalMinSafeDis){
                //RCLCPP_INFO(msp_planner_node_->get_logger(), "SDFvalue < finalMinSafeDis!!!  min Distance: %f", min_distance);
                return true;
            }
                
        }
    }
    //RCLCPP_INFO(msp_planner_node_->get_logger(), "\033[39;35m min Distance: %f \033[0m", min_distance);
    return false;
}

template <typename EIGENVEC>
inline void MSPlanner::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT){
    const int sizeT = RT.size();
    VT.resize(sizeT);
    for (int i = 0; i < sizeT; ++i){
        VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                            : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
}

template <typename EIGENVEC>
inline void MSPlanner::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT){
    const int sizeTau = VT.size();
    RT.resize(sizeTau);
    for (int i = 0; i < sizeTau; ++i){
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
}

inline int MSPlanner::earlyExit(void *instance,
                            const Eigen::VectorXd &x,
                            const Eigen::VectorXd &g,
                            const double fx,
                            const double step,
                            const int k,
                            const int ls){
    MSPlanner &obj = *(MSPlanner *)instance;
    obj.FinalIntegralXYError_ = obj.FinalIntegralXYError;
    obj.collision_point_ = obj.collision_point;
    // std::cout<<"cost: "<<fx<<std::endl;

    if(obj.if_visual_optimization_){
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "fx: %f  step: %f  k: %d  ls: %d", fx, step, k, ls);
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "x: %s", x.transpose().toString().c_str());
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "g: %s", g.transpose().toString().c_str());
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "");
        int offset = 0;
        Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum - 1);
        offset += 2 * (obj.TrajNum - 1);
        obj.finalInnerpoints = P;

        obj.finState(1,0) = x[offset];
        ++offset;

        Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum);
        offset += obj.TrajNum;
        obj.VirtualT2RealT(t,obj.finalpieceTime);
        obj.Minco.setTConditions(obj.finState);
        obj.Minco.setParameters(obj.finalInnerpoints, obj.finalpieceTime);
        obj.Minco.getTrajectory(obj.optimizer_traj_);
        obj.mincoPathPub(obj.optimizer_traj_, obj.iniStateXYTheta, obj.processmincoinitPath);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}


double MSPlanner::costFunctionCallback(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g){

    if(x.norm()>1e4)
        return inf;

    MSPlanner &obj = *(MSPlanner *)ptr;
    obj.iter_num_ += 1;

    g.setZero();
    // Map the input variables to the variable matrix
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data()+offset, 2, obj.TrajNum - 1);
    offset += 2 * (obj.TrajNum - 1);

    double* gradTailS = g.data()+offset;
    obj.finState(1,0) = x[offset];
    ++offset;
    
    gradP.setZero();
    obj.Innerpoints = P;
    
    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum);
    Eigen::Map<Eigen::VectorXd> gradt(g.data()+offset, obj.TrajNum);

    offset += obj.TrajNum;
    obj.VirtualT2RealT(t, obj.pieceTime);
    gradt.setZero();

    double cost;
    obj.Minco.setTConditions(obj.finState);
    obj.Minco.setParameters(obj.Innerpoints,obj.pieceTime);
    obj.Minco.getEnergy(cost);
    obj.Minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.Minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
    if(obj.ifprint){
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "Energy cost: %f", cost);
    }
    obj.attachPenaltyFunctional(cost);
    if(obj.ifprint){
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "attachPenaltyFunctional cost: %f", cost);
    }
    obj.Minco.propogateArcYawLenghGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                        obj.gradByPoints, obj.gradByTimes, obj.gradByTailStateS);

    cost += obj.penaltyWt.time_weight * obj.pieceTime.sum();
    if(obj.ifprint){
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "T cost: %f", obj.penaltyWt.time_weight * obj.pieceTime.sum());
    }
    Eigen::VectorXd rhotimes;
    rhotimes.resize(obj.gradByTimes.size());
    obj.gradByTimes += obj.penaltyWt.time_weight * rhotimes.setOnes();
    
    *gradTailS = obj.gradByTailStateS.y();

    gradP = obj.gradByPoints;
    backwardGradT(t, obj.gradByTimes, gradt);
    
    return cost;
}

void MSPlanner::attachPenaltyFunctional(double &cost){
    collision_point.clear();
    double ini_x = iniStateXYTheta.x();
    double ini_y = iniStateXYTheta.y();

    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    double IntegralAlpha, Alpha, omg, omgstep;
    
    double unoccupied_averageT;
    unoccupied_averageT = pieceTime.mean();
    
    double cost_corrb=0, cost_v=0, cost_a=0, cost_omega = 0, cost_domega=0, cost_endp=0, cost_moment=0, cost_meanT=0, cost_centripetal_acc=0;
    
    double violaAcc, violaAlp, violaPos, violaMom, violaCenAcc;
    double violaAccPena, violaAlpPena, violaPosPena, violaMomPena, violaCenAccPena;
    double violaAccPenaD, violaAlpPenaD, violaPosPenaD, violaMomPenaD, violaCenAccPenaD;
    double gradViolaAT, gradViolaDOT, gradViolaPt, gradViolaMt, gradViolaCAt;

    double violaVel, violaVelPena, violaVelPenaD;
    double violaOmega, violaOmegaPena, violaOmegaPenaD;

    Eigen::Matrix2d help_L;
    Eigen::Vector3d gradESDF;
    Eigen::Vector2d gradESDF2d;
    

    // Used to obtain the position of each integral point
    std::vector<Eigen::VectorXd> VecIntegralX;
    std::vector<Eigen::VectorXd> VecIntegralY;
    std::vector<Eigen::Vector2d> VecTrajFinalXY;
    VecTrajFinalXY.emplace_back(ini_x, ini_y);

    // Store derivatives for chain rule
    std::vector<Eigen::MatrixXd> VecSingleXGradCS;
    std::vector<Eigen::MatrixXd> VecSingleXGradCTheta;
    std::vector<Eigen::VectorXd> VecSingleXGradT;
    std::vector<Eigen::MatrixXd> VecSingleYGradCS;
    std::vector<Eigen::MatrixXd> VecSingleYGradCTheta;
    std::vector<Eigen::VectorXd> VecSingleYGradT;

    Eigen::MatrixXd SingleXGradCS(6,SamNumEachPart+1);
    Eigen::MatrixXd SingleXGradCTheta(6,SamNumEachPart+1);
    Eigen::VectorXd SingleXGradT(SamNumEachPart+1);
    Eigen::MatrixXd SingleYGradCS(6,SamNumEachPart+1);
    Eigen::MatrixXd SingleYGradCTheta(6,SamNumEachPart+1);
    Eigen::VectorXd SingleYGradT(SamNumEachPart+1);
    Eigen::VectorXd IntegralX(sparseResolution_);
    Eigen::VectorXd IntegralY(sparseResolution_);

    // Used to store the positions obtained by integration
    Eigen::VectorXd VecCoeffChainX(TrajNum*(SamNumEachPart+1));VecCoeffChainX.setZero();
    Eigen::VectorXd VecCoeffChainY(TrajNum*(SamNumEachPart+1));VecCoeffChainY.setZero();
    Eigen::Vector2d CurrentPointXY(ini_x, ini_y);

    for(int i=0; i<TrajNum; i++){
        const Eigen::Matrix<double, 6, 2> &c = Minco.getCoeffs().block<6,2>(6*i, 0);
        double step = pieceTime[i] / sparseResolution_;
        double halfstep = step / 2.0;
        double CoeffIntegral = pieceTime[i] / sparseResolution_6_;
        
        IntegralX.setZero();
        IntegralY.setZero();
        
        s1 = 0.0;

        for(int j=0; j<=SamNumEachPart; j++){
            if(j%2 == 0){
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                s1 += halfstep;        
                IntegralAlpha = 1.0 / SamNumEachPart * j;
                Alpha = 1.0 / sparseResolution_ * (double(j)/2); 
                omg = (j==0||j==SamNumEachPart)? 0.5:1;
                omgstep = omg * step;
                sigma = c.transpose() * beta0;
                dsigma = c.transpose() * beta1;
                ddsigma = c.transpose() * beta2;
                dddsigma = c.transpose() * beta3;
                // Store gradients for beta0, beta1, and beta2 to simplify calculations. Columns represent yaw and s, rows represent beta0, beta1, and beta2.
                Eigen::MatrixXd gradBeta;gradBeta.resize(3,2);gradBeta.setZero();
                // Store cos and sin to simplify calculations
                double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());
                

                if(if_standard_diff_){
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * dsigma.y() * cosyaw;
                        IntegralY[j/2-1] += CoeffIntegral * dsigma.y() * sinyaw;
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * dsigma.y() * cosyaw;
                        IntegralY[j/2] += CoeffIntegral * dsigma.y() * sinyaw;
                    }

                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
                }
                else{
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2-1] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }

                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                                        + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                                    + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                                        - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                                    + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
                }
                
                
                violaAcc = ddsigma.y()*ddsigma.y() - config_.max_acc_*config_.max_acc_;
                violaAlp = ddsigma.x()*ddsigma.x() - config_.max_domega_*config_.max_domega_;
                
                if(violaAcc > 0){
                    positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
                    gradViolaAT = 2.0 * Alpha * ddsigma.y() * dddsigma.y();
                    gradBeta(2,1) +=  omgstep * penaltyWt.acc_weight * violaAccPenaD * 2.0 * ddsigma.y();
                    partialGradByTimes(i) += omg * penaltyWt.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / sparseResolution_);
                    cost += omgstep * penaltyWt.acc_weight * violaAccPena;
                    cost_a += omgstep * penaltyWt.acc_weight * violaAccPena;
                }
                if(violaAlp > 0){
                    positiveSmoothedL1(violaAlp, violaAlpPena, violaAlpPenaD);
                    gradViolaDOT = 2.0 * Alpha * ddsigma.x() * dddsigma.x();
                    gradBeta(2,0) += omgstep * penaltyWt.domega_weight * violaAlpPenaD * 2.0 * ddsigma.x();
                    partialGradByTimes(i) += omg * penaltyWt.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / sparseResolution_);
                    cost += omgstep * penaltyWt.domega_weight * violaAlpPena;
                    cost_domega += omgstep * penaltyWt.domega_weight * violaAlpPena;
                }

                if(config_.if_directly_constrain_v_omega_){
                    // Directly constrain velocity and angular velocity
                    violaVel = dsigma.y() * dsigma.y() - config_.max_vel_ * config_.max_vel_;
                    if(violaVel > 0){
                        positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);
                        gradViolaPt = 2.0 * Alpha * dsigma.y()
                                        * ddsigma.y();
                        gradBeta(1,1) += omgstep * penaltyWt.moment_weight * violaVelPenaD * 2.0 * dsigma.y();
                        partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaVelPenaD * gradViolaPt * step + violaVelPena / sparseResolution_);
                        cost += omgstep * penaltyWt.moment_weight * violaVelPena;
                        cost_moment += omgstep * penaltyWt.moment_weight * violaVelPena;
                    }
                    violaOmega = dsigma.x() * dsigma.x() - config_.max_omega_ * config_.max_omega_;
                    if(violaOmega > 0){
                        positiveSmoothedL1(violaOmega, violaOmegaPena, violaOmegaPenaD);
                        gradViolaPt = 2.0 * Alpha * dsigma.x()
                                        * ddsigma.x();
                        gradBeta(1,0) += omgstep * penaltyWt.moment_weight * violaOmegaPenaD * 2.0 * dsigma.x();
                        partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaOmegaPenaD * gradViolaPt * step + violaOmegaPena / sparseResolution_);
                        cost += omgstep * penaltyWt.moment_weight * violaOmegaPena;
                        cost_moment += omgstep * penaltyWt.moment_weight * violaOmegaPena;
                    }
                }
                else{
                    // Handle the constraints on speed and angular velocity caused by the driving wheel torque. 
                    // The polynomial inequality forms a symmetric quadrilateral, so four hyperplanes are used for constraint.
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
                        violaMom = omg_sym * config_.max_vel_ * dsigma.x() + config_.max_omega_ * dsigma.y() - config_.max_vel_ * config_.max_omega_;
                        if(violaMom > 0){
                            positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = Alpha * (omg_sym * config_.max_vel_ * ddsigma.x() + config_.max_omega_ * ddsigma.y());
                            gradBeta(1,0) += omgstep * penaltyWt.moment_weight * violaMomPenaD * omg_sym * config_.max_vel_;
                            gradBeta(1,1) += omgstep * penaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                            partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                            cost += omgstep * penaltyWt.moment_weight * violaMomPena;
                            cost_moment += omgstep * penaltyWt.moment_weight * violaMomPena;
                        }
                    }
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
                        violaMom = omg_sym * -config_.min_vel_ * dsigma.x() - config_.max_omega_ * dsigma.y() + config_.min_vel_ * config_.max_omega_;
                        if(violaMom > 0){
                            positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = Alpha * (omg_sym * -config_.min_vel_ * ddsigma.x() - config_.max_omega_ * ddsigma.y());
                            gradBeta(1,0) += omgstep * penaltyWt.moment_weight * violaMomPenaD * omg_sym * -config_.min_vel_;
                            gradBeta(1,1) -= omgstep * penaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                            partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                            cost += omgstep * penaltyWt.moment_weight * violaMomPena;
                            cost_moment += omgstep * penaltyWt.moment_weight * violaMomPena;
                        }
                    }
                }
                // Anti-skid or anti-rollover constraint
                violaCenAcc = dsigma.x()*dsigma.x()*dsigma.y()*dsigma.y() - config_.max_centripetal_acc_*config_.max_centripetal_acc_;
                if(violaCenAcc > 0){
                    positiveSmoothedL1(violaCenAcc, violaCenAccPena, violaCenAccPenaD);
                    gradViolaCAt = 2.0 * Alpha * (dsigma.x() * dsigma.y() * dsigma.y() * ddsigma.x() + dsigma.y() * dsigma.x() * dsigma.x() * ddsigma.y());
                    gradBeta(1,0) += omgstep * penaltyWt.cen_acc_weight * violaCenAccPenaD * (2 * dsigma.x() * dsigma.y() * dsigma.y());
                    gradBeta(1,1) += omgstep * penaltyWt.cen_acc_weight * violaCenAccPenaD * (2 * dsigma.x() * dsigma.x() * dsigma.y());
                    partialGradByTimes(i) += omg * penaltyWt.cen_acc_weight * (violaCenAccPenaD * gradViolaCAt * step + violaCenAccPena / sparseResolution_);
                    cost += omgstep * penaltyWt.cen_acc_weight * violaCenAccPena;
                    cost_centripetal_acc += omgstep * penaltyWt.cen_acc_weight * violaCenAccPena;
                }

                // Collision constraint
                if(j != 0) CurrentPointXY+=Eigen::Vector2d(IntegralX[j/2-1],IntegralY[j/2-1]);

                Eigen::Matrix2d ego_R;
                ego_R << cosyaw,-sinyaw, sinyaw, cosyaw;

                bool if_coolision = false;
                Eigen::Vector2d all_grad2Pos; all_grad2Pos.setZero();


                for(auto cp2D:check_point){
                    Eigen::Vector2d bpt = CurrentPointXY + ego_R * cp2D;
                    double sdf_value = map_->occupancy_grid_map.getDistWithGradBilinear(bpt, gradESDF2d, safeDis);
                    violaPos = -sdf_value + safeDis;
                    if (violaPos > 0.0){
                        if_coolision = true;
                        positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
                        all_grad2Pos -= omgstep * penaltyWt.collision_weight * violaPosPenaD * gradESDF2d;
                        help_L << -sinyaw, -cosyaw, cosyaw, -sinyaw;
                        gradViolaPt = -Alpha * dsigma.x() * gradESDF2d.transpose() * help_L * cp2D;
                        
                        gradBeta(0, 0) -= omgstep * penaltyWt.collision_weight * violaPosPenaD * gradESDF2d.transpose() * help_L * cp2D;
                        partialGradByTimes(i) += omg * penaltyWt.collision_weight * (violaPosPenaD * gradViolaPt * step + violaPosPena / sparseResolution_);
                        cost += omgstep * penaltyWt.collision_weight * violaPosPena;
                        cost_corrb += omgstep * penaltyWt.collision_weight * violaPosPena;
                    }
                }
                if(if_coolision){
                    VecCoeffChainX.head(i*(SamNumEachPart+1)+j+1).array() += all_grad2Pos.x();
                    VecCoeffChainY.head(i*(SamNumEachPart+1)+j+1).array() += all_grad2Pos.y();
                }

                partialGradByCoeffs.block<6,2>(i*6, 0) += beta0 * gradBeta.row(0) + beta1 * gradBeta.row(1) + beta2 * gradBeta.row(2);
            }
            else{
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                s1 += halfstep;
                IntegralAlpha = 1.0 / SamNumEachPart * j;
                sigma = c.transpose() * beta0;
                dsigma = c.transpose() * beta1;
                ddsigma = c.transpose() * beta2;
                double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());

                if(if_standard_diff_){
                    IntegralX[j/2] += 4 * CoeffIntegral * dsigma.y() * cosyaw;
                    IntegralY[j/2] += 4 * CoeffIntegral * dsigma.y() * sinyaw;
                    
                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
                }
                else{

                    IntegralX[j/2] += 4 * CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                    IntegralY[j/2] += 4 * CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    
                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                                        + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                                    + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                                        - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                                    + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
                }            
            }
        }

        // segment duration balance
        if( pieceTime[i] < unoccupied_averageT * mean_time_lowBound_){
            cost += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            cost_meanT += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            partialGradByTimes.array() += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_)  * (- mean_time_lowBound_ / TrajNum);
            partialGradByTimes(i) += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
        }
        if (pieceTime[i] > unoccupied_averageT * mean_time_uppBound_){
            cost += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            cost_meanT += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            partialGradByTimes.array() += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (-mean_time_uppBound_ / TrajNum);
            partialGradByTimes(i) += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
        }

        VecIntegralX.push_back(IntegralX);
        VecIntegralY.push_back(IntegralY);
        VecTrajFinalXY.push_back(VecTrajFinalXY[i] + Eigen::Vector2d(IntegralX.sum(), IntegralY.sum()));
        ///////////////////////////////////////////////////////////////////////////
        VecSingleXGradCS.push_back(SingleXGradCS * CoeffIntegral);
        VecSingleXGradCTheta.push_back(SingleXGradCTheta * CoeffIntegral);
        VecSingleXGradT.push_back(SingleXGradT);
        VecSingleYGradCS.push_back(SingleYGradCS * CoeffIntegral);
        VecSingleYGradCTheta.push_back(SingleYGradCTheta * CoeffIntegral);
        VecSingleYGradT.push_back(SingleYGradT);
        ///////////////////////////////////////////////////////////////////////////
    }

    // final position constraint
    FinalIntegralXYError = VecTrajFinalXY.back() - finStateXYTheta.head(2);
    cost += 0.5 * (EqualRho[0] * pow(FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0], 2) + EqualRho[1] * pow(FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1], 2));
    cost_endp += 0.5 * (EqualRho[0] * pow(FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0], 2) + EqualRho[1] * pow(FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1], 2));
    if(ifprint){
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "\033[40;33m iter finStateXY:%f  %f  \033[0m", VecTrajFinalXY.back().x(), VecTrajFinalXY.back().y());
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "\033[40;33m real finStateXY:%f  %f  \033[0m", finStateXYTheta.x(), finStateXYTheta.y());
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "error: %f", FinalIntegralXYError.norm());
    }
    VecCoeffChainX.array() += EqualRho[0] * (FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0]);
    VecCoeffChainY.array() += EqualRho[1] * (FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1]);


    if(ifprint){
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost: %f", cost);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost corridor: %f", cost_corrb);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost end p: %f", cost_endp);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost v: %f", cost_v);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost a: %f", cost_a);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost omega: %f", cost_omega);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost domega: %f", cost_domega);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost moment: %f", cost_moment);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost meanT: %f", cost_meanT);
        RCLCPP_INFO(msp_planner_node_->get_logger(), "cost centripetal_acc: %f", cost_centripetal_acc);
    } 
 
    // Push the coefficients to the gradient, note that this part must be after the final state constraints and collision constraints
    for(int i=0; i<TrajNum; i++){
        ///////////////////////////////////////////////////////////////////////////
        Eigen::VectorXd CoeffX = VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        Eigen::VectorXd CoeffY = VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleXGradCS[i] * CoeffX;
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleXGradCTheta[i] * CoeffX;
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleYGradCS[i] * CoeffY;
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleYGradCTheta[i] * CoeffY;
        ///////////////////////////////////////////////////////////////////////////
        partialGradByTimes(i) += (VecSingleXGradT[i].cwiseProduct(CoeffX)).sum();
        partialGradByTimes(i) += (VecSingleYGradT[i].cwiseProduct(CoeffY)).sum();
    }
}

inline void MSPlanner::positiveSmoothedL1(const double &x, double &f, double &df){
    const double pe = smoothEps;
    const double half = 0.5 * pe;
    const double f3c = 1.0 / (pe * pe);
    const double f4c = -0.5 * f3c / pe;
    const double d2c = 3.0 * f3c;
    const double d3c = 4.0 * f4c;

    if (x < pe){
        f = (f4c * x + f3c) * x * x * x;
        df = (d3c * x + d2c) * x * x;
    }
    else{
        f = x - half;
        df = 1.0;
    }
    return;
}

template <typename EIGENVEC>
inline void MSPlanner::backwardGradT(const Eigen::VectorXd &tau,
                          const Eigen::VectorXd &gradT,
                          EIGENVEC &gradTau){
    const int sizetau = tau.size();
    gradTau.resize(sizetau);
    double gradrt2vt;
    for (int i = 0; i < sizetau; i++){
        if(tau(i)>0){
            gradrt2vt = tau(i)+1.0;
        }
        else{
            double denSqrt = (0.5*tau(i)-1.0)*tau(i)+1.0;
            gradrt2vt = (1.0-tau(i))/(denSqrt*denSqrt);
        }
        gradTau(i) = gradT(i) * gradrt2vt;
    }
    return;
}

void MSPlanner::get_the_predicted_state(const double& time, Eigen::Vector3d& XYTheta, Eigen::Vector3d& VAJ, Eigen::Vector3d& OAJ){

    double check_time = time;
    if(time > final_traj_.getTotalDuration()){
        check_time = final_traj_.getTotalDuration();
    }
    
    double x1, x2, x3, y1, y2, y3;

    XYTheta = final_initStateXYTheta_;

    double step = trajPredictResolution_;
    double halfstep = step / 2.0;
    double step1_6 = step / 6.0;

    int sequence_num = floor(check_time / step);
    double left_time = check_time - sequence_num * step;

    Eigen::Vector2d p1, p2, p3, v1, v2, v3, a3, j3;
    p3 = final_traj_.getPos(0.0);
    v3 = final_traj_.getVel(0.0);
    
    for(int i=0; i<sequence_num; ++i){
        p1 = p3; v1 = v3;
        p2 = final_traj_.getPos(i * step + halfstep);
        v2 = final_traj_.getVel(i * step + halfstep);
        p3 = final_traj_.getPos(i * step + step);
        v3 = final_traj_.getVel(i * step + step);
        if(if_standard_diff_){
            XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
            XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
        }
        else{
            x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
            x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
            x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

            y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
            y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
            y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

            XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
            XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
        }

        XYTheta.z() = p3.x();
    }

    step1_6 = left_time/6.0;
    p1 = p3; v1 = v3;
    p2 = final_traj_.getPos(check_time - left_time / 2.0);
    v2 = final_traj_.getVel(check_time - left_time / 2.0);
    p3 = final_traj_.getPos(check_time);
    v3 = final_traj_.getVel(check_time);

    if(if_standard_diff_){
        XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
        XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
    }
    else{
        x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
        x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
        x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

        y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
        y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
        y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

        XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
        XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
    }

    XYTheta.z() = p3.x();


    a3 = final_traj_.getAcc(check_time);
    j3 = final_traj_.getJer(check_time);

    OAJ << v3.x(), a3.x(), j3.x();
    VAJ << v3.y(), a3.y(), j3.y();
    return; 
}
//从一条已生成的轨迹中，预测机器人在未来某个时间点的状态，并可选地生成路径点。
std::vector<Eigen::Vector3d> MSPlanner::get_the_predicted_state_and_path(const double &start_time, const double &time, 
                                                                       const Eigen::Vector3d &start_XYTheta, Eigen::Vector3d &XYTheta, bool &if_forward){
    std::vector<Eigen::Vector3d> path;
    double check_time = time;
    if(time > final_traj_.getTotalDuration()){
        check_time = final_traj_.getTotalDuration();
    }
    
    double x1, x2, x3, y1, y2, y3;

    XYTheta = start_XYTheta;
    path.push_back(start_XYTheta);

    double step = trajPredictResolution_;
    double halfstep = step / 2.0;
    double step1_6 = step / 6.0;

    int sequence_num = floor((check_time - start_time) / step);
    double left_time = check_time - sequence_num * step - start_time;

    Eigen::Vector2d p1, p2, p3, v1, v2, v3, a3, j3;
    p3 = final_traj_.getPos(start_time);
    v3 = final_traj_.getVel(start_time);
    
    for(int i=0; i<sequence_num; ++i){
        p1 = p3; v1 = v3;
        p2 = final_traj_.getPos(start_time + i * step + halfstep);
        v2 = final_traj_.getVel(start_time + i * step + halfstep);
        p3 = final_traj_.getPos(start_time + i * step + step);
        v3 = final_traj_.getVel(start_time + i * step + step);

        if(if_standard_diff_){
            XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
            XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
        }
        else{
            x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
            x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
            x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

            y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
            y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
            y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

            XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
            XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
        }
        XYTheta.z() = p3.x();
        // path.push_back(XYTheta);
    }

    step1_6 = left_time/6.0;
    p1 = p3; v1 = v3;
    p2 = final_traj_.getPos(check_time - left_time / 2.0);
    v2 = final_traj_.getVel(check_time - left_time / 2.0);
    p3 = final_traj_.getPos(check_time);
    v3 = final_traj_.getVel(check_time);
    if(if_standard_diff_){
        //标准差速模型
        XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
        XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
    }
    else{
        //带ICR模型
        // 舵轮是否可行？
        x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
        x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
        x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

        y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
        y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
        y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

        XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
        XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
    }
    XYTheta.z() = p3.x();
    // path.push_back(XYTheta);

    if_forward = final_traj_.getPos(time).y() - final_traj_.getPos(start_time).y() > 0.0? true:false;
    
    return path;
}

double MSPlanner::costFunctionCallbackPath(void *ptr,
                                         const Eigen::VectorXd &x,
                                         Eigen::VectorXd &g){
    if(x.norm()>1e4){
        return inf;
    }
    MSPlanner &obj = *(MSPlanner *)ptr;
    ++obj.iter_num_;
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data()+offset, 2, obj.TrajNum - 1);
    offset += 2 * (obj.TrajNum - 1);

    double* gradTailS = g.data()+offset;
    obj.finState(1,0) = x[offset];
    ++offset;

    gradP.setZero();
    obj.Innerpoints = P;
    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum);
    Eigen::Map<Eigen::VectorXd> gradt(g.data()+offset, obj.TrajNum);
    offset += obj.TrajNum;
    obj.VirtualT2RealT(t, obj.pieceTime);
    gradt.setZero();
    double cost;
    obj.Minco.setTConditions(obj.finState);
    obj.Minco.setParameters(obj.Innerpoints,obj.pieceTime);
    obj.Minco.getEnergy(cost);
    obj.Minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.Minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
    obj.attachPenaltyFunctionalPath(cost);
    obj.Minco.propogateArcYawLenghGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                        obj.gradByPoints, obj.gradByTimes, obj.gradByTailStateS);

    *gradTailS = obj.gradByTailStateS.y();

    cost += obj.PathpenaltyWt.time_weight * obj.pieceTime.sum();

    Eigen::VectorXd rhotimes;
    rhotimes.resize(obj.gradByTimes.size());
    obj.gradByTimes += obj.penaltyWt.time_weight * rhotimes.setOnes();
    gradP = obj.gradByPoints;
    backwardGradT(t, obj.gradByTimes, gradt);
    
    return cost;
}

void MSPlanner::attachPenaltyFunctionalPath(double &cost){
    double ini_x = iniStateXYTheta.x();
    double ini_y = iniStateXYTheta.y();

    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    int SamNumEachPart = 2 * sparseResolution_;
    double IntegralAlpha, omg;

    double unoccupied_averageT;
    unoccupied_averageT = pieceTime.mean();
    
    double violaPos;

    double violaMom;
    double violaMomPena;
    double violaMomPenaD;

    double cost_bp=0, cost_final_p=0, cost_moment=0, cost_meanT=0;

    Eigen::Matrix2d help_L;
    Eigen::Vector2d gradESDF2d;

    Eigen::VectorXd IntegralChainCoeff(SamNumEachPart + 1);
    IntegralChainCoeff.setZero();
    for(int i=0; i<sparseResolution_; i++){
        IntegralChainCoeff.block(2*i,0,3,1) += Eigen::Vector3d(1.0, 4.0, 1.0);
    }

    std::vector<Eigen::VectorXd> VecIntegralX(TrajNum);
    std::vector<Eigen::VectorXd> VecIntegralY(TrajNum);
    std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum+1);
    VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

    std::vector<Eigen::MatrixXd> VecSingleXGradCS(TrajNum);
    std::vector<Eigen::MatrixXd> VecSingleXGradCTheta(TrajNum);
    std::vector<Eigen::VectorXd> VecSingleXGradT(TrajNum);
    std::vector<Eigen::MatrixXd> VecSingleYGradCS(TrajNum);
    std::vector<Eigen::MatrixXd> VecSingleYGradCTheta(TrajNum);
    std::vector<Eigen::VectorXd> VecSingleYGradT(TrajNum);

    Eigen::VectorXd VecCoeffChainX(TrajNum*(SamNumEachPart+1));VecCoeffChainX.setZero();
    Eigen::VectorXd VecCoeffChainY(TrajNum*(SamNumEachPart+1));VecCoeffChainY.setZero();
    // Eigen::Vector2d CurrentPointXY(ini_x, ini_y);

    for(int i=0; i<TrajNum; i++){
        const Eigen::Matrix<double, 6, 2> &c = Minco.getCoeffs().block<6,2>(6*i, 0);
        double step = pieceTime[i] / sparseResolution_;
        double halfstep = step / 2;
        double CoeffIntegral = pieceTime[i] / sparseResolution_ / 6;
        Eigen::MatrixXd SingleXGradCS(6,SamNumEachPart+1);
        Eigen::MatrixXd SingleXGradCTheta(6,SamNumEachPart+1);
        Eigen::VectorXd SingleXGradT(SamNumEachPart+1);
        Eigen::MatrixXd SingleYGradCS(6,SamNumEachPart+1);
        Eigen::MatrixXd SingleYGradCTheta(6,SamNumEachPart+1);
        Eigen::VectorXd SingleYGradT(SamNumEachPart+1);

        Eigen::VectorXd IntegralX(sparseResolution_);IntegralX.setZero();
        Eigen::VectorXd IntegralY(sparseResolution_);IntegralY.setZero();
        s1 = 0.0;
        for(int j=0; j<=SamNumEachPart; j++){
            if(j%2 == 0){
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                s1 += halfstep;        
                IntegralAlpha = 1.0 / SamNumEachPart * j;
                omg = (j==0||j==SamNumEachPart)? 0.5:1;
                sigma = c.transpose() * beta0;
                dsigma = c.transpose() * beta1;
                ddsigma = c.transpose() * beta2;
                dddsigma = c.transpose() * beta3;
                double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());

                if(if_standard_diff_){
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * dsigma.y() * cosyaw;
                        IntegralY[j/2-1] += CoeffIntegral * dsigma.y() * sinyaw;
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * dsigma.y() * cosyaw;
                        IntegralY[j/2] += CoeffIntegral * dsigma.y() * sinyaw;
                    }

                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
                }
                else{
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2-1] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }

                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                                        + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                                    + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                                        - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                                    + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
                }

                // Path similarity constraint
                // if(j != 0) CurrentPointXY+=Eigen::Vector2d(IntegralX[j/2-1],IntegralY[j/2-1]);
                
                double gradViolaMt;
                double Alpha = 1.0 / sparseResolution_ * (double(j)/2); 
                Eigen::MatrixXd gradBeta;gradBeta.resize(3,2);gradBeta.setZero();
                for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
                    violaMom = omg_sym * config_.max_vel_ * dsigma.x() + config_.max_omega_ * dsigma.y() - config_.max_vel_ * config_.max_omega_;
                    if(violaMom > 0){
                        positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                        gradViolaMt = Alpha * (omg_sym * config_.max_vel_ * ddsigma.x() + config_.max_omega_ * ddsigma.y());
                        gradBeta(1,0) += omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * omg_sym * config_.max_vel_;
                        gradBeta(1,1) += omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                        partialGradByTimes(i) += omg * PathpenaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                        cost += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                        cost_moment += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                    }
                }
                for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
                    violaMom = omg_sym * -config_.min_vel_ * dsigma.x() - config_.max_omega_ * dsigma.y() + config_.min_vel_ * config_.max_omega_;
                    if(violaMom > 0){
                        positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                        gradViolaMt = Alpha * (omg_sym * -config_.min_vel_ * ddsigma.x() - config_.max_omega_ * ddsigma.y());
                        gradBeta(1,0) += omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * omg_sym * -config_.min_vel_;
                        gradBeta(1,1) -= omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                        partialGradByTimes(i) += omg * PathpenaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                        cost += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                        cost_moment += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                    }
                }

                double violaAcc = ddsigma.y()*ddsigma.y() - config_.max_acc_*config_.max_acc_;
                double violaAlp = ddsigma.x()*ddsigma.x() - config_.max_domega_*config_.max_domega_;
                double violaAccPena, violaAccPenaD, violaAlpPena, violaAlpPenaD;
                if(violaAcc > 0){
                    positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
                    double gradViolaAT = 2.0 * Alpha * ddsigma.y() * dddsigma.y();
                    gradBeta(2,1) +=  omg * step * PathpenaltyWt.acc_weight * violaAccPenaD * 2.0 * ddsigma.y();
                    partialGradByTimes(i) += omg * PathpenaltyWt.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / sparseResolution_);
                    cost += omg * step * PathpenaltyWt.acc_weight * violaAccPena;
                    cost_moment += omg * step * PathpenaltyWt.acc_weight * violaAccPena;
                }
                if(violaAlp > 0){
                    positiveSmoothedL1(violaAlp, violaAlpPena, violaAlpPenaD);
                    double gradViolaDOT = 2.0 * Alpha * ddsigma.x() * dddsigma.x();
                    gradBeta(2,0) += omg * step * PathpenaltyWt.domega_weight * violaAlpPenaD * 2.0 * ddsigma.x();
                    partialGradByTimes(i) += omg * PathpenaltyWt.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / sparseResolution_);
                    cost += omg * step * PathpenaltyWt.domega_weight * violaAlpPena;
                    cost_moment += omg * step * PathpenaltyWt.domega_weight * violaAlpPena;
                }

                partialGradByCoeffs.block<6,2>(i*6, 0) += beta0 * gradBeta.row(0) + beta1 * gradBeta.row(1) + beta2 * gradBeta.row(2);
            }
            else{
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                s1 += halfstep;
                IntegralAlpha = 1.0 / SamNumEachPart * j;
                sigma = c.transpose() * beta0;
                dsigma = c.transpose() * beta1;
                ddsigma = c.transpose() * beta2;

                double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());
                

                if(if_standard_diff_){
                    IntegralX[j/2] += 4 * CoeffIntegral * dsigma.y() * cosyaw;
                    IntegralY[j/2] += 4 * CoeffIntegral * dsigma.y() * sinyaw;
                    
                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
                }
                else{

                    IntegralX[j/2] += 4 * CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                    IntegralY[j/2] += 4 * CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    
                    SingleXGradCS.col(j) = beta1 * cosyaw;
                    SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
                    SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                                        + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                                    + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

                    SingleYGradCS.col(j) = beta1 * sinyaw;
                    SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
                    SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                                        - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                                    + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
                }
            }
        }
        VecIntegralX[i] = IntegralX;
        VecIntegralY[i] = IntegralY;
        VecTrajFinalXY[i+1] = VecTrajFinalXY[i] + Eigen::Vector2d(IntegralX.sum(), IntegralY.sum());
        VecSingleXGradCS[i] = SingleXGradCS * CoeffIntegral;
        VecSingleXGradCTheta[i] = SingleXGradCTheta * CoeffIntegral;
        VecSingleXGradT[i] = SingleXGradT ;
        VecSingleYGradCS[i] = SingleYGradCS * CoeffIntegral;
        VecSingleYGradCTheta[i] = SingleYGradCTheta * CoeffIntegral;
        VecSingleYGradT[i] = SingleYGradT;

        if( pieceTime[i] < unoccupied_averageT * mean_time_lowBound_){
            cost += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            cost_meanT += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            partialGradByTimes.array() += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_)  * (- mean_time_lowBound_ / TrajNum);
            partialGradByTimes(i) += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
        }
        if (pieceTime[i] > unoccupied_averageT * mean_time_uppBound_){
            cost += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            cost_meanT += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            partialGradByTimes.array() += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_)  * (- mean_time_uppBound_ / TrajNum);
            partialGradByTimes(i) += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
        }

        // Path point constraint
        Eigen::Vector2d innerpointXY = VecTrajFinalXY[i+1];
        violaPos = (innerpointXY - inner_init_positions[i].head(2)).squaredNorm();
        VecCoeffChainX.head((i+1)*(SamNumEachPart+1)).array() += PathpenaltyWt.bigpath_sdf_weight * 2.0 * (innerpointXY.x() - inner_init_positions[i].x());
        VecCoeffChainY.head((i+1)*(SamNumEachPart+1)).array() += PathpenaltyWt.bigpath_sdf_weight * 2.0 * (innerpointXY.y() - inner_init_positions[i].y());
        cost += PathpenaltyWt.bigpath_sdf_weight * violaPos;
        cost_bp += PathpenaltyWt.bigpath_sdf_weight * violaPos;

    }

    if(ifprint){
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "cost: %f", cost);
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "cost big path dis: %f", cost_bp);
        ///RCLCPP_INFO(msp_planner_node_->get_logger(), "cost final p: %f", cost_final_p);
        //RCLCPP_INFO(msp_planner_node_->get_logger(), "cost moment: %f", cost_moment);
    } 

    for(int i=0; i<TrajNum; i++){
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleXGradCS[i] * VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleXGradCTheta[i] * VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleYGradCS[i] * VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleYGradCTheta[i] * VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByTimes(i) += (VecSingleXGradT[i].cwiseProduct(VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff))).sum();
        partialGradByTimes(i) += (VecSingleYGradT[i].cwiseProduct(VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff))).sum();
    }
}

    void MSPlanner::mincoPathPub(const Trajectory<5, 2> &final_traj, 
                        const Eigen::Vector3d &start_state_XYTheta, 
                        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher){
    double ini_x = start_state_XYTheta.x();
    double ini_y = start_state_XYTheta.y();

    double s1;
    int sparseResolution = sparseResolution_ * 10;
    int SamNumEachPart = 2 * sparseResolution;
    double sumT = 0.0;

    int TrajNum = final_traj.getPieceNum();
    Eigen::VectorXd pieceTime = final_traj.getDurations();

    std::vector<Eigen::VectorXd> VecIntegralX(TrajNum);
    std::vector<Eigen::VectorXd> VecIntegralY(TrajNum);
    std::vector<Eigen::VectorXd> VecYaw(TrajNum);
    std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum+1);
    VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

    for(int i=0; i<TrajNum; i++){
        double step = pieceTime[i] / sparseResolution;
        double halfstep = step / 2.0;
        double CoeffIntegral = pieceTime[i] / sparseResolution / 6.0;

        Eigen::VectorXd IntegralX(sparseResolution);IntegralX.setZero();
        Eigen::VectorXd IntegralY(sparseResolution);IntegralY.setZero();
        Eigen::VectorXd Yaw(sparseResolution);Yaw.setZero();
        s1 = 0.0;
        for(int j=0; j<=SamNumEachPart; j++){
            if(j%2 == 0){
                Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
                s1 += halfstep;

                double cosyaw = cos(currPos.x()), sinyaw = sin(currPos.x());
                if(if_standard_diff_){
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
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2-1] += CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
                        Yaw[j/2-1] = currPos.x();
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2] += CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
                    }
                }
                // outFile << currPos.x() << " " << currPos.y() << std::endl;
            }
            else{
                Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
                s1 += halfstep;
                double cosyaw = cos(currPos.x()), sinyaw = sin(currPos.x());
                if(if_standard_diff_){
                    IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
                    IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
                }
                else{
                    IntegralX[j/2] += 4 * CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
                    IntegralY[j/2] += 4 * CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
                }
                // outFile << currPos.x() << " " << currPos.y() << std::endl;
            }
        }
        VecIntegralX[i] = IntegralX;
        VecIntegralY[i] = IntegralY;
        VecYaw[i] = Yaw;
        sumT += pieceTime[i];
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = "world";
    path.header.stamp = msp_planner_node_->now();
    Eigen::Vector2d pos(ini_x, ini_y);
    for(u_int i=0; i<VecIntegralX.size(); i++){
        for(u_int j=0; j<VecIntegralX[i].size(); j++){
            pos.x() += VecIntegralX[i][j];
            pos.y() += VecIntegralY[i][j];
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = msp_planner_node_->now();
            pose.pose.position.x = pos.x();
            pose.pose.position.y = pos.y();
            pose.pose.position.z = 0.15;
            tf2::Quaternion q;
            q.setRPY(0, 0, VecYaw[i][j]);  // roll=0, pitch=0, yaw=init_point.z()
            pose.pose.orientation = tf2::toMsg(q); 
            path.poses.push_back(pose);
            // outFile2 << pos.x() << " " << pos.y() << std::endl;
        }
    }
    publisher->publish(path);
    // outFile.close();
    // outFile2.close();
}

void MSPlanner::mincoPointPub(const Trajectory<5,2> &final_traj, 
    const Eigen::Vector3d &start_state_XYTheta, 
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,  
    const Eigen::Vector3d &color){
    double ini_x = start_state_XYTheta.x();
    double ini_y = start_state_XYTheta.y();

    double s1;
    int sparseResolution = sparseResolution_ * 10;
    int SamNumEachPart = 2 * sparseResolution;
    double sumT = 0.0;

    int TrajNum = final_traj.getPieceNum();
    Eigen::VectorXd pieceTime = final_traj.getDurations();

    std::vector<Eigen::VectorXd> VecIntegralX(TrajNum);
    std::vector<Eigen::VectorXd> VecIntegralY(TrajNum);
    std::vector<Eigen::VectorXd> VecYaw(TrajNum);
    std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum+1);
    VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

    for(int i=0; i<TrajNum; i++){
        double step = pieceTime[i] / sparseResolution;
        double halfstep = step / 2.0;
        double CoeffIntegral = pieceTime[i] / sparseResolution / 6.0;

        Eigen::VectorXd IntegralX(sparseResolution);IntegralX.setZero();
        Eigen::VectorXd IntegralY(sparseResolution);IntegralY.setZero();
        Eigen::VectorXd Yaw(sparseResolution);Yaw.setZero();
        s1 = 0.0;
        for(int j=0; j<=SamNumEachPart; j++){
            if(j%2 == 0){
                Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
                s1 += halfstep;
                double cosyaw = cos(currPos.x()), sinyaw = sin(currPos.x());
                if(if_standard_diff_){
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
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2-1] += CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
                        Yaw[j/2-1] = currPos.x();
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2] += CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
                    }
                }
            }
            else{
                Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
                s1 += halfstep;
                double cosyaw = cos(currPos.x()), sinyaw = sin(currPos.x());
                if(if_standard_diff_){
                    IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
                    IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
                }
                else{
                    IntegralX[j/2] += 4 * CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
                    IntegralY[j/2] += 4 * CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
                }
            }
        }
        VecIntegralX[i] = IntegralX;
        VecIntegralY[i] = IntegralY;
        VecYaw[i] = Yaw;
        // VecTrajFinalXY[i+1] = Eigen::Vector2d(IntegralX[IntegralX.size()-1], IntegralY[IntegralX.size()-1]);
        sumT += pieceTime[i];
    }

    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker.ns = "Innerpoint";
    marker.header.frame_id = "world";
    marker.header.stamp = msp_planner_node_->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    publisher->publish(marker);
    
    
    marker.action = visualization_msgs::msg::Marker::ADD;    
    marker.id = 0;
    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;
    marker.color.a = 1.0;
    marker.color.r = color.x()/255.0;
    marker.color.g = color.y()/255.0;
    marker.color.b = color.z()/255.0;
    Eigen::Vector2d pos(ini_x, ini_y);
    for(u_int i=0; i<VecIntegralX.size(); i++){
        for(u_int j=0; j<VecIntegralX[i].size(); j++){
            pos.x() += VecIntegralX[i][j];
            pos.y() += VecIntegralY[i][j];
            geometry_msgs::msg::Point pt;
            pt.x = pos.x();
            pt.y = pos.y();
            pt.z = 0.15;
            if(j == VecIntegralX[i].size()-1){
                marker.points.push_back(pt);
            }
        }
    }
    publisher->publish(marker);
}


inline double MSPlanner::normlize_angle(double angle){
    if(angle > M_PI) angle -= 2 * M_PI;
    else if(angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void MSPlanner::Collision_point_Pub(){
    sensor_msgs::msg::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::PointXYZRGB  pt;
    for(u_int i=0;i<collision_point_.size();i++){
        pt.x = collision_point_[i][0];
        pt.y = collision_point_[i][1];
        pt.z = 0.3;
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        colored_pcl_ptr->points.push_back(pt); 
    }
    cloudMap.height = colored_pcl_ptr->points.size();
    cloudMap.width = 1;
    // cloudMap.is_dense = true;
    pcl::toROSMsg(*colored_pcl_ptr,globalMap_pcd);
    globalMap_pcd.header.stamp = msp_planner_node_->now();
    globalMap_pcd.header.frame_id = "world";
    CollisionpointPub->publish(globalMap_pcd);
}

void MSPlanner::pub_inner_init_positions(const std::vector<Eigen::Vector3d> &inner_init_positions){
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker.ns = "Innerpoint";
    marker.header.frame_id = "world";
    marker.header.stamp = msp_planner_node_->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    innerinitpositionsPoint->publish(marker);
    
    
    marker.action = visualization_msgs::msg::Marker::ADD;    
    marker.id = 0;
    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;
    marker.color.a = 1.0;
    marker.color.r = 193.0/255.0;
    marker.color.g = 125.0/255.0;
    marker.color.b = 17.0/255.0;
    for(u_int i=0; i<inner_init_positions.size(); i++){
        geometry_msgs::msg::Point pt;
        pt.x = inner_init_positions[i].x();
        pt.y = inner_init_positions[i].y();
        pt.z = 0.15;
        marker.points.push_back(pt);
    }
    innerinitpositionsPoint->publish(marker);
}