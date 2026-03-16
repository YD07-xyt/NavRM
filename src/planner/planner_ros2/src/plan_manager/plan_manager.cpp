#include"plan_manager/plan_manager.hpp"

PlanManager::PlanManager(const rclcpp::NodeOptions & options): Node("plan_manager", options),
                                                  config_reader_(this->shared_from_this()) {
      
      /** 
       * TODO: init params 
      */
      get_params();
     //===============

      sdfmap_ = std::make_shared<planner::map::Map>(occmap_config,this->shared_from_this());
      
      msplanner_ = std::make_shared<MSPlanner>(conf, sdfmap_,
            msp_config,penaltyWt,PathpenaltyWt ,path_lbfgs_params,this->shared_from_this());
      
      jps_planner_ = std::make_shared<JPS::JPSPlanner>(sdfmap_,jps_config);

      //发速度
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);  
      
      //接受目标点
      goal_sub_ =this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal",10, std::bind(&PlanManager::goal_callback, this, std::placeholders::_1));
      //odom
      current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10, std::bind(&PlanManager::GeometryCallback, this, std::placeholders::_1));
      
      //0.001
      main_thread_timer_ =this->create_wall_timer(std::chrono::milliseconds(1),  // 1ms周期
        std::bind(&PlanManager::MainThread, this));

      //紧急停车消息
      emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/emergency_stop",1);

      // 记录规划时间的消息
      record_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planner/calculator_time",1);
      
      /** 
      TODO: mpc 控制部分
       */
      // mpc_polynome_pub_ = nh_.advertise<carstatemsgs::Polynome>("traj", 1);

      have_geometry_ = false;
      
      have_goal_ = false;
      
      
      if_fix_final_ = this->declare_parameter<bool>("if_fix_final", false);
        
      if (if_fix_final_) {
            final_state_(0) = this->declare_parameter<double>("final_x", 0.0);
            final_state_(1) = this->declare_parameter<double>("final_y", 0.0);
            final_state_(2) = this->declare_parameter<double>("final_yaw", 0.0);
      }
        
      replan_time_ = this->declare_parameter<double>("replan_time", 10000.0);
      
      max_replan_time_ = this->declare_parameter<double>("max_replan_time", 1.0);

      state_machine_ = StateMachine::IDLE;

      loop_start_time_ = this->now();

    }

    void PlanManager::MainThread(){
      
      if(!have_geometry_ || !have_goal_) {
        RCLCPP_INFO(this->get_logger(),"Waiting for odom or goal...");
        return;
      }
      // collision check
      if(have_geometry_){


        //判断当前位置是否在障碍物内
        if(sdfmap_->occupancy_grid_map.getDistanceReal(
            Eigen::Vector2d(current_state_XYTheta_.x(), current_state_XYTheta_.y())) < 0.0){
          /**
          *  TODO：这个功能现在是否启用？
          */
          
          // 发布紧急停车消息，进入紧急停车状态
          std_msgs::msg::Bool emergency_stop;//紧急停止
          
          emergency_stop.data = true;
          
          emergency_stop_pub_->publish(emergency_stop);
          
          state_machine_ = EMERGENCY_STOP;
          
          RCLCPP_INFO_STREAM(this->get_logger(),"current_state_XYTheta_: " 
            << current_state_XYTheta_.transpose());

          RCLCPP_INFO_STREAM(this->get_logger(),"Dis: " 
            << sdfmap_->occupancy_grid_map.getDistanceReal
            (Eigen::Vector2d(current_state_XYTheta_.x(), current_state_XYTheta_.y())));
          
          RCLCPP_ERROR(this->get_logger(),"EMERGENCY_STOP!!! too close to obstacle!!!");
          
          return;
        }
      }

      // 如果在空闲状态，或者在规划/重新规划状态但已经超过重新规划时间，则开始新的规划
      if(state_machine_ == StateMachine::IDLE || 
          ((state_machine_ == StateMachine::PLANNING||state_machine_ == StateMachine::REPLAN) 
            && (this->now() - loop_start_time_).seconds() > replan_time_)){
        
        loop_start_time_ = this->now();
        
        double current = loop_start_time_.seconds();
        
        // 开始新规划
        if(state_machine_ == StateMachine::IDLE){
          
          state_machine_ = StateMachine::PLANNING;
          //规划开始时间（？-1表示未设置或无效 ？）
          plan_start_time_ = -1;
          
          // 预测轨迹开始时间（？-1表示未设置或无效 ？）
          predicted_traj_start_time_ = -1;
          
          // 规划开始状态
          plan_start_state_XYTheta = current_state_XYTheta_;
          plan_start_state_VAJ = current_state_VAJ_;
          plan_start_state_OAJ = current_state_OAJ_;
        } 

        // Use predicted distance for replanning in planning state
        // 重新规划时使用预测距离
        else if(state_machine_ == StateMachine::PLANNING || state_machine_ == StateMachine::REPLAN){
          // 预测轨迹开始时间
          /**  squaredNorm()用于计算向量平方范数的函数。它返回向量各分量平方和，但不进行开平方运算。
                fmod(.... , 2.0 * M_PI)：将角度差归一化到 [0, 2π) 范围内
          //  位置误差 + 角度误差(归一化 + 缩放0.02) < 1.0      缩放->角度误差部分影响小于位置误差 
          */ 
          if(((current_state_XYTheta_ - goal_state_).head(2).squaredNorm() + 
            fmod(fabs((plan_start_state_XYTheta - goal_state_)[2]), 2.0 * M_PI)*0.02 < 1.0) ||
            //检查生成的轨迹总时长是否小于最大允许重规划时间 
            msplanner_->final_traj_.getTotalDuration() < max_replan_time_){
              //执行路径，向目标移动
              state_machine_ = StateMachine::GOINGTOGOAL;
              return;
          }

          // 进入重新规划状态
          state_machine_ = StateMachine::REPLAN;

          // 计算预测轨迹开始时间，通常是当前时间加上最大允许重规划时间减去上次规划开始时间
          predicted_traj_start_time_ = current + max_replan_time_ - plan_start_time_;
          // 获取预测轨迹开始时刻的状态，包括位置、速度、加速度等信息
          // ?? TODO： 理解
          msplanner_->get_the_predicted_state(predicted_traj_start_time_, plan_start_state_XYTheta, plan_start_state_VAJ, plan_start_state_OAJ);

        } 
        
        RCLCPP_INFO(this->get_logger(),"\033[32;40m \n\n\n\n\n-------------------------------------start new plan------------------------------------------ \033[0m");
        
        // 用于在 RViz 中可视化起点和终点的位置和朝向
        visualizer_->finalnodePub(plan_start_state_XYTheta, goal_state_);
        
        RCLCPP_INFO(this->get_logger(),"init_state_: %.10f  %.10f  %.10f", plan_start_state_XYTheta(0), plan_start_state_XYTheta(1), plan_start_state_XYTheta(2));
        RCLCPP_INFO(this->get_logger(),"goal_state_: %.10f  %.10f  %.10f", goal_state_(0), goal_state_(1), goal_state_(2));
        /**  
        *  TODO： 优化cout-->rmlog::info
        */
        std::cout<<"<arg name=\"start_x_\" value=\""<< plan_start_state_XYTheta(0) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"start_y_\" value=\""<< plan_start_state_XYTheta(1) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"start_yaw_\" value=\""<< plan_start_state_XYTheta(2) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_x_\" value=\""<< goal_state_(0) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_y_\" value=\""<< goal_state_(1) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_yaw_\" value=\""<< goal_state_(2) <<"\"/>"<<std::endl;

        std::cout<<"plan_start_state_VAJ: "<<plan_start_state_VAJ.transpose()<<std::endl;
        std::cout<<"plan_start_state_OAJ: "<<plan_start_state_OAJ.transpose()<<std::endl;

        RCLCPP_INFO(this->get_logger(),"<arg name=\"start_x_\" value=\"%f\"/>", plan_start_state_XYTheta(0));
        RCLCPP_INFO(this->get_logger(),"<arg name=\"start_y_\" value=\"%f\"/>", plan_start_state_XYTheta(1));
        RCLCPP_INFO(this->get_logger(),"<arg name=\"start_yaw_\" value=\"%f\"/>", plan_start_state_XYTheta(2));
        RCLCPP_INFO(this->get_logger(),"<arg name=\"final_x_\" value=\"%f\"/>", goal_state_(0));
        RCLCPP_INFO(this->get_logger(),"<arg name=\"final_y_\" value=\"%f\"/>", goal_state_(1));
        RCLCPP_INFO(this->get_logger(),"<arg name=\"final_yaw_\" value=\"%f\"/>", goal_state_(2));

        RCLCPP_INFO_STREAM(this->get_logger(),"plan_start_state_VAJ: " << plan_start_state_VAJ.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(),"plan_start_state_OAJ: " << plan_start_state_OAJ.transpose());

        // front end
        // 路径搜索开始时间
        rclcpp::Time astar_start_time = this->now();
        // findJPSRoad()只返回ture? why? TODO:理解
        //永不执行 ？
        if(!findJPSRoad()){
          state_machine_ = EMERGENCY_STOP;
          RCLCPP_ERROR(this->get_logger(),"EMERGENCY_STOP!!! can not find astar road !!!");
          return;
        }
        RCLCPP_INFO(this->get_logger(),
        "\033[41;37m all of front end time:%f \033[0m", (this->now()-astar_start_time).seconds());

        // optimizer
        //TODO:理解为什么要在这里进行优化器的调用?
        bool result = msplanner_->minco_plan(jps_planner_->flat_traj_);
        if(!result){
          // 优化失败 TODO:log
          return;
        }

        RCLCPP_INFO(this->get_logger(),
          "\033[43;32m all of plan time:%f \033[0m", (this->now().seconds()-current));

        // visualization
        // 发布轨迹和路径点到 RViz 进行可视化
        msplanner_->mincoPathPub(msplanner_->final_traj_, 
          plan_start_state_XYTheta, visualizer_->mincoPathPath);
        msplanner_->mincoPointPub(msplanner_->final_traj_, 
          plan_start_state_XYTheta, 
          visualizer_->mincoPointMarker, Eigen::Vector3d(239, 41, 41));
        
        // for replan
        // 记录当前规划的起始状态和时间，以便在需要重新规划时使用
        if(plan_start_time_ < 0){
          // 首次规划 系统刚启动，还没有进行过规划 之前的规划已被清除
          Traj_start_time_ = this->now();
          plan_start_time_ = Traj_start_time_.seconds();
        }
        else{
          //已经有一条轨迹在执行，但需要重新规划
          plan_start_time_ = current + max_replan_time_;
          Traj_start_time_ = rclcpp::Time(plan_start_time_);
        }
        
        //TODO： 待实现
        MPCPathPub(plan_start_time_);

        // 轨迹总时间
        Traj_total_time_ = msplanner_->final_traj_.getTotalDuration();
      }

      // 轨迹完成检测
      if((this->now() - Traj_start_time_).seconds() >= Traj_total_time_){
        state_machine_ = StateMachine::IDLE;
        have_goal_ = false;
      }

    }

    bool PlanManager::findJPSRoad(){

      rclcpp::Time current = this->now();

      Eigen::Vector3d start_state;
      
      std::vector<Eigen::Vector3d> start_path;
      std::vector<Eigen::Vector3d> start_path_both_end;
      
      //是否向前运动
      bool if_forward = true;
      
      if(plan_start_time_ > 0){
        
        start_path = msplanner_->get_the_predicted_state_and_path(predicted_traj_start_time_, 
          predicted_traj_start_time_ + jps_planner_->jps_truncation_time_, 
          plan_start_state_XYTheta,
          start_state, if_forward);
        
        u_int start_path_size = start_path.size();

        //何意味 TODO: start_path_i 更换为更具描述性的变量名
        u_int start_path_i = 0;
        for(; start_path_i < start_path_size; start_path_i++){
          
          //检查路径上的点是否发生碰撞
          if(!jps_planner_->JPS_check_if_collision(start_path[start_path_i].head(2))){
            break; // 遇到第一个不安全点就停止
          }
        
        }
        //路径起点已经处于危险区域
        if(start_path_i == 0){
          start_state = plan_start_state_XYTheta;
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
        //中途遇到不安全点
        else if(start_path_i < start_path_size){
          // 截断路径到第一个不安全点之前
          start_path = std::vector<Eigen::Vector3d>(
              start_path.begin(), start_path.begin() + start_path_i);
          start_state = start_path.back(); // 新的终点是路径最后一个安全点;
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
        //所有点都安全
        else{
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
      }
      else{
        //规划起始时间无效
        //使用保存的规划起点状态
        start_state = plan_start_state_XYTheta;
      }

      //调用JPS路径规划器进行路径规划
      jps_planner_->plan(start_state, goal_state_);
      
      //将规划结果与起点路径进行合并，得到完整的路径
      jps_planner_->getKinoNodeWithStartPath(start_path, if_forward, 
        plan_start_state_VAJ, plan_start_state_OAJ);

      //TODO： 独立的可视化函数
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.ns = "jps_planner";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 11;
      marker.pose.position.y = 8;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.5;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      double search_time = (this->now()-current).seconds() * 1000.0;
      std::ostringstream out;
      out << std::fixed <<"JPS: \n"<< std::setprecision(2) << search_time<<" ms";
      marker.text = out.str();
      record_pub_->publish(marker);


      RCLCPP_INFO(this->get_logger(),"\033[40;36m jps_planner_ search time:%lf  \033[0m", (this->now()-current).seconds());

      return true;
    }

    void PlanManager::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      // Ignore the given goal at runtime, commenting out this check may cause unexpected bugs
      // Especially when there is no re-planning
      if(state_machine_ != StateMachine::IDLE){
        RCLCPP_ERROR(this->get_logger(),"Haven't reached the goal yet!!");
        return;
      }
      
      RCLCPP_INFO(this->get_logger(),"\n\n\n\n\n\n\n\n");
      RCLCPP_INFO(this->get_logger(),"---------------------------------------------------------------");
      RCLCPP_INFO(this->get_logger(),"---------------------------------------------------------------");

      RCLCPP_INFO(this->get_logger(),"get goal!");
      
      state_machine_ = StateMachine::IDLE;
      have_goal_ = true;
      goal_state_<<msg->pose.position.x, msg->pose.position.y, tf2::getYaw(msg->pose.orientation);
      if(if_fix_final_) {
        goal_state_ = final_state_;
      }
      RCLCPP_INFO_STREAM(this->get_logger(),"goal state: " << goal_state_.transpose());

      RCLCPP_INFO(this->get_logger(),"---------------------------------------------------------------");
      RCLCPP_INFO(this->get_logger(),"---------------------------------------------------------------");
      RCLCPP_INFO(this->get_logger(),"\n\n\n\n\n\n\n\n");
    
    }





    void PlanManager::GeometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
      have_geometry_ = true;
      current_state_XYTheta_ << msg->pose.pose.position.x, msg->pose.pose.position.y, 
      tf2::getYaw(msg->pose.pose.orientation);
      current_state_VAJ_ << 0.0, 0.0, 0.0;
      current_state_OAJ_ << 0.0, 0.0, 0.0;
      current_time_ = msg->header.stamp;
    }

void PlanManager::get_params(){
      conf = config_reader_.getConfig();
      msp_config = config_reader_.getMSPConfig();
      penaltyWt = config_reader_.getPenaltyWeights();
      PathpenaltyWt = config_reader_.getPathPenaltyWeights();
      path_lbfgs_params = config_reader_.getPathLbfgsParams();
      jps_config = config_reader_.getJPSPlannerParams();
      occmap_config = config_reader_.getOccupancyGridMapConfig();

}