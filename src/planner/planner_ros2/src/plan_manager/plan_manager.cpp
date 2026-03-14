#include"plan_manager/plan_manager.hpp"

PlanManager::PlanManager(const rclcpp::NodeOptions & options): Node("plan_manager", options){
      
      /** 
       * TODO: init params 
      */
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


      emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/emergency_stop",1);

      record_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planner/calculator_time",1);

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
        
        // start new plan
        if(state_machine_ == StateMachine::IDLE){
          state_machine_ = StateMachine::PLANNING;
          plan_start_time_ = -1;
          predicted_traj_start_time_ = -1;
          plan_start_state_XYTheta = current_state_XYTheta_;
          plan_start_state_VAJ = current_state_VAJ_;
          plan_start_state_OAJ = current_state_OAJ_;
        } 
        // Use predicted distance for replanning in planning state
        else if(state_machine_ == StateMachine::PLANNING || state_machine_ == StateMachine::REPLAN){
          
          if(((current_state_XYTheta_ - goal_state_).head(2).squaredNorm() + fmod(fabs((plan_start_state_XYTheta - goal_state_)[2]), 2.0 * M_PI)*0.02 < 1.0) ||
             msplanner_->final_traj_.getTotalDuration() < max_replan_time_){
            state_machine_ = StateMachine::GOINGTOGOAL;
            return;
          }

          state_machine_ = StateMachine::REPLAN;

          predicted_traj_start_time_ = current + max_replan_time_ - plan_start_time_;
          msplanner_->get_the_predicted_state(predicted_traj_start_time_, plan_start_state_XYTheta, plan_start_state_VAJ, plan_start_state_OAJ);

        } 
        
        RCLCPP_INFO(this->get_logger(),"\033[32;40m \n\n\n\n\n-------------------------------------start new plan------------------------------------------ \033[0m");
        visualizer_->finalnodePub(plan_start_state_XYTheta, goal_state_);
        RCLCPP_INFO(this->get_logger(),"init_state_: %.10f  %.10f  %.10f", plan_start_state_XYTheta(0), plan_start_state_XYTheta(1), plan_start_state_XYTheta(2));
        RCLCPP_INFO(this->get_logger(),"goal_state_: %.10f  %.10f  %.10f", goal_state_(0), goal_state_(1), goal_state_(2));
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
        rclcpp::Time astar_start_time = this->now();
        if(!findJPSRoad()){
          state_machine_ = EMERGENCY_STOP;
          RCLCPP_ERROR(this->get_logger(),"EMERGENCY_STOP!!! can not find astar road !!!");
          return;
        }
        RCLCPP_INFO(this->get_logger(),
        "\033[41;37m all of front end time:%f \033[0m", (this->now()-astar_start_time).seconds());

        // optimizer
        bool result = msplanner_->minco_plan(jps_planner_->flat_traj_);
        if(!result){
          return;
        }

        RCLCPP_INFO(this->get_logger(),
          "\033[43;32m all of plan time:%f \033[0m", (this->now().seconds()-current));

        // visualization
        msplanner_->mincoPathPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->mincoPathPath);
        msplanner_->mincoPointPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->mincoPointMarker, Eigen::Vector3d(239, 41, 41));
        
        // for replan
        if(plan_start_time_ < 0){
          Traj_start_time_ = this->now();
          plan_start_time_ = Traj_start_time_.seconds();
        }
        else{
          plan_start_time_ = current + max_replan_time_;
          Traj_start_time_ = rclcpp::Time(plan_start_time_);
        }
        

        MPCPathPub(plan_start_time_);

        Traj_total_time_ = msplanner_->final_traj_.getTotalDuration();
      }

      if((this->now() - Traj_start_time_).seconds() >= Traj_total_time_){
        state_machine_ = StateMachine::IDLE;
        have_goal_ = false;
      }

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
      if(if_fix_final_) goal_state_ = final_state_;
      
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

