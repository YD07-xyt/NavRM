#ifndef JPS_PLANNER_H
#define JPS_PLANNER_H

#include<rclcpp/rclcpp.hpp>

#include <../../planner_map/include/planner_map.hpp>
#include <../../path_searching/include/jps_planner/graph_search.h>
#include <../../path_searching/include/jps_planner/traj_representation.h>



namespace JPS
{
    struct JPSPlannerParams{
            double safe_dis_;
            double max_jps_dis_;
            double distance_weight_;
            double yaw_weight_;
            double trajCutLength_;
            double max_vel_;
            double max_acc_;
            double max_omega_;
            double max_domega_;
            double sampletime_;
            int mintrajNum_;
    };

    class JPSPlanner
    {
        private:
            
            // params
            double safe_dis_;
            double max_jps_dis_;
            double distance_weight_;
            double yaw_weight_;
            double trajCutLength_;
            double max_vel_;
            double max_acc_;
            double max_omega_;
            double max_domega_;
            double sampletime_;
            int mintrajNum_;

            // data
            Eigen::Vector3d start_state_;// x y yaw
            Eigen::Vector3d current_state_VAJ_;
            Eigen::Vector3d current_state_OAJ_; 
            Eigen::Vector3d end_state_;// x y yaw

            bool if_first_point_cut_;
            
            std::shared_ptr<planner::map::Map> map_util_;
            std::shared_ptr<GraphSearch> graph_search_;

            int status_;
            std::vector<Eigen::Vector2d> raw_path_;
            std::vector<Eigen::Vector2d> path_;

            std::vector<Eigen::Vector2d> Unoccupied_path_;

            std::vector<Eigen::VectorXd> Unoccupied_sample_trajs_; // x y theta dtheta ds
            std::vector<Eigen::VectorXd> cut_Unoccupied_sample_trajs_; //    // x y theta dtheta ds
            
            std::vector<Eigen::Vector2i> small_resolution_path_;

        public:

            FlatTrajData flat_traj_;

            double jps_truncation_time_;

            explicit JPSPlanner(std::shared_ptr<planner::map::Map> map,JPSPlannerParams params);
            
            bool plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

            void get_small_resolution_path_();

            //void pubPath(const std::vector<Eigen::Vector2d> &path, const ros::Publisher &pub);

            std::vector<Eigen::Vector2d> removeCornerPts(const std::vector<Eigen::Vector2d> &path);

            bool checkLineCollision(const Eigen::Vector2d &start, const Eigen::Vector2d &end);

            std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
            
            void getKinoNodeWithStartPath(const std::vector<Eigen::Vector3d> &start_path, const bool if_forward, 
                                          const Eigen::Vector3d &current_state_VAJ, const Eigen::Vector3d &current_state_OAJ);

            void getSampleTraj();

            void getTrajsWithTime();

            void normalizeAngle(const double &ref_angle, double &angle);

            double evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA);
            double evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA);
            double evaluateVel(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA);
            double evaluteTimeOfPos(const double &pos, const double &locallength, const double &startV, const double &endV, const double &maxV, const double &maxA);

            bool JPS_check_if_collision(const Eigen::Vector2d &pos);
    };

};



#endif // JPS_PLANNER_H