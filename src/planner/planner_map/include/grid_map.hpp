#pragma once


#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP
#include <vector>
#include "config.hpp"
#include <unordered_map>
#include<queue>
//Eigen
#include<Eigen/Core>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define logit(x) (log((x) / (1 - (x))))
namespace planner {
    namespace map {
    enum GridState {
        FREE = 0,      // 空闲（可通过）
        OCCUPIED = 100,// 占用（障碍物）
        UNKNOWN = 1   // 未知区域
    };

    class OccupancyGridMap
    {
    public:
        OccupancyGridMap();
        ~OccupancyGridMap(){
            delete[] gridmap_;
            gridmap_ = nullptr;
        };
        void updateOccupancycallback();

    public:
        double x_upper_ = -std::numeric_limits<double>::infinity(),
               y_upper_ = -std::numeric_limits<double>::infinity();
        double x_lower_ = std::numeric_limits<double>::infinity(),
               y_lower_ = std::numeric_limits<double>::infinity();

    private:
        // 存储每个栅格的对数几率值
        std::vector<double> occupancy_map_;
        //TODO： 改为unordered_map设计
        std::vector<short> count_hit_; //击中次数
        std::vector<short> count_hit_and_miss_; //总观测次数
        std::queue<Eigen::Vector2i> cache_voxel_;
        double prob_hit_log_ ; //观测到占用时的更新量
        double prob_miss_log_; //观测到空闲时的更新量
        double clamp_max_log_; //对数几率的最大值（表示非常确定是占用）
        double clamp_min_log_; //对数几率的最小值（表示非常确定是空闲）
        double min_occupancy_log_; //用于判断栅格是否为"空闲"的下限值
    public:
        uint8_t *gridmap_ = nullptr;

    public:
        bool occ_need_update_;
        bool has_map_;

    public:
        OccupancyGridMapConfig map_params;
        double X_SIZE_,Y_SIZE_,XY_SIZE_;
        //X，Y方向栅格数 
        int GLX_SIZE_, GLY_SIZE_,GLXY_SIZE_;
        
        double inv_grid_interval_= 1/map_params.resolution; //地图分辨率的倒数
    private:
        void raycastProcess(
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud,Eigen::Vector3d odom_pos);
        void updateOccupancyMap();
        /** 
            @brief:当机器人周围没有任何观测点 → 无法确定安全区域 → 无法规划路径 时
                在机器人周围生成一个正方形区域的采样点，强制进行射线投射
            @param: 传入odom当前坐标
            TODO: 该函数不会更新地图 逻辑上暂时不启用
        */        
        void cirSupRaycastProcess(Eigen::Vector3d odom_pos);
        /**
        @brief: 移除地图中的离群点（异常值），通过填补孤立未知栅格和确保机器人周围区域已知 
        */
        void RemoveOutliers(Eigen::Vector3d odom_pos);
    private:
        // 从给定位置指向目标点的射线与地图边界的交点，返回地图边界上的最近点
        Eigen::Vector2d closetPointInMap(const Eigen::Vector2d& pt,
            const Eigen::Vector2d& pos);
        bool isInGlobalMap(const Eigen::Vector2d& pt);
        /**   
        @param 传入 pos: 点的世界坐标位置，
               occ:  栅格占用情况  0(空闲) 或 1(占用)
        @return 一维数组索引
        */
        int setCacheOccupancy(Eigen::Vector2i idx, int occ);
        int setCacheOccupancy(Eigen::Vector2d pos, int occ);
    public:
        // 栅格索引->一维数组
        int Index2Vectornum(const int &x, const int &y);
        int Index2Vectornum(const Eigen::Vector2i idx);
        // 世界坐标->栅格索引
        Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d& pt);
        /** 
        @brief:角度归一化，将任意角度转换到 [-π, π] 范围内
        @param: 角度
        @return: 归一化后的角度
        */
        inline double normalize_angle(double angle);
        /**
        @brief: 将栅格索引坐标转换为世界坐标系下的实际物理坐标
        @param: index - 栅格索引坐标（x, y）
        @return: 世界坐标系下的实际物理坐标
        */
        Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index);
        /**
        @brief: 将栅格索引坐标转换为世界坐标系下的实际物理坐标
        @param: x - 栅格索引坐标x
        @param: y - 栅格索引坐标y
        @return: 世界坐标系下的实际物理坐标
        */
        Eigen::Vector2d gridIndex2coordd(const int &x, const int &y);

        bool isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis);
        bool isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis);

        bool isOccupied(const Eigen::Vector2i &index);
        bool isOccupied(const int &idx, const int &idy);
        bool isUnOccupied(const int &idx, const int &idy);
        bool isUnOccupied(const Eigen::Vector2i &index);
        bool isUnknown(const Eigen::Vector2i &index);
        bool isUnknown(const int &idx, const int &idy);
        Eigen::Vector2i vectornum2gridIndex(const int &num);


    private:
        //Bresenham 直线算法 来获取两点之间所有栅格
        std::vector<Eigen::Vector2i> getGridsBetweenPoints2D
        (const Eigen::Vector2i &start, const Eigen::Vector2i &end); 

    
    public:
        // for ESDF
        void updateESDFCallback();
        void updateESDF2d(const Eigen::Vector2d &odom_pos);
        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
        //TODO:
        void publish_ESDF();
        //TODO:
        void publish_ESDFGrad();
        inline double getDistance(const Eigen::Vector2i& id);
        inline double getDistance(const int& idx, const int& idy);
        double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad);
        double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis);
        double getDistWithGradBilinear(const Eigen::Vector2d &pos);

        double getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad);
        /**
        @brief: 获取世界坐标系下某点到最近障碍物的距离
        @param:pos - 世界坐标系下的一个点（x, y）
        @return:该点到最近障碍物的距离
        */
        double getDistanceReal(const Eigen::Vector2d& pos);
        // Check for collisions at half the height of the map
        inline Eigen::Vector2i ESDFcoord2gridIndex(const Eigen::Vector2d &pt);
    private:

    public:
        bool has_esdf_ = false;
        bool esdf_need_update_ = false;
    private:
        std::vector<double> distance_buffer_all_ 
            = std::vector<double>(GLXY_SIZE_, std::numeric_limits<double>::max());
    public:
            Eigen::Vector3d odom_pos;
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud;
    };  



    enum class PointError {

    };
}
}// namespace planner

#endif