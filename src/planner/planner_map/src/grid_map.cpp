#include"../include/grid_map.hpp"
#include "../../../tool/rm_log/include/glog.hpp"
#include <Eigen/src/Core/Matrix.h>
#include<vector>
#include <chrono>
#include"../include/raycast.h"
#include<rclcpp/rclcpp.hpp>
namespace planner::map {
    OccupancyGridMap::OccupancyGridMap(OccupancyGridMapConfig init_map_param){
        // init map
        map_params = init_map_param;

      GLX_SIZE_ = ceil((map_params.global_x_upper - map_params.global_x_lower) / map_params.resolution);
      GLY_SIZE_ = ceil((map_params.global_y_upper - map_params.global_y_lower) / map_params.resolution);
      GLXY_SIZE_ = GLX_SIZE_ * GLY_SIZE_;
      gridmap_ = new uint8_t[GLXY_SIZE_];
      memset(gridmap_, UNKNOWN, GLXY_SIZE_ * sizeof(uint8_t));

      X_SIZE_ = ceil(map_params.detection_range / map_params.resolution) * 2;
      Y_SIZE_ = ceil(map_params.detection_range / map_params.resolution) * 2;
      XY_SIZE_ = X_SIZE_ * Y_SIZE_;
        
      prob_hit_log_ = logit(map_params.p_hit);
      prob_miss_log_ = logit(map_params.p_miss);
      clamp_min_log_ = logit(map_params.p_min);
      clamp_max_log_ = logit(map_params.p_max);
      min_occupancy_log_ = logit(map_params.p_occ);

      double unknown_flag_ = 0.01;
      occupancy_map_ = std::vector<double>(GLXY_SIZE_, clamp_min_log_ - unknown_flag_);

      count_hit_ = std::vector<short>(GLXY_SIZE_, 0);
      count_hit_and_miss_ = std::vector<short>(GLXY_SIZE_, 0);

      odom_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
      point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }
    

    void OccupancyGridMap::updateOccupancycallback()
    {
        if (!occ_need_update_) {
            /**
            TODO: info
            */
            
            LOG(INFO)<<YELLOW << "Current odom position: " << odom_pos.transpose() << RESET ;
            return;
        }
        if (!map_params.if_perspective) {
            //TODO:timer 
            //?? 时间意义何为
            auto timer_start = std::chrono::high_resolution_clock::now();
            x_lower_ = std::max(odom_pos.x() - ceil(map_params.detection_range /
                                                    map_params.resolution) *
                                                   map_params.resolution,
                map_params.global_x_lower);
            x_upper_ = std::min(odom_pos.x() + ceil(map_params.detection_range /
                                                    map_params.resolution) *
                                                   map_params.resolution,
                map_params.global_x_upper);
            y_lower_ = std::max(odom_pos.y() - ceil(map_params.detection_range /
                                                    map_params.resolution) *
                                                   map_params.resolution,
                map_params.global_y_lower);
            y_upper_ = std::min(odom_pos.y() + ceil(map_params.detection_range /
                                                    map_params.resolution) *
                                                   map_params.resolution,
                map_params.global_y_upper);
            X_SIZE_ =
                ceil((x_upper_ - x_lower_) / map_params.resolution);
            Y_SIZE_ =
                ceil((y_upper_ - y_lower_) / map_params.resolution);
            XY_SIZE_ = X_SIZE_ * Y_SIZE_;

            raycastProcess(point_cloud,odom_pos);

            if(map_params.if_cirSupRaycast){
                static int cirSup = 1;
                cirSup ++;
                if(cirSup%3==0){
                    //逻辑上未启用
                    cirSupRaycastProcess(odom_pos);
                    cirSup = 1; 
                }
            }

            RemoveOutliers(odom_pos);
                // Update the map based on occupancy_map_
            Eigen::Vector2i min_id, max_id;
            min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
            max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));
    
            // // Will treat obstacles as free space
            // for (int x=min_id.x(); x<=max_id.x(); x++) {
            //   for (int y=min_id.y(); y<=max_id.y(); y++){
            //     if(occupancy_map_[Index2Vectornum(x,y)] >= clamp_min_log_ && occupancy_map_[Index2Vectornum(x,y)] <= min_occupancy_log_){
            //       gridmap_[Index2Vectornum(x,y)] = Unoccupied;
            //     }
            //     else if(occupancy_map_[Index2Vectornum(x,y)] > min_occupancy_log_){
            //       gridmap_[Index2Vectornum(x,y)] = Occupied;
            //     }
            //   }
            // }
            // Will not treat obstacles as free space!!
            for (int x=min_id.x(); x<=max_id.x(); x++) {
                for (int y=min_id.y(); y<=max_id.y(); y++){
                    int vecIndex = Index2Vectornum(x,y);
                    if(gridmap_[vecIndex] == UNKNOWN && occupancy_map_[vecIndex] >= clamp_min_log_ && occupancy_map_[vecIndex] <= min_occupancy_log_){
                        gridmap_[vecIndex] = FREE;
                    }
                    else if(occupancy_map_[vecIndex] > min_occupancy_log_){
                        gridmap_[vecIndex] = OCCUPIED;
                    }
                }
            }
        }
        else {
            /**
            TODO:透视模式
            */
            x_lower_ = std::max(odom_pos.x() - ceil(map_params.detection_range / map_params.resolution)*map_params.resolution, map_params.global_x_lower);
            x_upper_ = std::min(odom_pos.x() + ceil(map_params.detection_range / map_params.resolution)*map_params.resolution, map_params.global_x_upper);
            y_lower_ = std::max(odom_pos.y() - ceil(map_params.detection_range / map_params.resolution)*map_params.resolution, map_params.global_y_lower);
            y_upper_ = std::min(odom_pos.y() + ceil(map_params.detection_range / map_params.resolution)*map_params.resolution, map_params.global_y_upper);

            X_SIZE_ = ceil((x_upper_ - x_lower_) / map_params.resolution);
            Y_SIZE_ = ceil((y_upper_ - y_lower_) / map_params.resolution);
            XY_SIZE_ = X_SIZE_ * Y_SIZE_;

            Eigen::Vector2i min_id, max_id;
            min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
            max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

            for (int x=min_id.x(); x<=max_id.x(); x++) {
                for (int y=min_id.y(); y<=max_id.y(); y++){
                    if(gridmap_[Index2Vectornum(x,y)] == UNKNOWN){
                        gridmap_[Index2Vectornum(x,y)] = FREE;
                    }
                }
            }
            if (!point_cloud) {
                LOG(WARNING)<<YELLOW << "Point cloud is null, skipping updateOccupancycallback 透视模式"<<RESET ;
                return ;
            }

            for(auto point:point_cloud->points){
                Eigen::Vector2d coord = Eigen::Vector2d(point.x, point.y);
                if(!isInGlobalMap(coord)){
                    continue;
                }
                Eigen::Vector2i idx = coord2gridIndex(coord);
                gridmap_[Index2Vectornum(idx)] = OCCUPIED;
            }
        }
        has_map_ = true;
        occ_need_update_ = false;
    }

    void OccupancyGridMap::raycastProcess(
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud,
        Eigen::Vector3d odom_pos)
    {
        if (!point_cloud) {
            LOG(WARNING)<<YELLOW << "Point cloud is null, skipping raycast process."<<RESET ;
            return ;
        }
        Eigen::Vector2d cur_point;//当前点
        Eigen::Vector2d odom_pos_xy = odom_pos.head(2); // 机器人当前位置
        int vox_idx; 
        for (auto point: point_cloud->points) {
            cur_point << point.x, point.y; 
            
            // 检查点是否在地图范围内
            if (!isInGlobalMap(cur_point)) {
                //不在 ------>  投影到地图边界
                // 第一次截断：地图边界截断 确保点在地图范围内
                cur_point = closetPointInMap(cur_point, odom_pos_xy);
                double length = (cur_point - odom_pos_xy).norm();
                //如果距离超过检测范围，再次截断 限制在传感器可信范围内
                if (length > map_params.detection_range) {
                    cur_point = (cur_point - odom_pos_xy) / length * // 单位方向向量
                                    map_params.detection_range + //乘以最大距离
                                odom_pos_xy;    // + 机器人当前位置
                }
            vox_idx = setCacheOccupancy(cur_point, 0);
            }else {
                // 点在地图内
                double length = (cur_point - odom_pos_xy).norm();
                //如果距离超过检测范围，截断 限制在传感器可信范围内
                if (length > map_params.detection_range) {
                    cur_point = (cur_point - odom_pos_xy) / length * map_params.detection_range + odom_pos_xy;
                    vox_idx = setCacheOccupancy(cur_point, 0);
                } else {
                    vox_idx = setCacheOccupancy(cur_point, 1);
                } 
            }
            std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(coord2gridIndex(odom_pos_xy), coord2gridIndex(cur_point));
    
            for (int i=0; i<line.size() - 1; i++) {
                vox_idx = setCacheOccupancy(line[i], 0);
            }
        }
        updateOccupancyMap();
    };


void OccupancyGridMap::cirSupRaycastProcess(Eigen::Vector3d odom_pos){
    Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);
    double length;

    int vox_idx;
    
    Eigen::Vector2d odom_pos_xy=odom_pos.head(2);
    
    //采样点
    std::vector<Eigen::Vector2d> cir_points;
    // 生成上下两条边上的点
    for(double x = odom_pos.x() - map_params.detection_range; x < odom_pos.x() + map_params.detection_range+1e-10;  x += 2*map_params.detection_range){
        for(double y = odom_pos.y() - map_params.detection_range; y < odom_pos.y() + map_params.detection_range+1e-10;  y += map_params.resolution){
        cir_points.emplace_back(x,y);
        }
    }
    // 生成左右两条边上的点
    for(double y = odom_pos.y() - map_params.detection_range; y < odom_pos.y() + map_params.detection_range+1e-10;  y += 2*map_params.detection_range){
        for(double x = odom_pos.x() - map_params.detection_range; x < odom_pos.x() + map_params.detection_range+1e-10;  x += map_params.resolution){
        cir_points.emplace_back(x,y);
        }
    }

    RayCaster raycaster;

    //处理思路与raycastProcess一样 多了一个FOV角度检查
    for(auto cir_point:cir_points){
        //FOV角度检查
        if(map_params.hrz_limited){
            // 计算点相对于机器人的角度
            double angle = atan2(cir_point.y() - odom_pos.y(), cir_point.x() - odom_pos.x());
            // 转换为相对于机器人朝向的角度
            angle = normalize_angle(angle - odom_pos.z());
            // 检查是否在水平视场角范围内
            if(angle < -map_params.hrz_laser_range_dgr/2.2 || angle >map_params.hrz_laser_range_dgr/2.2){
                continue;  // 超出FOV，跳过这个点
            }
        }
        // 如果点不在全局地图内，投影到地图边界
        if(!isInGlobalMap(cir_point)){
            cir_point = closetPointInMap(cir_point, odom_pos_xy);
        }
        // 计算到机器人的距离
        length = (cir_point - odom_pos_xy).norm();
        // 如果超出检测范围，截断到最大距离
        if(length > map_params.detection_range){
            cir_point = (cir_point - odom_pos_xy) / length * map_params.detection_range + odom_pos_xy;
        }

        std::vector<Eigen::Vector2i> line;
        // 设置射线从机器人到采样点
        raycaster.setInput(
            Eigen::Vector3d(cir_point.x(), cir_point.y(), 0.1) / map_params.resolution, 
            Eigen::Vector3d(odom_pos_xy.x(), odom_pos_xy.y(), 0.1) / map_params.resolution);
        
        //障碍物检查
        bool occ = false;

        Eigen::Vector3d ray_pt;
        
        while (raycaster.step(ray_pt)) {
            Eigen::Vector2d tmp = (ray_pt.head(2) + half) * map_params.resolution;
            Eigen::Vector2i tmp_idx = coord2gridIndex(tmp);
            line.emplace_back(tmp_idx);
            // 检查当前点及其4邻域是否有障碍物
            if(gridmap_[Index2Vectornum(tmp_idx)] == OCCUPIED || 
                (tmp_idx.y() < GLY_SIZE_ && gridmap_[Index2Vectornum(tmp_idx)+1] == OCCUPIED) || 
                (tmp_idx.y() > 0 && gridmap_[Index2Vectornum(tmp_idx)-1] == OCCUPIED) || 
                (tmp_idx.x() < GLX_SIZE_ && gridmap_[Index2Vectornum(tmp_idx)+GLY_SIZE_] == OCCUPIED) || 
                (tmp_idx.x() > 0 && gridmap_[Index2Vectornum(tmp_idx)-GLY_SIZE_] == OCCUPIED)){
                occ = true;
                break;
            }
            }
            // 有障碍物，跳过这条射线
            if(occ) {
                continue;
            }
            // 如果没有障碍物，将射线上的所有栅格（除终点）标记为空闲
            int size = line.size()-1;
            for (int i=0; i<size; i++) {
                vox_idx = setCacheOccupancy(line[i], 0);
            }
    }

    // updateOccupancyMap();

    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    while (!cache_voxel_.empty()) {
        Eigen::Vector2i idx = cache_voxel_.front();
        int idx_ctns = Index2Vectornum(idx);
        cache_voxel_.pop();

        double log_odds_update =
        count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 4*count_hit_[idx_ctns] ?
        prob_hit_log_ :
        prob_miss_log_;
        //########################//
        //######## ？？？？########//
        //########################//
        //导致该函数并未发挥作用，地图未更新
        log_odds_update = 0.0;

        count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

        if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
            continue;
        } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
            occupancy_map_[idx_ctns] = clamp_min_log_;
            continue;
        }

        bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
        if (!in_local) {
            occupancy_map_[idx_ctns] = clamp_min_log_;
        }

        occupancy_map_[idx_ctns] =
            std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                    clamp_max_log_);
    }

    }
    void OccupancyGridMap::updateOccupancyMap(){
        Eigen::Vector2i min_id, max_id;
        min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
        max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

        while (!cache_voxel_.empty()) {
            Eigen::Vector2i idx = cache_voxel_.front();
            int idx_ctns = Index2Vectornum(idx);
            cache_voxel_.pop();
        
            //如果击中次数超过总观测次数的1/4，就认为是趋向占用的，增加占用概率
            //log-odds = log(p / (1-p)) 
            double log_odds_update =
            count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 3*count_hit_[idx_ctns] ?
            prob_hit_log_ :prob_miss_log_;
            
            //重置计数器
            count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;
            
            //防止过度占用/空闲   log_odds_update (p=0.5 )=0  >0 --> 占用  <0 --> 空闲
            if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
                continue;
            } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
                occupancy_map_[idx_ctns] = clamp_min_log_;
                continue;
            }
            //检查栅格是否在局部地图范围内
            bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
            //如果不在，将其设为最小值（表示未知/空闲）
            if (!in_local) {
                occupancy_map_[idx_ctns] = clamp_min_log_;
            }
            
            //更新  clamp_min_log_<ocupancy_map_<clamp_max_log_
            occupancy_map_[idx_ctns] =
                std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                        clamp_max_log_);
        }
    }

    void OccupancyGridMap::RemoveOutliers(Eigen::Vector3d odom_pos){
        std::vector<Eigen::Vector2d> cir_points;
        // 在机器人周围 detection_range 的正方形区域内生成密集采样点
        for(double x = odom_pos.x() - map_params.detection_range; x < odom_pos.x() + map_params.detection_range+1e-10;  x += map_params.resolution){
            for(double y = odom_pos.y() - map_params.detection_range; y < odom_pos.y() + map_params.detection_range+1e-10;  y += map_params.resolution){
            cir_points.emplace_back(x,y);
            }
        }
        // 留出一个栅格的边界，避免检查边界外的点
        double xlow = map_params.global_x_lower + map_params.resolution;
        double xup = map_params.global_x_upper - map_params.resolution;
        double ylow = map_params.global_y_lower + map_params.resolution;
        double yup = map_params.global_y_upper - map_params.resolution;

        // 孤立点检测与修复
        for(auto cir_point:cir_points){
            // 确保点在内部区域
            if(cir_point.x() > xlow && cir_point.x() < xup && cir_point.y() > ylow && cir_point.y() < yup){
                // 如果当前栅格是未知
                if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] == UNKNOWN){
                    // 检查4个邻域是否都是空闲
                    /**情况1：孤立未知点（应该修复）
                            ┌─────┬─────┬─────┐
                            │ FREE│ FREE│ FREE│
                            ├─────┼─────┼─────┤
                            │ FREE│ UNK │ FREE│ ← 被4个FREE包围
                            ├─────┼─────┼─────┤
                            │ FREE│ FREE│ FREE│
                            └─────┴─────┴─────┘
                        情况2：真正的未知区域（保留）
                            ┌─────┬─────┬─────┐
                            │ FREE│ OCC │ FREE│
                            ├─────┼─────┼─────┤
                            │ FREE│ UNK │ FREE│ ← 有OCC邻居
                            ├─────┼─────┼─────┤
                            │ FREE│ FREE│ FREE│
                            └─────┴─────┴─────┘
                        情况3：边界未知（不处理）
                            ┌─────┬─────┬─────┐
                            │ UNK │ UNK │ UNK │
                            ├─────┼─────┼─────┤
                            │ UNK │ UNK │ UNK │ ← 边界区域
                            ├─────┼─────┼─────┤
                            │ UNK │ UNK │ UNK │
                            └─────┴─────┴─────┘ */
                    if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point))+1] == FREE && 
                    gridmap_[Index2Vectornum(coord2gridIndex(cir_point))-1] == FREE && 
                    gridmap_[Index2Vectornum(coord2gridIndex(cir_point))+GLY_SIZE_] == FREE && 
                    gridmap_[Index2Vectornum(coord2gridIndex(cir_point))-GLY_SIZE_] == FREE){
                        // 将当前未知栅格设为空闲
                        gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] = FREE;
                    }
                }
            }
            
        }
        
        Eigen::Vector2i idx = coord2gridIndex(Eigen::Vector2d(odom_pos.x(), odom_pos.y()));
        // 检查机器人周围3x3区域    
        //避免机器人因未知区域而无法移动
        for(int i=-1; i<=1; i++){
            for(int j=-1; j<=1; j++){
                if(gridmap_[Index2Vectornum(idx.x()+i, idx.y()+j)] == UNKNOWN){
                    gridmap_[Index2Vectornum(idx.x()+i, idx.y()+j)] = FREE;
                }
            }
        }

    }


    std::vector<Eigen::Vector2i> OccupancyGridMap::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end){
        //Bresenham 直线算法 来获取两点之间所有栅格

        std::vector<Eigen::Vector2i> line;
        
        int dx = abs(end.x() - start.x());
        int dy = abs(end.y() - start.y());
        
        int sx = (start.x() < end.x()) ? 1 : -1; // X方向：向右+1，向左-1
        int sy = (start.y() < end.y()) ? 1 : -1; // Y方向：向上+1，向下-1
        
        //err 表示当前点与理想直线的偏离程度
        //- err > 0: 偏向X方向
        //- err < 0: 偏向Y方向
        //- err = 0: 正好在对角线上
        int err = dx - dy; // 决定下一步是走X还是Y

        double x0 = start.x();
        double y0 = start.y();

        while (true) {
            line.emplace_back(x0, y0);
            //TODO:double = 待优化
            if (x0 == end.x() && y0 == end.y()) { // 到达终点则退出
                break;
            }

            int e2 = 2 * err;
            // 决策逻辑
            if (e2 > -dy) {    // 决定在X方向上前进
                err -= dy;      // 更新误差
                x0 += sx;       // X坐标增加
            }
            if (e2 < dx) {     // 决定在Y方向上前进
                err += dx;      // 更新误差
                y0 += sy;       // Y坐标增加
            }
        }

        return line;
    }
    /*  
        @param 传入 pos: 点的世界坐标位置，
               occ:  栅格占用情况  0(空闲) 或 1(占用)
        @return 一维数组索引
    
    */
    int OccupancyGridMap::setCacheOccupancy(Eigen::Vector2d pos, int occ) {
        // 只接受 0(空闲) 或 1(占用)
        if (occ != 1 && occ != 0) {
            //rmlog::warn("occ is not 1 / 0");
            LOG(WARNING) << YELLOW << "Invalid occupancy value: " << occ << RESET;
            return -1;
        }
        // 世界坐标 → 栅格索引
        Eigen::Vector2i idx = coord2gridIndex(pos);
        // 栅格索引 → 一维数组索引
        int idx_ctns = Index2Vectornum(idx);

        // 无论空闲/占用，都增加总观测次数
        count_hit_and_miss_[idx_ctns] += 1;

        // 第一次观测到这个栅格，加入缓存队列
        if (count_hit_and_miss_[idx_ctns] == 1) {
            cache_voxel_.push(idx);
        }

        // 如果是占用，增加占用计数
        if (occ == 1) {
            count_hit_[idx_ctns] += 1;
        } 
        return idx_ctns;
    }

    int OccupancyGridMap::setCacheOccupancy(Eigen::Vector2i idx, int occ) {
        if (occ != 1 && occ != 0) return -1;

        int idx_ctns = Index2Vectornum(idx);

        count_hit_and_miss_[idx_ctns] += 1;

        if (count_hit_and_miss_[idx_ctns] == 1) {
            cache_voxel_.push(idx);
        }

        if (occ == 1) count_hit_[idx_ctns] += 1;

        return idx_ctns;
        }
    /*
    @brief: 给定一个点 pt（可能在地图外）和机器人位置 pos，找到从机器人指向该点的射线与地图边界的交点，
            并返回一个稍微靠内一点的点。
    @param: pt 可能在地图外
            pos 机器人位置
    @return 机器人指向该点的射线与地图边界的交点(稍微靠内一点),
        P(t) = P₀ + t · v,  where t ≥ 0 
        x(t) = x₀ + t · (x₁ - x₀)
        y(t) = y₀ + t · (y₁ - y₀) 
        x_min ≤ x(t) ≤ x_max
        y_min ≤ y(t) ≤ y_max
    */
    Eigen::Vector2d OccupancyGridMap::closetPointInMap(
        const Eigen::Vector2d& pt,
        const Eigen::Vector2d& pos)
    {
        Eigen::Vector2d diff = pt - pos;// 方向向量
        Eigen::Vector2d max_tc = Eigen::Vector2d(map_params.global_x_upper,
                                     map_params.global_y_upper) -
                                 pos;// 到最大边界的向量
        Eigen::Vector2d min_tc = Eigen::Vector2d(map_params.global_x_lower,
                                     map_params.global_y_lower) -
                                 pos;// 到最小边界的向量

        double min_t = std::numeric_limits<int>::max();

        for (int i = 0; i < 2; ++i) {
            if (fabs(diff[i]) > 0)//diff不为0
            {
                // 计算与最大边界的交点参数t
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t) {
                    min_t = t1;
                }
                // 计算与最小边界的交点参数t
                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t) {
                    min_t = t2;
                }
            }
        }
        // 减去小量确保点在地图内部
        return pos + (min_t - 1e-3) * diff;
    }
    //将世界坐标系中的点转换为栅格索引
    Eigen::Vector2i OccupancyGridMap::coord2gridIndex(const Eigen::Vector2d& pt)
    {
        Eigen::Vector2i idx;
        //int((pt(0) - map_params.global_x_lower) * inv_grid_interval_) 计算原始珊格索引
        idx << std::min(
            std::max(int((pt(0) - map_params.global_x_lower) * inv_grid_interval_), 0),
            GLX_SIZE_ - 1), 
            std::min(
                std::max(int((pt(1) - map_params.global_y_lower) * inv_grid_interval_),
                    0),
                GLY_SIZE_ - 1);
        return idx;
    }
    bool OccupancyGridMap::isInGlobalMap(const Eigen::Vector2d& pt)
    {
        if (pt.hasNaN() || !pt.allFinite()) {
            //rmlog::info("pt has nan");
            LOG(INFO) << YELLOW << "Point has NaN or infinite values: " << pt.transpose() << RESET;
            return false;
        }
        return pt.x() < map_params.global_x_upper &&
               pt.x() > map_params.global_x_lower &&
               pt.y() < map_params.global_y_upper &&
               pt.y() > map_params.global_y_lower;
    }
    
    int OccupancyGridMap::Index2Vectornum(const int &x, const int &y){
        return x * GLY_SIZE_ + y;
    }
    
    int OccupancyGridMap::Index2Vectornum(const Eigen::Vector2i idx){
        return idx.x() * GLY_SIZE_ + idx.y();
    }

    inline double OccupancyGridMap::normalize_angle(double angle){
        if(angle>M_PI){ 
            angle -= 2*M_PI;
        }
        if(angle<-M_PI){ 
            angle += 2*M_PI;
        }
        return angle;
    }

    Eigen::Vector2d OccupancyGridMap::gridIndex2coordd(const Eigen::Vector2i &index){
        Eigen::Vector2d pt;
        pt(0) = ((double)index(0) + 0.5) * map_params.resolution + map_params.global_x_lower;
        pt(1) = ((double)index(1) + 0.5) * map_params.resolution + map_params.global_y_lower;
        return pt;
    }

    Eigen::Vector2d OccupancyGridMap::gridIndex2coordd(const int &x, const int &y){
        Eigen::Vector2d pt;
        pt(0) = ((double)x + 0.5) * map_params.resolution + map_params.global_x_lower;
        pt(1) = ((double)y + 0.5) * map_params.resolution + map_params.global_y_lower;
        return pt;
    }

    //esdf_map

    void OccupancyGridMap::updateESDFCallback(){
        if(!esdf_need_update_) {
            LOG(INFO) << YELLOW << "ESDF is up to date, skipping update." << RESET;
            return;
        }
        Eigen::Vector2d odom_pos_xy=odom_pos.head(2);
        updateESDF2d(odom_pos_xy);

        has_esdf_ = true;
        esdf_need_update_ = false;
        
    }

    void OccupancyGridMap::updateESDF2d(const Eigen::Vector2d & odom_pos){
        Eigen::Vector2i min_esdf(floor(std::max(0.0, odom_pos.x() - map_params.detection_range - map_params.global_x_lower)*inv_grid_interval_), 
        floor(std::max(0.0, odom_pos.y() - map_params.detection_range - map_params.global_y_lower)*inv_grid_interval_));
        Eigen::Vector2i max_esdf(ceil(std::min(map_params.global_x_upper - map_params.global_x_lower, odom_pos.x() + map_params.detection_range - map_params.global_x_lower)*inv_grid_interval_) - 1,
                                ceil(std::min(map_params.global_y_upper - map_params.global_y_lower, odom_pos.y() + map_params.detection_range - map_params.global_y_lower)*inv_grid_interval_) - 1);

        int update_X_SIZE = max_esdf.x() - min_esdf.x();
        int update_Y_SIZE = max_esdf.y() - min_esdf.y();
        int update_XY_SIZE = (update_X_SIZE + 1) * (update_Y_SIZE + 1);

        std::vector<double> tmp_buffer1_ = std::vector<double>(update_XY_SIZE, 0.0);
        std::vector<double> distance_buffer_ = std::vector<double>(update_XY_SIZE, 0.0);
        std::vector<double> distance_buffer_neg_ = std::vector<double>(update_XY_SIZE, 0.0);
        // distance_buffer_all_ = std::vector<double>(GLXY_SIZE, 0.0);
        /* ========== compute positive DT (distance transform outside the obstacles) ========== */
        for (int x = 0; x <= update_X_SIZE; x++) {
            fillESDF(
                [&](int y) {
                return gridmap_[(x+min_esdf.x()) * GLY_SIZE_ + (y+min_esdf.y())] == GridState::OCCUPIED ?
                    0.0 :
                    std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer1_[x * update_Y_SIZE + y] = val; }, 0,
                update_Y_SIZE, update_Y_SIZE+1);
        }
        for (int y = 0; y <= update_Y_SIZE; y++) {
            fillESDF(
            [&](int x) { 
                return tmp_buffer1_[x * update_Y_SIZE + y]; },
                    [&](int x, double val) {
                        distance_buffer_[x * update_Y_SIZE + y] = map_params.resolution * std::sqrt(val);
                    },
                    0, update_X_SIZE, update_X_SIZE+1);
        }
        /* ========== compute negative distance inside the obstacles ========== */
        for (int x = 0; x <= update_X_SIZE; x++) {
            fillESDF(
                [&](int y) {
                int state = gridmap_[(x+min_esdf.x()) * GLY_SIZE_ + (y+min_esdf.y())];
                return (state == GridState::FREE || state == GridState::UNKNOWN) ?
                    0.0 :
                    std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer1_[x * update_Y_SIZE + y] = val; }, 0,
                update_Y_SIZE, update_Y_SIZE+1);
        }
        for (int y = 0; y <= update_Y_SIZE; y++) {
            fillESDF([&](int x) { return tmp_buffer1_[x * update_Y_SIZE + y]; },
                    [&](int x, double val) {
                        distance_buffer_neg_[x * update_Y_SIZE + y] = map_params.resolution * std::sqrt(val);
                    },
                    0, update_X_SIZE, update_X_SIZE+1);
        }
        /* ========== combine pos and neg DT ========== */
        for (int x = 0; x < update_X_SIZE; x++)
            for (int y = 0; y < update_Y_SIZE; y++){
                int global_idx = (x + min_esdf.x()) * GLY_SIZE_ + y + min_esdf.y();
                int idx =  x * update_Y_SIZE + y;
                distance_buffer_all_[global_idx] = distance_buffer_[idx];

                if (distance_buffer_neg_[idx] > 0.0)
                distance_buffer_all_[global_idx] += (-distance_buffer_neg_[idx] + map_params.resolution);
            }
        }
    
    template <typename F_get_val, typename F_set_val>
    void OccupancyGridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size) {
            int v[dim_size];
            double z[dim_size + 1];

            int k = start;
            v[start] = start;
            z[start] = -std::numeric_limits<double>::max();
            z[start + 1] = std::numeric_limits<double>::max();

            for (int q = start + 1; q <= end; q++) {
                k++;
                double s;

                do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
                } while (s <= z[k]);

                k++;

                v[k] = q;
                z[k] = s;
                z[k + 1] = std::numeric_limits<double>::max();
            }

            k = start;

            for (int q = start; q <= end; q++) {
                while (z[k + 1] < q) k++;
                double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
                f_set_val(q, val);
            }
    }

    inline double OccupancyGridMap::getDistance(const Eigen::Vector2i& id){
        // if(distance_buffer_all_[Index2Vectornum(id.x(), id.y())]>5){
        //   ROS_ERROR("out of map!!  %d  %d   distance:%f",id.x(),id.y(),distance_buffer_all_[Index2Vectornum(id[0],id[1])]);
        // }
        return distance_buffer_all_[Index2Vectornum(id[0],id[1])];
        }

    inline double OccupancyGridMap::getDistance(const int& idx, const int& idy){
    // if(distance_buffer_all_[Index2Vectornum(idx, idy)]>5){
    //   ROS_ERROR("out of map!!  %d  %d  distance:%f",idx,idy,distance_buffer_all_[Index2Vectornum(idx, idy)]);
    // }
        return distance_buffer_all_[Index2Vectornum(idx, idy)];
    }

    inline Eigen::Vector2i OccupancyGridMap::ESDFcoord2gridIndex(const Eigen::Vector2d &pt){
        Eigen::Vector2i idx;
        idx << std::min(std::max(int((pt(0) - map_params.global_x_lower) * inv_grid_interval_ - 0.5), 0), GLX_SIZE_ - 1),
                std::min(std::max(int((pt(1) - map_params.global_y_lower) * inv_grid_interval_ - 0.5), 0), GLY_SIZE_ - 1);
        return idx;
    }

    double OccupancyGridMap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
        if(pos.x()<map_params.global_x_lower||pos.y()<map_params.global_y_lower||pos.x()>map_params.global_x_upper||pos.y()>map_params.global_y_upper){
            grad.setZero();
            // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
            return 100;
        }
        Eigen::Vector2d pos_m = pos;
        Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
        if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
            grad.setZero();
            // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
            return 100;
        }

        Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
        Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


        double values[2][2];
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            values[x][y] = getDistance(current_idx);
            }
        }

        double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
        double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
        double dist = (1 - diff[1]) * v0 + diff[1] * v1;

        grad[1] = (v1 - v0) * inv_grid_interval_;
        grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

        return dist;
    }

    double OccupancyGridMap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double& mindis){
        if(pos.x()<map_params.global_x_lower||pos.y()<map_params.global_y_lower||pos.x()>map_params.global_x_upper||pos.y()>map_params.global_y_upper){
            grad.setZero();
            // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
            return 1e10;
        }
        Eigen::Vector2d pos_m = pos;
        Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
        if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
            grad.setZero();
            // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
            return 1e10;
        }

        Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
        Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


        double values[2][2];
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            values[x][y] = getDistance(current_idx);
            }
        }

        double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
        double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
        double dist = (1 - diff[1]) * v0 + diff[1] * v1;

        if(dist > mindis){
            return dist;
        }

        grad[1] = (v1 - v0) * inv_grid_interval_;
        grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

        return dist;
    }

    double OccupancyGridMap::getDistWithGradBilinear(const Eigen::Vector2d &pos){
        if(pos.x()<map_params.global_x_lower||pos.y()<map_params.global_y_lower||pos.x()>map_params.global_x_upper||pos.y()>map_params.global_y_upper){
            return 1e10;
        }
        Eigen::Vector2d pos_m = pos;
        Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
        if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
            return 1e10;
        }

        Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
        Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


        double values[2][2];
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            values[x][y] = getDistance(current_idx);
            }
        }

        double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
        double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
        double dist = (1 - diff[1]) * v0 + diff[1] * v1;

        return dist; 
    }

    double OccupancyGridMap::getDistanceReal(const Eigen::Vector2d& pos){
        if(pos.x()<map_params.global_x_lower||pos.y()<map_params.global_y_lower||pos.x()>map_params.global_x_upper||pos.y()>map_params.global_y_upper){
            //TODO: 更改返回值，这里是希望返回一个很大的距离，表示离障碍物很远
            return 10000;
        }
        Eigen::Vector2i idx = coord2gridIndex(pos);
        return distance_buffer_all_[idx.x() * GLY_SIZE_ + idx.y()];
    }

    double OccupancyGridMap::getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
        if(pos.x()<map_params.global_x_lower||pos.y()<map_params.global_y_lower||pos.x()>map_params.global_x_upper||pos.y()>map_params.global_y_upper){
            grad.setZero();
            // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
            return 100;
        }
        Eigen::Vector2d pos_m = pos;
        Eigen::Vector2i idx = coord2gridIndex(pos_m);
        if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
            grad.setZero();
            // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
            return 100;
        }

        Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
        Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

        double values[2][2];
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            values[x][y] = gridmap_[Index2Vectornum(current_idx)]==GridState::UNKNOWN?1:0;
            }
        }

        double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
        double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
        double dist = (1 - diff[1]) * v0 + diff[1] * v1;

        grad[1] = (v1 - v0) * inv_grid_interval_;
        grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

        return dist;
    }

    bool OccupancyGridMap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis){
        return distance_buffer_all_[Index2Vectornum(index)] < safe_dis;
    }

    bool OccupancyGridMap::isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis){
        return distance_buffer_all_[Index2Vectornum(idx, idy)] < safe_dis;
    }
    bool OccupancyGridMap::isOccupied(const Eigen::Vector2i &index){
        return gridmap_[Index2Vectornum(index)] == GridState::OCCUPIED;
    }

    bool OccupancyGridMap::isOccupied(const int &idx, const int &idy){
        return gridmap_[Index2Vectornum(idx, idy)] == GridState::OCCUPIED;
    }

    bool OccupancyGridMap::isUnOccupied(const int &idx, const int &idy){
        return gridmap_[Index2Vectornum(idx, idy)] == GridState::FREE;
    }

    bool OccupancyGridMap::isUnOccupied(const Eigen::Vector2i &index){
        return gridmap_[Index2Vectornum(index)] == GridState::FREE;
    }

    bool OccupancyGridMap::isUnknown(const Eigen::Vector2i &index){
        return gridmap_[Index2Vectornum(index)] == GridState::UNKNOWN;
    }

    bool OccupancyGridMap::isUnknown(const int &idx, const int &idy){
        return gridmap_[Index2Vectornum(idx, idy)] == GridState::UNKNOWN;
    }
    Eigen::Vector2i OccupancyGridMap::vectornum2gridIndex(const int &num){
        Eigen::Vector2i index;
        index(0) = num / GLY_SIZE_;
        index(1) = num % GLY_SIZE_;
        return index;
    }

}// namespace planner::map