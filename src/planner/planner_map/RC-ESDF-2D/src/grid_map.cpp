#include "grid_map.hpp"
#include <chrono>
namespace planner {
    void OccupancyGridMap::updateOccupancy(Eigen::Vector3d odom_pos)
    {
        if (!occ_need_update_) {
            return;
        }
        if (!map_params.if_perspective) {
            //TODO:timer
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
        }
        else {
        }
    }
    void OccupancyGridMap::raycastProcess(
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud,
        Eigen::Vector3d odom_pos)
    {
        Eigen::Vector2d cur_point;
        Eigen::Vector2d odom_pos_xy = odom_pos.head(2);
        for (auto point: point_cloud->points) {
            cur_point << point.x, point.y;
            if (!isInGlobalMap(cur_point)) {
                cur_point = closetPointInMap(cur_point, odom_pos_xy);
                double length = (cur_point - odom_pos_xy).norm();
                if (length > map_params.detection_range) {
                    cur_point = (cur_point - odom_pos_xy) / length *
                                    map_params.detection_range +
                                odom_pos_xy;
                }
            }
        }
    };
    /*
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
    Eigen::Vector2i OccupancyGridMap::coord2gridIndex(const Eigen::Vector2d& pt)
    {
        Eigen::Vector2i idx;
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
            std::printf("pt has nan\n");
            return false;
        }
        return pt.x() < map_params.global_x_upper &&
               pt.x() > map_params.global_x_lower &&
               pt.y() < map_params.global_y_upper &&
               pt.y() > map_params.global_y_lower;
    }
}// namespace planner