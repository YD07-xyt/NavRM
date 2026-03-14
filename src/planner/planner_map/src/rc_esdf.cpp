#include "../include/rc_esdf.h"
#include <limits>
namespace rc_esdf {


    void
    RcEsdfMap::initialize(double width_m, double height_m, double resolution)
    {
        width_m_ = width_m;
        height_m_ = height_m;
        resolution_ = resolution;

        grid_size_x_ = std::ceil(width_m_ / resolution_);
        grid_size_y_ = std::ceil(height_m_ / resolution_);

        // 假设机器人中心在地图中心
        origin_x_ = -width_m_ / 2.0;
        origin_y_ = -height_m_ / 2.0;

        data_.resize(grid_size_x_ * grid_size_y_, 0.0f);
    }

  

    void RcEsdfMap::generateFromPolygon(
        const std::vector<Eigen::Vector2d>& polygon)
    {
        // 遍历每一个格子
        for (int y = 0; y < grid_size_y_; ++y) {
            for (int x = 0; x < grid_size_x_; ++x) {
                // 计算格子中心的物理坐标
                double px = origin_x_ + (x + 0.5) * resolution_;
                double py = origin_y_ + (y + 0.5) * resolution_;
                Eigen::Vector2d p(px, py);

                // 1. 计算到多边形轮廓的最小距离
                double min_dist_sq = std::numeric_limits<double>::max();
                for (size_t i = 0; i < polygon.size(); ++i) {
                    Eigen::Vector2d v1 = polygon[i];
                    Eigen::Vector2d v2 = polygon[(i + 1) % polygon.size()];
                    double d_sq = pointToSegmentDistSq(p, v1, v2);
                    if (d_sq < min_dist_sq) {
                        min_dist_sq = d_sq;
                    }
                }
                double min_dist = std::sqrt(min_dist_sq);

                // 2. 判断内部还是外部
                if (isPointInPolygon(p, polygon)) {
                    // 内部：存负距离
                    data_[y * grid_size_x_ + x] = -min_dist;
                }
                else {
                    // 外部：存 0 (根据论文设定)
                    // 如果你想做安全余量，这里可以存 min_dist
                    data_[y * grid_size_x_ + x] = min_dist;//0.0f;
                }
            }
        }
        std::cout << "[RC-ESDF] Map Generated. Size: " << grid_size_x_ << "x"
                  << grid_size_y_ << std::endl;
    }

    bool RcEsdfMap::query(const Eigen::Vector2d& pos_body,
        double& dist,
        Eigen::Vector2d& grad) const
    {
        double gx, gy;
        posToGrid(pos_body, gx, gy);

        // 将坐标偏移 0.5，使得格子的中心点对应整数索引
        double u = gx - 0.5;
        double v = gy - 0.5;

        // 使用 clamp 保证索引在安全范围内，防止边缘处直接跳变到 0
        // 允许插值的范围是 [0, size-1]
        if (u < 0 || u >= grid_size_x_ - 1 || v < 0 || v >= grid_size_y_ - 1) {
            // 如果完全超出物理边界，则返回 false 或 边界值
            dist = 0.0;
            grad.setZero();
            return false;
        }

        int x0 = std::floor(u);
        int y0 = std::floor(v);
        double alpha = u - x0;
        double beta = v - y0;

        float v00 = getRaw(x0, y0);
        float v10 = getRaw(x0 + 1, y0);
        float v01 = getRaw(x0, y0 + 1);
        float v11 = getRaw(x0 + 1, y0 + 1);

        // 双线性插值
        dist = (1 - alpha) * (1 - beta) * v00 + alpha * (1 - beta) * v10 +
               (1 - alpha) * beta * v01 + alpha * beta * v11;

        // 梯度计算 (数值导数)
        double d_alpha = (1 - beta) * (v10 - v00) + beta * (v11 - v01);
        double d_beta = (1 - alpha) * (v01 - v00) + alpha * (v11 - v10);

        grad.x() = d_alpha / resolution_;
        grad.y() = d_beta / resolution_;

        return true;
    }

    // ----- 数学辅助函数 -----

    double RcEsdfMap::pointToSegmentDistSq(const Eigen::Vector2d& p,
        const Eigen::Vector2d& v,
        const Eigen::Vector2d& w)
    {
        double l2 = (v - w).squaredNorm();
        if (l2 == 0.0) {
            return (p - v).squaredNorm();
        }
        double t = ((p - v).dot(w - v)) / l2;
        t = std::max(0.0, std::min(1.0, t));
        Eigen::Vector2d projection = v + t * (w - v);
        return (p - projection).squaredNorm();
    }

    bool RcEsdfMap::isPointInPolygon(const Eigen::Vector2d& p,
        const std::vector<Eigen::Vector2d>& poly)
    {
        bool inside = false;
        for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
            if (((poly[i].y() > p.y()) != (poly[j].y() > p.y())) &&
                (p.x() < (poly[j].x() - poly[i].x()) * (p.y() - poly[i].y()) /
                                 (poly[j].y() - poly[i].y()) +
                             poly[i].x())) {
                inside = !inside;
            }
        }
        return inside;
    }
}// namespace rc_esdf