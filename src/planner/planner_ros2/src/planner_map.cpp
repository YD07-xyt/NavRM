#include "planner_ros2/planner_map.hpp"

namespace planner {

    void Map::init_esdf_map(MapConfig param,
        std::vector<Eigen::Vector2d> footprint)
    {
        Map::esdf_map.initialize(param.width_m,
            param.height_m,
            param.resolution);
        Map::esdf_map.generateFromPolygon(footprint);
    };

    
    void Map::update_esdf(std::vector<Eigen::Vector2d> footprint){
        Map::esdf_map.generateFromPolygon(footprint);
    }


    std::tuple<bool, double, Eigen::Vector2d> Map::query(
        std::vector<Eigen::Vector2d> obs_points_body)
    {
        double dist;
        Eigen::Vector2d grad;
        bool in_box;
        for (const auto& p: obs_points_body) {
            // 核心查询函数
            in_box = Map::esdf_map.query(p, dist, grad);
        }
        return std::make_tuple(in_box, dist, grad);
    }
}// namespace planner