#include "planner_map.hpp"

namespace planner {

    ESDFMap::ESDFMap(MapConfig params,
        std::vector<Eigen::Vector2d>& obs_points_body)
        : params(params)
    {

        ESDFMap::init_esdf_map(ESDFMap::params, ESDFMap::footprint);
    };
    void ESDFMap::init_esdf_map(MapConfig param,
        std::vector<Eigen::Vector2d> footprint)
    {
        ESDFMap::esdf_map.initialize(param.width_m,
            param.height_m,
            param.resolution);
        ESDFMap::esdf_map.generateFromPolygon(footprint);
    };


    void ESDFMap::update_esdf(std::vector<Eigen::Vector2d> footprint)
    {
        ESDFMap::esdf_map.generateFromPolygon(footprint);
    }


    std::tuple<bool, double, Eigen::Vector2d> ESDFMap::query(
        const Eigen::Vector2d& obs_points_body)
    {
        bool in_box;
        double dist;
        Eigen::Vector2d grad;
        // 核心查询函数
        in_box = ESDFMap::esdf_map.query(obs_points_body, dist, grad);
        return std::make_tuple(in_box, dist, grad);
    }
    std::tuple<bool, double, Eigen::Vector2d> ESDFMap::query(int obs_points_body_x,
        int obs_points_body_y)
    {
        bool in_box;
        double dist;
        Eigen::Vector2d grad;
        // 核心查询函数
        in_box = ESDFMap::esdf_map.query(
            Eigen::Vector2d(obs_points_body_x, obs_points_body_y),
            dist,
            grad);
        return std::make_tuple(in_box, dist, grad);
    };

}// namespace planner