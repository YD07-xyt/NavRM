#include "config.hpp"
#include "rc_esdf.h"
namespace planner {


    class ESDFMap
    {
    public:
        explicit ESDFMap(MapConfig params,
            std::vector<Eigen::Vector2d>& obs_points_body);
        std::tuple<bool, double, Eigen::Vector2d> query(
            const Eigen::Vector2d& obs_points_body);
        std::tuple<bool, double, Eigen::Vector2d> query(int obs_points_body_x,int obs_points_body_y);
    public:
        MapConfig params;

    private:
        rc_esdf::RcEsdfMap esdf_map;
        std::vector<Eigen::Vector2d> footprint;

    private:
        void update_esdf(std::vector<Eigen::Vector2d> footprint);
        void init_esdf_map(MapConfig param,
            std::vector<Eigen::Vector2d> footprint);
    };


}// namespace planner