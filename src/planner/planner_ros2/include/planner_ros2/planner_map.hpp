#include"rc_esdf.h"
#include"config.hpp"
namespace planner {


class Map {
    public:
        std::tuple<bool,double,Eigen::Vector2d> query(std::vector<Eigen::Vector2d> obs_points_body);
        void update_esdf(std::vector<Eigen::Vector2d> footprint);
    private:
        rc_esdf::RcEsdfMap esdf_map;
    private:
        void init_esdf_map(MapConfig param,std::vector<Eigen::Vector2d> footprint);
};


}