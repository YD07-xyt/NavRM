#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
// #include "../../../..tool/io_yaml/include/io_yaml.hpp"
namespace planner {

//TODO:参数未设置
struct ESDFMapConfig{
    double width_m;
    double height_m;
    double resolution;
};

struct OccupancyGridMapConfig{
    double resolution=0.6; //分辨率
    double detection_range=1.0; //传感器的最大检测距离
    double global_x_lower=5.0;
    double global_x_upper=5.0;
    double global_y_lower=5.0;
    double global_y_upper=5.0;
    bool if_perspective;//透视模式
    bool if_cirSupRaycast;
    //FOV视场角  
    // 是否启用水平视场角限制
    bool hrz_limited;  // true: 启用FOV限制, false: 无限制(360°)
    // 水平激光视场角范围（单位：度）
    double hrz_laser_range_dgr=360.0;   
    double p_hit =0.79;
    double p_miss=0.35;
    double p_min=0.12;
    double p_max=0.90;
    double p_occ=0.8;

    // OccupancyGridMapConfig(){
    //     auto yaml_node=tools::load("src/planner/planner_map/planner_map/config/param.yaml");
    //     resolution=tools::read<double>(yaml_node, "resolution");
    //     detection_range=tools::read<double>(yaml_node, "detection_range");
    //     global_x_lower=tools::read<double>(yaml_node, "global_x_lower");
    //     global_x_upper=tools::read<double>(yaml_node, "global_x_upper");
    //     global_y_lower=tools::read<double>(yaml_node, "global_y_lower");
    //     global_y_upper=tools::read<double>(yaml_node, "global_y_upper");
    //     if_perspective=tools::read<bool>(yaml_node, "if_perspective");
    //     if_cirSupRaycast=tools::read<bool>(yaml_node, "if_cirSupRaycast");
    //     hrz_limited=tools::read<bool>(yaml_node, "hrz_limited");
    //     hrz_laser_range_dgr=tools::read<double>(yaml_node, "hrz_laser_range_dgr");
    //     p_hit=tools::read<double>(yaml_node, "p_hit");
    //     p_miss=tools::read<double>(yaml_node, "p_miss");
    //     p_min=tools::read<double>(yaml_node, "p_min");
    //     p_max=tools::read<double>(yaml_node, "p_max");
    //     p_occ=tools::read<double>(yaml_node, "p_occ");
    //}
};
}
#endif 