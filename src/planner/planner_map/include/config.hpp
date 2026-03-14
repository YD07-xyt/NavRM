#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
// #include "../../../..tool/io_yaml/include/io_yaml.hpp"
namespace planner {


struct ESDFMapConfig{
    double width_m;
    double height_m;
    double resolution;
};

struct OccupancyGridMapConfig{
    double resolution; //分辨率
    double detection_range; //传感器的最大检测距离
    double global_x_lower;
    double global_x_upper;
    double global_y_lower;
    double global_y_upper;
    bool if_perspective;//透视模式
    bool if_cirSupRaycast;
    //FOV视场角  
    // 是否启用水平视场角限制
    bool hrz_limited;  // true: 启用FOV限制, false: 无限制(360°)
    // 水平激光视场角范围（单位：度）
    double hrz_laser_range_dgr;
    double p_hit;      // 占用更新概率
    double p_miss;     // 空闲更新概率
    double p_min;     // 最小概率
    double p_max;     // 最大概率
    double p_occ;      // 占用阈值
    
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