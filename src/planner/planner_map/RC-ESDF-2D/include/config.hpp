
namespace planner {

struct MapConfig{
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
    bool if_perspective; //透视模式
    bool if_cirSupRaycast; //循环补充光线投射
};
}