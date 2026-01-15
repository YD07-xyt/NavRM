#include "planning_env/core/processing.hpp"

#include <algorithm>
#include <grid_map_core/iterators/SlidingWindowIterator.hpp>


namespace planner {


    inline pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filters(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
        float voxel_leaf_size)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;

        sor.setInputCloud(input_cloud);

        sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        sor.filter(*output_cloud);
        return output_cloud;
    }


    inline grid_map::GridMap init_grid_map(GridMapConfig grid_map_config)
    {
        grid_map::GridMap grid_map;
        grid_map.setFrameId(grid_map_config.map_frame);
        grid_map.setGeometry(grid_map::Length(grid_map_config.map_size_x,
                                 grid_map_config.map_size_y),
            grid_map_config.resolution,
            grid_map::Position(grid_map_config.map_postion_x,
                grid_map_config.map_size_y));// 地图尺寸、分辨率、原点
        grid_map.add("elevation", NAN);      // 高程层 (z轴高度)
        grid_map.add("Semantics");           // 语义层
        grid_map.add("obstacles", 0.0);// 障碍物层 (0:可通行, 1:障碍物)
        grid_map.add("inflation", 0.0);                          // 膨胀层
        grid_map.add("esdf", std::numeric_limits<double>::max());//esdf层

        spdlog::info("初始化2.5d map");
        return grid_map;
    }

    inline grid_map::GridMap init_elevation_layer(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        grid_map::GridMap& grid_map)
    {
        if (point_cloud->empty()) {
            spdlog::warn("map增加elevation层时传入点云为空");
            return grid_map;
        }
        if (!grid_map.exists("elevation")) {
            grid_map.add("elevation");
        }
        for (const auto& point: point_cloud->points) {
            grid_map::Position pos(point.x, point.y);
            if (grid_map.isInside(pos)) {
                grid_map.atPosition("elevation", pos) = point.z;
            }
        }
    }

    inline grid_map::GridMap init_obstacles_map_layer(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        grid_map::GridMap& grid_map,
        float min_obstacle_height,
        float max_obstacle_height)
    {
        if (point_cloud->empty()) {
            spdlog::warn("map增加obstacles层时传入点云为空");
            return grid_map;
        }
        if (!grid_map.exists("obstacles")) {
            grid_map.add("obstacles", 0.0);
        }
        grid_map::Index index;
        for (const auto& point: point_cloud->points) {
            if (point.z > min_obstacle_height &&
                point.z < max_obstacle_height) {
                grid_map::Position position(point.x, point.y);

                if (grid_map.getIndex(position, index)) {
                    grid_map.at("obstacles", index) = 1.0;
                }
            }
        }
    }
    /*
    * @brief:  map 增加 膨胀层
    * @params: map (含障碍物层)
    * @return: 输出map
    */
    inline grid_map::GridMap init_inflation_map_layer(
        grid_map::GridMap& grid_map,
        double inflation_radius)
    {
        if (!grid_map.exists("inflation")) {
            grid_map.add("inflation", 0.0);
        }
        if (!grid_map.exists("obstacles")) {
            spdlog::error("map增加膨胀层时obstacles层为空");
            return grid_map;
        }
        inflate_layer(grid_map, "obstacles", "inflation", inflation_radius);
        return grid_map;
    }

    inline grid_map::GridMap init_esdf_map_layer(grid_map::GridMap& grid_map,
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        GridMapConfig& grid_map_config)
    {
        if (!grid_map.exists("esdf")) {
            grid_map.add("esdf", std::numeric_limits<double>::max());
        }

        Eigen::Vector3d origin(grid_map_config.map_postion_x,
            grid_map_config.map_postion_y,
            grid_map_config.map_postion_z);
        double resolution = grid_map_config.resolution;
        Eigen::Vector3d map_size(grid_map_config.map_size_x,
            grid_map_config.map_size_y,
            grid_map_config.map_size_z);// 长、宽、高

        fiesta::ESDFMap esdf_generator(origin, resolution, map_size);
        for (const auto& point: point_cloud->points) {
            Eigen::Vector3d pos(point.x, point.y, point.z);
            esdf_generator.SetOccupancy(pos, 1);
        }
        esdf_generator.UpdateESDF();
        FiestaESDFMap_to_GridMap(esdf_generator, grid_map, "esdf");
    }


    //============================================================================//
    // tool

    inline void inflate_layer(grid_map::GridMap& grid_map,
        const std::string& sourceLayer,
        const std::string& targetLayer,
        double inflation_radius)
    {
        if (!grid_map.exists(targetLayer)) {
            grid_map.add(targetLayer);
        }
        grid_map[targetLayer] = grid_map[sourceLayer];

        //计算窗口大小（根据地图分辨率将半径转为像素数）
        double resolution = grid_map.getResolution();
        int kernelSize = std::ceil(inflation_radius / resolution) * 2 + 1;

        // 使用滑动窗口迭代器
        // 这里是对整个地图进行窗口搜索
        for (grid_map::SlidingWindowIterator it(grid_map,
                 sourceLayer,
                 grid_map::SlidingWindowIterator::EdgeHandling::CROP,
                 kernelSize);
             !it.isPastEnd();
             ++it) {

            // 如果当前窗口内存在任何障碍物
            // 使用 Eigen 的 max() 方法快速判断
            if (it.getData().maxCoeff() > 0.5) {
                grid_map.at(targetLayer, *it) = 1.0;
            }
        }
    }

    inline double squared(double x)
    {
        return x * x;
    }

    inline void dual_scan_update_esdf(grid_map::GridMap& grid_map,
        const std::string& inputLayer,
        const std::string& outputLayer)
    {
        if (!grid_map.exists(outputLayer)) {
            grid_map.add(outputLayer);
        }

        auto& input = grid_map[inputLayer];
        auto& esdf = grid_map[outputLayer];

        const int rows = grid_map.getSize()(0);
        const int cols = grid_map.getSize()(1);
        const double res = grid_map.getResolution();

        // 初始化：障碍物处为0，非障碍物处为无穷大
        esdf.setConstant(std::numeric_limits<double>::max());
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (input(i, j) > 0.5) {
                    esdf(i, j) = 0.0;
                }
            }
        }

        // 纵向扫描
        for (int j = 0; j < cols; ++j) {
            for (int i = 1; i < rows; ++i) {
                esdf(i, j) = std::min<float>(esdf(i, j), esdf(i - 1, j) + res);
            }
            for (int i = rows - 2; i >= 0; --i) {
                esdf(i, j) = std::min<float>(esdf(i, j), esdf(i + 1, j) + res);
            }
        }

        // 横向扫描
        for (int i = 0; i < rows; ++i) {
            for (int j = 1; j < cols; ++j) {
                esdf(i, j) = std::min<float>(esdf(i, j), esdf(i, j - 1) + res);
            }
            for (int j = cols - 2; j >= 0; --j) {
                esdf(i, j) = std::min<float>(esdf(i, j), esdf(i, j + 1) + res);
            }
        }

        spdlog::info("ESDF layer updated.");
    }
    
    inline void FiestaESDFMap_to_GridMap(fiesta::ESDFMap& fiesta_esdf_map,
        grid_map::GridMap& grid_map,
        const std::string& layer_name)
    {
        if (!grid_map.exists(layer_name)) {
            grid_map.add(layer_name);
        }

        // 遍历 GridMap 的所有栅格
        for (grid_map::GridMapIterator iterator(grid_map);
             !iterator.isPastEnd();
             ++iterator) {
            grid_map::Position pos;
            grid_map.getPosition(*iterator, pos);

            // FIESTA 是 3D 的，我们需要给它一个 Z 轴坐标（通常取地图中心高度）
            Eigen::Vector3d pos_3d(pos.x(), pos.y(), 0.0);

            // 获取距离并写入 GridMap
            float dist =
                static_cast<float>(fiesta_esdf_map.GetDistance(pos_3d));
            grid_map.at(layer_name, *iterator) = dist;
        }
    }
}// namespace planner