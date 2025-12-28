#include "map/global_map.hpp"

namespace map {
    GlobalMap::GlobalMap(const rclcpp::NodeOptions &options)
        : Node("GlobalMap",options)
    {
        this->init_parameter();

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto test =
            pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *pcl_cloud);
        if (test == -1) {
            RCLCPP_ERROR(this->get_logger(),
                "无法读取PCD文件:%s",
                pcd_file_path.c_str());
        }
        this->init_global_map();
        this->read_pcl(pcl_cloud);
        map_pub =
            this->create_publisher<grid_map_msgs::msg::GridMap>("/global_map",
                10);
    }

    void GlobalMap::init_global_map()
    {
        this->global_map.setFrameId(this->global_map_config.map_frame);
        this->global_map.setGeometry(
            grid_map::Length(global_map_config.map_size_x,
                global_map_config.map_size_y),
            this->global_map_config.resolution,
            grid_map::Position(0.0, 0.0));// 地图中心在原点
        this->global_map.add("height",
            NAN);// 2.5D高度层（存储每个栅格的实际高度，初始为NaN）
        this->global_map.add("ground", 0.0);// 地面层（1=地面，0=非地面）
        this->global_map.add("obstacle", 0.0);// 障碍物层（1=障碍物，0=无）
        this->global_map.add("esdf", 0.0);// ESDF层（基于障碍物层）
    }

    void GlobalMap::read_pcl(
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud)
    {
        global_map.clear("height");
        global_map.clear("ground");
        global_map.clear("obstacle");

        // 预处理：过滤无效点+降采样（提升效率）
        pcl_cloud = preprocess_point_cloud(pcl_cloud);
        get_global_map(pcl_cloud);
    }
        void GlobalMap::point_cloud_callback(
        sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        global_map.clear("height");
        global_map.clear("ground");
        global_map.clear("obstacle");

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);
        // 2. 预处理：过滤无效点+降采样（提升效率）
        pcl_cloud = preprocess_point_cloud(pcl_cloud);
        get_global_map(pcl_cloud);
    }
    grid_map::GridMap GlobalMap::get_global_map(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
    {
        //2.5D高度层生成：每个栅格存储最高/平均高度
        //所用hash表
        //hash函数
        struct GridHash {
            size_t operator()(const grid_map::Index& index) const
            {
                size_t h1 = std::hash<int>()(index(0));
                size_t h2 = std::hash<int>()(index(1));
                return h1 * 31 + h2;
            };
        };
        //自定义相等
        struct GridEqual {
            bool operator()(const Eigen::Array<int, 2, 1>& index1,
                const Eigen::Array<int, 2, 1>& index2) const
            {
                return (index1(0) == index2(0)) && (index1(1) == index2(1));
            };
        };
        std::unordered_map<grid_map::Index,
            std::vector<double>,
            GridHash,
            GridEqual>
            grid_height_map;// 栅格索引 -> 高度列表
        for (const auto& point: *pcl_cloud) {
            if (std::isnan(point.x) || std::isnan(point.y) ||
                std::isnan(point.z) || point.z < 0.0 ||
                point.z > global_map_config.max_point_height) {
                continue;
            }

            grid_map::Position position(point.x, point.y);
            if (!global_map.isInside(position)) {
                continue;
            }
            // 栅格位置 -> 索引（用于聚合同一栅格的多个点）
            grid_map::Index index;
            global_map.getIndex(position, index);
            grid_height_map[index].push_back(point.z);
        }
        for (const auto& [index, heights]: grid_height_map) {
            if (heights.empty()) {
                continue;
            }
            // 计算栅格高度（可选：最大值/平均值/中位数）
            float max_height =
                *std::max_element(heights.begin(), heights.end());
            global_map.at("height", index) = max_height;

            // 区分地面/障碍物
            if (max_height < global_map_config.ground_height_threshold) {
                global_map.at("ground", index) = 1.0;// 标记为地面
            }
            else if (max_height > global_map_config.obstacle_height_threshold) {
                global_map.at("obstacle", index) = 1.0;// 标记为障碍物
            }
        }
        // 填充无数据区域的高度（可选：用周围地面高度插值）
        fill_nan_heights(global_map);
        get_esdf_layer(global_map);
        // 发布2.5D栅格地图
        grid_map_msgs::msg::GridMap grid_map_msg;
        grid_map::GridMapRosConverter::toMessage(global_map);
        grid_map_msg.header.stamp = this->now();
        grid_map_msg.header.frame_id = global_map_config.map_frame;
        map_pub->publish(grid_map_msg);
        return this->global_map;
    }

    grid_map::GridMap GlobalMap::get_esdf_layer(grid_map::GridMap global_map)
    {
        grid_map::SignedDistanceField esdf;

        // 确保esdf图层存在且尺寸匹配
        if (!global_map.exists("esdf")) {
            global_map.add("esdf", 0.0);
        }

        grid_map::Matrix& esdf_layer = global_map["esdf"];

        esdf.calculateSignedDistanceField(global_map,
            "obstacle",
            global_map_config.obstacle_height_threshold);
        for (grid_map::GridMapIterator it(global_map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            grid_map::Position3 pos_3d;
            global_map.getPosition3("obstacle", index, pos_3d);
            double distance = esdf.getDistanceAt(pos_3d);
            esdf_layer(index(0), index(1)) = distance;
        }
        return this->global_map;
    }

    grid_map::GridMap GlobalMap::fill_nan_heights(grid_map::GridMap global_map)
    {
        grid_map::Matrix& height_layer = global_map["height"];
        grid_map::Matrix& ground_layer = global_map["ground"];
        for (grid_map::GridMapIterator it(global_map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            if (!std::isnan(height_layer(index(0), index(1)))) {
                continue;
            }
            std::vector<float> neighbor_heights;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    grid_map::Index neighbor_idx(index(0) + dx, index(1) + dy);
                    if (global_map.isValid(neighbor_idx) &&
                        !std::isnan(
                            height_layer(neighbor_idx(0), neighbor_idx(1))) &&
                        ground_layer(neighbor_idx(0), neighbor_idx(1)) == 1.0) {
                        neighbor_heights.push_back(
                            height_layer(neighbor_idx(0), neighbor_idx(1)));
                    }
                }
            }
            // 填充邻域平均高度
            if (!neighbor_heights.empty()) {
                double avg_height = std::accumulate(neighbor_heights.begin(),
                                        neighbor_heights.end(),
                                        0.0f) /
                                    neighbor_heights.size();
                height_layer(index(0), index(1)) = avg_height;
            }
            else {
                height_layer(index(0), index(1)) = 0.0;// 无邻域则设为0
            }
        }
        return global_map;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalMap::preprocess_point_cloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, global_map_config.max_point_height);
        pass.filter(*pcl_cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(pcl_cloud);
        auto* size = &global_map_config.voxel_leaf_size;
        voxel_grid.setLeafSize(*size, *size, *size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid.filter(*filtered_cloud);

        return filtered_cloud;
    }

    MapConfig GlobalMap::init_parameter()
    {
        this->declare_parameter<std::string>("global_map.map_frame", "map");
        this->declare_parameter<double>("global_map.resolution", 0.1);
        this->declare_parameter<double>("global_map.map_size_x", 10.0);
        this->declare_parameter<double>("global_map.map_size_y", 10.0);
        this->declare_parameter<double>("global_map.obstacle_height_threshold",
            0.1);
        this->declare_parameter<double>("global_map.ground_height_threshold",
            0.05);
        this->declare_parameter<double>("global_map.voxel_leaf_size", 0.05);
        this->declare_parameter<double>("global_map.max_point_height", 2.0);
        this->declare_parameter<std::string>("pcd", "src/bringup/pcd/rmuc_2025.pcd");
        this->global_map_config.map_frame =
            this->get_parameter("global_map.map_frame").as_string();
        this->global_map_config.resolution =
            this->get_parameter("global_map.resolution").as_double();
        this->global_map_config.map_size_x =
            this->get_parameter("global_map.map_size_x").as_double();
        this->global_map_config.map_size_y =
            this->get_parameter("global_map.map_size_y").as_double();
        this->global_map_config.obstacle_height_threshold =
            this->get_parameter("global_map.obstacle_height_threshold")
                .as_double();
        this->global_map_config.ground_height_threshold =
            this->get_parameter("global_map.ground_height_threshold")
                .as_double();
        this->global_map_config.voxel_leaf_size =
            this->get_parameter("global_map.voxel_leaf_size").as_double();
        this->global_map_config.max_point_height =
            this->get_parameter("global_map.max_point_height").as_double();
        this->pcd_file_path = this->get_parameter("pcd").as_string();
        return this->global_map_config;
    };
};// namespace map