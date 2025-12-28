#include "map/local_map.hpp"
namespace map {
    LocalMap::LocalMap(const rclcpp::NodeOptions &options)
        : Node("LocalMap",options)
    {
        this->init_parameter();
        this->init_local_map();
        point_cloud_sub =
            this->create_subscription<sensor_msgs::msg::PointCloud2>("/points2",
                10,
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    point_cloud_callback(std::forward<decltype(msg)>(msg));
                });
        map_pub =
            this->create_publisher<grid_map_msgs::msg::GridMap>("/local_map",
                10);
    }

    void LocalMap::init_local_map()
    {
        this->local_map.setFrameId(this->Local_map_config.map_frame);
        this->local_map.setGeometry(
            grid_map::Length(Local_map_config.map_size_x,
                Local_map_config.map_size_y),
            this->Local_map_config.resolution,
            grid_map::Position(0.0, 0.0));// 地图中心在原点
        this->local_map.add("height",
            NAN);// 2.5D高度层（存储每个栅格的实际高度，初始为NaN）
        this->local_map.add("ground", 0.0);// 地面层（1=地面，0=非地面）
        this->local_map.add("obstacle", 0.0);// 障碍物层（1=障碍物，0=无）
        this->local_map.add("esdf", 0.0);// ESDF层（基于障碍物层）
    }

    void LocalMap::point_cloud_callback(
        sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        local_map.clear("height");
        local_map.clear("ground");
        local_map.clear("obstacle");

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);
        // 2. 预处理：过滤无效点+降采样（提升效率）
        pcl_cloud = preprocess_point_cloud(pcl_cloud);
        get_local_map(pcl_cloud);
    }

    grid_map::GridMap LocalMap::get_local_map(
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
                ;
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
                point.z > Local_map_config.max_point_height) {
                continue;
            }

            grid_map::Position position(point.x, point.y);
            if (!local_map.isInside(position)) {
                continue;
            }
            // 栅格位置 -> 索引（用于聚合同一栅格的多个点）
            grid_map::Index index;
            local_map.getIndex(position, index);
            grid_height_map[index].push_back(point.z);
        }
        for (const auto& [index, heights]: grid_height_map) {
            if (heights.empty()) {
                continue;
            }
            // 计算栅格高度（可选：最大值/平均值/中位数）
            float max_height =
                *std::max_element(heights.begin(), heights.end());
            local_map.at("height", index) = max_height;

            // 区分地面/障碍物
            if (max_height < Local_map_config.ground_height_threshold) {
                local_map.at("ground", index) = 1.0;// 标记为地面
            }
            else if (max_height > Local_map_config.obstacle_height_threshold) {
                local_map.at("obstacle", index) = 1.0;// 标记为障碍物
            }
        }
        // 5. 填充无数据区域的高度（可选：用周围地面高度插值）
        fill_nan_heights(local_map);
        get_esdf_layer(local_map);
        // 7. 发布2.5D栅格地图
        grid_map_msgs::msg::GridMap grid_map_msg;
        grid_map::GridMapRosConverter::toMessage(local_map);
        grid_map_msg.header.stamp = this->now();
        grid_map_msg.header.frame_id = Local_map_config.map_frame;
        map_pub->publish(grid_map_msg);
        return this->local_map;
    }

    grid_map::GridMap LocalMap::get_esdf_layer(grid_map::GridMap local_map)
    {
        grid_map::SignedDistanceField esdf;

        // 确保esdf图层存在且尺寸匹配
        if (!local_map.exists("esdf")) {
            local_map.add("esdf",
                grid_map::Matrix::Zero(local_map.getSize()(0),
                    local_map.getSize()(1)));
        }

        grid_map::Matrix& esdf_layer = local_map["esdf"];

        esdf.calculateSignedDistanceField(local_map,
            "obstacle",
            Local_map_config.obstacle_height_threshold);
        for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            grid_map::Position3 pos_3d;
            local_map.getPosition3("obstacle", index, pos_3d);
            double distance = esdf.getDistanceAt(pos_3d);
            esdf_layer(index(0), index(1)) = distance;
        }
        return this->local_map;
    }

    grid_map::GridMap LocalMap::fill_nan_heights(grid_map::GridMap local_map)
    {
        grid_map::Matrix& height_layer = local_map["height"];
        grid_map::Matrix& ground_layer = local_map["ground"];
        for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            if (!std::isnan(height_layer(index(0), index(1)))) {
                continue;
            }
            std::vector<float> neighbor_heights;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    grid_map::Index neighbor_idx(index(0) + dx, index(1) + dy);
                    if (local_map.isValid(neighbor_idx) &&
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
        return local_map;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::preprocess_point_cloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, Local_map_config.max_point_height);
        pass.filter(*pcl_cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(pcl_cloud);
        auto* size = &Local_map_config.voxel_leaf_size;
        voxel_grid.setLeafSize(*size, *size, *size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid.filter(*filtered_cloud);

        return filtered_cloud;
    }

    MapConfig LocalMap::init_parameter()
    {
        this->declare_parameter<std::string>("local_map.map_frame", "map");
        this->declare_parameter<double>("local_map.resolution", 0.1);
        this->declare_parameter<double>("local_map.map_size_x", 10.0);
        this->declare_parameter<double>("local_map.map_size_y", 10.0);
        this->declare_parameter<double>("local_map.obstacle_height_threshold",
            0.1);
        this->declare_parameter<double>("local_map.ground_height_threshold",
            0.05);
        this->declare_parameter<double>("local_map.voxel_leaf_size", 0.05);
        this->declare_parameter<double>("local_map.max_point_height", 2.0);
        this->Local_map_config.map_frame =
            this->get_parameter("local_map.map_frame").as_string();
        this->Local_map_config.resolution =
            this->get_parameter("local_map.resolution").as_double();
        this->Local_map_config.map_size_x =
            this->get_parameter("local_map.map_size_x").as_double();
        this->Local_map_config.map_size_y =
            this->get_parameter("local_map.map_size_y").as_double();
        this->Local_map_config.obstacle_height_threshold =
            this->get_parameter("local_map.obstacle_height_threshold")
                .as_double();
        this->Local_map_config.ground_height_threshold =
            this->get_parameter("local_map.ground_height_threshold")
                .as_double();
        this->Local_map_config.voxel_leaf_size =
            this->get_parameter("local_map.voxel_leaf_size").as_double();
        this->Local_map_config.max_point_height =
            this->get_parameter("local_map.max_point_height").as_double();
        return this->Local_map_config;
    };
};// namespace map