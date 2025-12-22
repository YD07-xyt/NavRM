#include "gicp/gicp_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
using namespace small_gicp;
namespace relocation {

    SmallGicp::SmallGicp()
        : Node("gicp_node")
        , result_t(Eigen::Isometry3d::Identity())
        , target_pcd(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
        , source_pcd(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
        , target(std::make_shared<pcl::PointCloud<pcl::PointCovariance>>())
        // 初始化配准器
        ,registration(
            std::make_shared<small_gicp::Registration<small_gicp::GICPFactor,
                small_gicp::ParallelReductionOMP>>())
    {
        RCLCPP_INFO(get_logger(), "small_gicp开始接收pcd_msg");
        this->init_parameter();
        this->read_pcd();

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        target = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
            pcl::PointCloud<pcl::PointCovariance>>(*this->source_pcd,
            this->small_gicp_config.leaf_size);

        estimate_covariances_omp(*target,
            this->small_gicp_config.num_neighbors,
            this->small_gicp_config.num_threads);

        target_tree =
            std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                target,
                KdTreeBuilderOMP(this->small_gicp_config.num_threads));


        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_registered",
            10,
            std::bind(&SmallGicp::pcd_callback, this, std::placeholders::_1));
    }
    /// @brief Example to directly feed pcl::PointCloud<pcl::PointCovariance> to small_gicp::Registration.
    small_gicp::RegistrationResult SmallGicp::relocation_gicp(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source_pcd)
    {
        // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>.
        pcl::PointCloud<pcl::PointCovariance>::Ptr source =
            voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                pcl::PointCloud<pcl::PointCovariance>>(*source_pcd,
                this->small_gicp_config.leaf_size);

        // Estimate covariances of points.
        const int num_threads = this->small_gicp_config.num_threads;
        const int num_neighbors = this->small_gicp_config.num_neighbors;
        estimate_covariances_omp(*source, num_neighbors, num_threads);

        // Create KdTree for source.
        auto source_tree =
            std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                source,
                KdTreeBuilderOMP(num_threads));

        registration->reduction.num_threads = num_threads;
        registration->rejector.max_dist_sq =
            this->small_gicp_config.max_dist_sq;
        auto previous_result_t = result_t;
        // Align point clouds. Note that the input point clouds are pcl::PointCloud<pcl::PointCovariance>.
        auto result = registration->align(*target,
            *source,
            *target_tree,
            previous_result_t);
        result_t = result.T_target_source;
        return result;
    }
    void SmallGicp::pcd_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg)
    {
        RCLCPP_INFO(get_logger(), "small_gicp开始接收pcd_msg");

        auto last_scan_time = pcd_msg->header.stamp;
        auto current_scan_frame_id = pcd_msg->header.frame_id;
        
        source_pcd->clear();
        pcl::fromROSMsg(*pcd_msg, *source_pcd);

        auto result = this->relocation_gicp(source_pcd);

        RCLCPP_INFO(get_logger(), "small_gicp迭代次数: %zu", result.iterations);

        if (!result.converged) {
            RCLCPP_ERROR(get_logger(), "small_gicp 匹配失败，未收敛");
        }
        else {
            RCLCPP_INFO(get_logger(), "small_gicp 匹配成功，收敛");
            publish_transform(result_t, last_scan_time);
        }
    }
    void SmallGicp::publish_transform(Eigen::Isometry3d result_t,
        rclcpp::Time last_scan_time)
    {
        if (result_t.matrix().isZero()) {
            return;
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
        transform_stamped.header.stamp =
            last_scan_time + rclcpp::Duration::from_seconds(0.1);
        transform_stamped.header.frame_id = this->small_gicp_config.map_frame;
        transform_stamped.child_frame_id = this->small_gicp_config.odom_frame;

        const Eigen::Vector3d translation = result_t.translation();
        const Eigen::Quaterniond rotation(result_t.rotation());

        transform_stamped.transform.translation.x = translation.x();
        transform_stamped.transform.translation.y = translation.y();
        transform_stamped.transform.translation.z = translation.z();
        transform_stamped.transform.rotation.x = rotation.x();
        transform_stamped.transform.rotation.y = rotation.y();
        transform_stamped.transform.rotation.z = rotation.z();
        transform_stamped.transform.rotation.w = rotation.w();

        tf_broadcaster->sendTransform(transform_stamped);
        RCLCPP_DEBUG(get_logger(), "发布TF: %s -> %s, 平移(x:%.2f, y:%.2f, z:%.2f)",
                 transform_stamped.header.frame_id.c_str(),
                 transform_stamped.child_frame_id.c_str(),
                 translation.x(), translation.y(), translation.z());
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr SmallGicp::read_pcd()
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(
                this->small_gicp_config.pcl_path,
                *target_pcd) == -1) {
            RCLCPP_ERROR(this->get_logger(),
                "Couldn't read PCD file: %s",
                this->small_gicp_config.pcl_path.c_str());
        };
        return this->target_pcd;
    }


    GicpConfig SmallGicp::init_parameter()
    {
        RCLCPP_INFO(get_logger(), "small_gicp开始init_parameter");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("pcd_path",
            "src/bringup/pcd/rmuc_2026.pcd");
        this->declare_parameter<int>("num_threads", 4);
        this->declare_parameter<int>("num_neighbors", 20);
        this->declare_parameter<float>("leaf_size", 0.25);
        this->declare_parameter<float>("max_dist_sq", 1.0);
        this->small_gicp_config.map_frame =
            this->get_parameter("map_frame").as_string();
        this->small_gicp_config.odom_frame =
            this->get_parameter("odom_frame").as_string();
        this->small_gicp_config.pcl_path =
            this->get_parameter("pcd_path").as_string();
        this->small_gicp_config.num_threads =
            this->get_parameter("num_threads").as_int();

        this->small_gicp_config.num_neighbors =
            this->get_parameter("num_neighbors").as_int();
        this->small_gicp_config.leaf_size =
            this->get_parameter("leaf_size").as_double();
        this->small_gicp_config.max_dist_sq =
            this->get_parameter("max_dist_sq").as_double();
        return this->small_gicp_config;
    };
}// namespace relocation