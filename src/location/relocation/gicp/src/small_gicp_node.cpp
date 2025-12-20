#include "gicp/small_gicp_node.hpp"

using namespace small_gicp;
namespace relocation {

    SmallGicp::SmallGicp(): Node("small_gicp_node"){
        this->init_parameter();
        this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_registered",10,std::bind(&SmallGicp::pcd_callback, this, std::placeholders::_1));
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("align_cloud",10);
            
        
    }
    /// @brief Example to directly feed pcl::PointCloud<pcl::PointCovariance> to small_gicp::Registration.
    void SmallGicp::relocation_gicp(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw_target,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw_source)
    {
        // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>.
        pcl::PointCloud<pcl::PointCovariance>::Ptr target =
            voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                pcl::PointCloud<pcl::PointCovariance>>(*raw_target, 0.25);
        pcl::PointCloud<pcl::PointCovariance>::Ptr source =
            voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                pcl::PointCloud<pcl::PointCovariance>>(*raw_source, 0.25);

        // Estimate covariances of points.
        const int num_threads = 4;
        const int num_neighbors = 20;
        estimate_covariances_omp(*target, num_neighbors, num_threads);
        estimate_covariances_omp(*source, num_neighbors, num_threads);

        // Create KdTree for target and source.
        auto target_tree =
            std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                target,
                KdTreeBuilderOMP(num_threads));
        auto source_tree =
            std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                source,
                KdTreeBuilderOMP(num_threads));

        Registration<GICPFactor, ParallelReductionOMP> registration;
        registration.reduction.num_threads = num_threads;
        registration.rejector.max_dist_sq = 1.0;

        // Align point clouds. Note that the input point clouds are pcl::PointCloud<pcl::PointCovariance>.
        auto result = registration.align(*target,
            *source,
            *target_tree,
            Eigen::Isometry3d::Identity());

        // Because this usage exposes all preprocessed data, you can easily re-use them to obtain the best efficiency.
        // auto result2 = registration.align(*source,
        //     *target,
        //     *source_tree,
        //     Eigen::Isometry3d::Identity());
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr SmallGicp::read_pcd()
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(
                this->small_gicp_config.pcl_path,
                *global_map) == -1) {
            RCLCPP_ERROR(this->get_logger(),
                "Couldn't read PCD file: %s",
                this->small_gicp_config.pcl_path.c_str());
        };
        return this->global_map;
    }
    GicpConfig SmallGicp::init_parameter()
    {
        this->declare_parameter<std::string>("pcd_path",
            "src/bringup/pcd/rmuc_2026.pcd");
        this->declare_parameter<int>("num_threads", 4);
        this->declare_parameter<int>("num_neighbors", 20);
        this->declare_parameter<float>("leaf_size", 0.25);
        this->declare_parameter<float>("max_dist_sq", 1.0);
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