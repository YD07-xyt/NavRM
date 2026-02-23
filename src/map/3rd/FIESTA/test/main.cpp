#define USE_HASH_TABLE      // 使用哈希表实现
#define USE_PROBABILISTIC   // 使用概率网格
#define NEED_SIGNED_ESDF    // 需要计算带符号的ESDF

#include "Fiesta.h"
int main(int argc, char** argv)
{
    std::cout << "Before rclcpp::init" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "After rclcpp::init" << std::endl;
    
    auto node = std::make_shared<rclcpp::Node>("esdf_map");
    std::cout << "Node created" << std::endl;
    
    try {
        std::cout << "Creating Fiesta object..." << std::endl;
        fiesta::Fiesta<sensor_msgs::msg::PointCloud2,
            geometry_msgs::msg::TransformStamped>
            esdf_map(node);
        std::cout << "Fiesta object created successfully" << std::endl;
        
        std::cout << "Starting spin..." << std::endl;
        rclcpp::spin(node);
        std::cout << "Spin finished" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    std::cout << "Shutdown complete" << std::endl;
    return 0;
}