#include "behavior/bt2serial.hpp"

namespace decision {
    BT2Serial::BT2Serial(const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<rclcpp::Node> node)
        : BT::SyncActionNode(name, config)
        , node_(node)
    {
        this->bt_data_pub_= node_->create_publisher<rm_interfaces::msg::BtData>("/decision/bt_data", 10);

    }
    BT::PortsList BT2Serial::providedPorts()
    {
        return {
            // 输入端口 - 从黑板读取的数据
            BT::InputPort<std::uint8_t>("is_scan", "Scan status (0: not scanning, 1: scanning)"),
            BT::InputPort<std::uint8_t>("sentry_pose","1 is attack,2 is defend,3 is move ")
        };
    }
    
   BT::NodeStatus BT2Serial::tick() {
        publish();
        return BT::NodeStatus::SUCCESS;
    };

    void BT2Serial::publish() {
        auto is_scan = getInput<std::uint8_t>("is_scan");
        auto sentry_pose =getInput<std::uint8_t>("sentry_pose");
        bt_data_.sentry_pose = sentry_pose.value();
        bt_data_.is_scan = is_scan.value();
        bt_data_pub_->publish(bt_data_);
    }
}// namespace decision