#pragma once

#ifndef BT2SERIAL_HPP
#define BT2SERIAL_HPP

#include<rclcpp/rclcpp.hpp>

#include"rm_interfaces/msg/bt_data.hpp"

#include <behaviortree_cpp/behavior_tree.h>

namespace decision {
    class BT2Serial : public BT::SyncActionNode {
    public:
        BT2Serial(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts();
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<rm_interfaces::msg::BtData>::SharedPtr bt_data_pub_;
        void publish();
        rm_interfaces::msg::BtData bt_data_;
    };
}
#endif