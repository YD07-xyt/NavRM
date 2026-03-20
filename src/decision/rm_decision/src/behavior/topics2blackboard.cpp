#include"behavior/topics2blackboard.hpp"
#include "behaviortree_cpp/bt_factory.h"
namespace decision{
    Topics2Blackboard::Topics2Blackboard(const std::string &name, const BT::NodeConfig &config,
             std::shared_ptr<rclcpp::Node> node): BT::SyncActionNode(name, config), node_(node){
        
        init_bt_data();
        bt_data_callback();

        game_status_sub_ = node_->create_subscription<rm_interfaces::msg::GameStatus>(
            "game_status",        
            10,                     
            std::bind(&Topics2Blackboard::game_status_callback, 
                    this, 
                    std::placeholders::_1) 
        );
        
        robot_status_sub_ = node_->create_subscription<rm_interfaces::msg::RobotStatus>(
            "robot_status",          
            10,                     
            std::bind(&Topics2Blackboard::robot_status_callback, 
                    this, 
                    std::placeholders::_1) 
        );
    }
    BT::PortsList Topics2Blackboard::providedPorts()
    {
        return {
            //定义输出端口
            BT::OutputPort<std::uint16_t>("current_hp"),
            BT::OutputPort<std::uint8_t>("projectile_allowance_17mm"),
            BT::OutputPort<std::uint8_t>("game_progress"),
            BT::OutputPort<std::int32_t>("stage_remain_time"),
            BT::OutputPort<std::uint8_t>("is_scan"),
            BT::OutputPort<std::uint8_t>("sentry_pose")
        };
    }

    BT::NodeStatus Topics2Blackboard::tick(){
        check_subscriber_();
        return BT::NodeStatus::SUCCESS;
    }
    void Topics2Blackboard::game_status_callback(const rm_interfaces::msg::GameStatus::SharedPtr msg)
    {
        game_status_ = *msg;
        
        // 将游戏进度写入黑板
        //阶段
        setOutput<std::uint8_t>("game_progress", msg->game_progress);
        // 将阶段剩余时间写入黑板
        setOutput<std::int32_t>("stage_remain_time", msg->stage_remain_time);

       
    }

    void Topics2Blackboard::robot_status_callback(const rm_interfaces::msg::RobotStatus::SharedPtr msg){
        robot_status_ = *msg;
        setOutput<std::uint16_t>("current_hp", robot_status_->current_hp);
        setOutput<std::uint16_t>("projectile_allowance_17mm", msg->projectile_allowance_17mm);
    }
    
    void Topics2Blackboard::init_bt_data(){
        this->bt_data_->is_scan = 0;// 不扫描
        this->bt_data_->sentry_pose =3 ;//默认移动姿态
    }

    void Topics2Blackboard::bt_data_callback(){
        setOutput("is_scan",this->bt_data_->is_scan);
        setOutput("sentry_pose",this->bt_data_->sentry_pose);
    }

    void Topics2Blackboard::check_subscriber_()
    {
        if(!game_status_)
        {
            RCLCPP_WARN(rclcpp::get_logger("Topics2Blackboard"), "game_status_ is null");
            setOutput<std::uint16_t>("current_hp", 0);
            setOutput<std::uint8_t>("game_progress", 0);
            setOutput<std::uint16_t>("stage_remain_time", 0);
            setOutput<std::uint8_t>("armor_id", 0);
            setOutput<std::uint8_t>("hurt_type", 0);
            setOutput<std::uint16_t>("my_outpost_hp", 0);
            setOutput<std::uint16_t>("enemy_outpost_hp", 0);
            setOutput<std::uint16_t>("my_base_hp", 0);
            setOutput<std::uint16_t>("enemy_base_hp", 0);
            setOutput<std::uint16_t>("projectile_allowance_17mm", 0);
        }
    }
}