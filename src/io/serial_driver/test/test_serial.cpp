#include"serial_driver.hpp"
#include"../../../tool/rm_log/include/glog.hpp"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <vector>   
#include"crc.hpp"
#include"packet.hpp"

 std::vector<uint8_t> receive(io::serial::SerialDriver& serial_driver,const size_t &heard_data_length,const size_t &data_length){
    std::vector<uint8_t> heard_data(heard_data_length);
    std::vector<uint8_t> datas(data_length);
    while (1) {
        datas=serial_driver.receive_data();
        for(int i=0;i<datas.size();++i){
            if(datas[i]==0xA5){
                heard_data.assign(datas[i],datas[i+heard_data_length-1]);
                bool is_crc8_ok=
                io::serial::crc8::verify_CRC8_check_sum(heard_data.data(), sizeof(heard_data));
                if(!is_crc8_ok){
                    LOG(ERROR)<< RED <<"crc8 failed"<<RESET;
                    break;
                }
                bool is_crc16_ok=
                io::serial::crc16::verify_CRC16_check_sum(datas.data(),sizeof(datas));
                if(!is_crc16_ok){
                    LOG(ERROR)<<RED<<"crc16 failed"<<RESET;
                    break;
                }
                return datas;    
            }else {
                break;
            }
        }
    }
}

void read_robot_status(const std::vector<uint8_t>& receive_data){
    
}

void read_game_status(const std::vector<uint8_t>& receive_data){

}
void analysis_data(const std::vector<uint8_t>& receive_data){
    uint16_t id=static_cast<uint16_t>(receive_data[5])+static_cast<uint16_t>(receive_data[6]);
    switch (id) {
        case static_cast<uint16_t>(io::serial::receive_id::id_robot_status) :{
            io::serial::ReceiveRobotStatus robot_state=io::serial::fromVector<io::serial::ReceiveRobotStatus>(receive_data);
            //TODO
            read_robot_status(receive_data);
        }break;
        case static_cast<uint16_t>(io::serial::receive_id::id_game_status) : {
            io::serial::ReceiveGameStatusData game_status_data = io::serial::fromVector<io::serial::ReceiveGameStatusData>(receive_data);
            read_game_status(receive_data);        
        }break;
        default:{
            LOG(WARNING)<<YELLOW<<"id defalut"<<RESET;
        }break;
    }
}

void init_send_data(io::serial::SendRobotCmdData &send_data){
    send_data.frame_header.sof=io::serial::SOF_SEND;
    send_data.frame_header.id=static_cast<uint8_t>(io::serial::send_id::id_robot_cmd);
    send_data.frame_header.len=sizeof(io::serial::SendRobotCmdData) - 6;
    io::serial::crc8::append_CRC8_check_sum(reinterpret_cast<uint8_t *>(&send_data), sizeof(io::serial::HeaderFrame));
    send_data.is_scan=0;
    send_data.sentry_pose=3;//move姿态
    send_data.data.speed_vector.vx= 0;
    send_data.data.speed_vector.vy= 0;
    send_data.data.speed_vector.wz= 0;
}

void send_speed(const float &vx,const float &vy,const float &wz,io::serial::SendRobotCmdData& send_data){
    send_data.data.speed_vector.vx =vx;
    send_data.data.speed_vector.vy =vy;
    send_data.data.speed_vector.wz =wz;
};

void send_sentry_pose(const int &sentry_pose,io::serial::SendRobotCmdData& send_data){
    if(!(sentry_pose==1)||!(sentry_pose==2)||!(sentry_pose==3)){
        LOG(ERROR)<<RED<<"传入的sentry_pose超出范围"<<RESET;
        return;
    }
    send_data.sentry_pose=sentry_pose;
};

void send_is_scan(const int &is_scan,io::serial::SendRobotCmdData& send_data){
    if(!(is_scan==0)||!(is_scan==1)){
        LOG(ERROR)<<RED<<"传入的is_scan超出范围"<<RESET;
        return;
    }
    send_data.is_scan=is_scan;
};

void send_data(io::serial::SerialDriver &serial_driver, io::serial::SendRobotCmdData &send_data ,const size_t &heard_data_length,const size_t &data_length){
    while(1){
        io::serial::crc16::append_CRC16_check_sum(reinterpret_cast<uint8_t *>(&send_data), sizeof(io::serial::SendRobotCmdData));
        serial_driver.send_data(io::serial::toVector(send_data));
        
    }

}

int main(){
    io::serial::SerialDriver serial_driver;
    std::vector<uint8_t> receive_data=receive(serial_driver,5,17);
    analysis_data(receive_data);
    io::serial::SendRobotCmdData  send_data;
    init_send_data(send_data);


    return 0;
}