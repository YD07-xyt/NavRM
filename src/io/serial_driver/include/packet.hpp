#pragma once

#ifndef PACKET_HPP
#define PACKET_HPP
#include <algorithm>
#include <cstdint>
#include <vector>
namespace io {
namespace serial {
const uint8_t SOF_RECEIVE = {'M'};
const uint8_t SOF_SEND = {'M'};


enum class send_id : uint8_t{
  id_robot_cmd=0x01,
};

enum class receive_id : uint16_t{
  id_game_status =0x0001,
  id_robot_status=0x0201,
};

struct HeaderFrame {
  uint8_t sof; // 数据帧起始字节，固定值为 M
  uint16_t len; // 数据段长度
  uint8_t id;  // 数据段id
  uint8_t crc; // 数据帧头的 CRC8 校验
} __attribute__((packed));


// 比赛信息数据包 需要
struct ReceiveGameStatusData {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  uint16_t cmd_id;
  struct {
    // 比赛进度 当前比赛阶段 0-5
    uint8_t game_progress;
    // 当前比赛阶段剩余时间
    uint16_t stage_remain_time;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));


// 机器人状态数据包 需要
struct ReceiveRobotStatus {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  uint16_t cmd_id;
  struct {
    // 本机器人 ID
    uint8_t robot_id;
    // 机器人等级
    uint8_t robot_level;
    // 机器人当前血量
    uint16_t current_up;
    // 机器人血量上限
    uint16_t maximum_hp;
    // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_cooling_value;
    // 机器人射击热量上限
    uint16_t shooter_barrel_heat_limit;
    // 机器人底盘功率上限
    uint16_t shooter_17mm_1_barrel_heat;
    // 本机器人位置 x 坐标
    float robot_pos_x;
    // 本机器人位置 y 坐标
    float robot_pos_y;
    // 本机器人测速模块的朝向
    float robot_pos_angle;
    /* 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
    数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0*/
    uint8_t armor_id : 4;
    // 血量变化类型
    uint8_t hp_deduction_reason : 4;
    // 机器人自身拥有的 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_17mm;
    // 剩余金币数量
    uint16_t remaining_gold_coin;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));



/********************************************************/
/* Send data                                            */
/********************************************************/

struct SendRobotCmdData {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  uint8_t is_scan;
  uint8_t sentry_pose;
  struct {
    // 速度
    struct {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __attribute__((packed));

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T> inline T fromVector(const std::vector<uint8_t> &data) {
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T> inline std::vector<uint8_t> toVector(const T &data) {
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
            packet.begin());
  return packet;
}

}; // namespace serial
} // namespace io

#endif