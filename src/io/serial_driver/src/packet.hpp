#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <array>
#include <cstdint>

#if defined(__GNUC__)
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#elif defined(_MSC_VER)
#define PACK(__Declaration__)                                                  \
    __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#else
#error not support packed struct
#endif

namespace serial_driver {

    struct __attribute__((packed)) SendPacket {
        //uint8_t header = 0xFF;
        uint8_t header[2] = {'M', 'A'};
        // 控制相关
        float line_vel_x; // x轴线速度，前方为正方向，单位：m/s
        float line_vel_y; // y轴线速度，左方为正方向，单位：m/s
        float angle_vel_z;// 顺时针旋转速度，弧度制，单位：rad/s

        // 决策相关
        bool is_use_top; // 是否使用小陀螺
        uint8_t priority;// 优先级   0x00-导航优先  0x01-视觉优先

        uint16_t crc16;
    };
    struct __attribute__((packed)) ReceivePacket {
        uint8_t hearder[2] = {'M', 'A'};

        uint16_t crc16;
    };
    // inline ReceivePacket fromVector(const std::vector<uint8_t> &data) {
    //     ReceivePacket packet;
    //     if (data.size() == sizeof(ReceivePacket)) {
    //         std::memcpy(&packet, data.data(), sizeof(ReceivePacket));
    //     }
    //     return packet;
    // }


    //################################################################//
    const uint8_t SOF_SEND = {'M'};
    // Send
    const uint8_t ID_ROBOT_CMD = 0x01;

    const uint8_t DEBUG_PACKAGE_NUM = 10;
    const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

    struct HeaderFrame {
        uint8_t sof;// 数据帧起始字节，固定值为 M
        uint8_t len;// 数据段长度
        uint8_t id; // 数据段id
        uint8_t crc;// 数据帧头的 CRC8 校验
    } __attribute__((packed));

    struct SendRobotCmdData {
        HeaderFrame frame_header;
        uint8_t is_rotate;
        //时间戳（基于下位机运行时间)
        uint32_t time_stamp;

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

}// namespace serial_driver

#endif// RM_SERIAL_DRIVER__PACKET_HPP_
