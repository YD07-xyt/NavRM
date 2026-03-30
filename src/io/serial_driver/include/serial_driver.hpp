#pragma once


#ifndef SERIAL_HPP
#define SERIAL_HPP
#include<string>
#include <boost/asio/buffer.hpp>
#include <cstdint>
#include <memory>
#include <sys/types.h>
#include<boost/asio.hpp>
#include<boost/circular_buffer.hpp>
namespace io::serial {
        struct SerialConfig{
            std::string device_name;
            int baud_rate;
            std::string flow_control;
            std::string parity;
            std::string stop_bits;
        };

        class SerialDriver{
            public:
                SerialDriver();
                ~SerialDriver();
                size_t send_data(const std::vector<uint8_t>& send_data);
                void reopen_serial();
                std::vector<uint8_t> receive_data();
                void open();
                void close();
            private:
                SerialConfig serialconfig;
            private:
               void get_param();
            private:
                //串口
                std::shared_ptr<boost::asio::serial_port> serial_port_;
                boost::asio::io_service io_service_;
                //环形缓存区 
                boost::circular_buffer<uint8_t, std::allocator<uint8_t>> circular_buffer; 
                std::vector<uint8_t> receive_buffer;
        };        
};

#endif