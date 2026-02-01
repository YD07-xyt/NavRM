#pragma once

#ifndef LOG_HPP
#define LOG_HPP
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "spdlog/spdlog.h"
#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
/*
    对日志操作封装：
        1.防止头文件重复包含
        2.包含头文件
        3.声明命名空间
        4.声明全局日志器
        5.声明日志配置结构体
        6.声明全局日志器初始化接口
        7.封装日志输出宏
*/
namespace rmlog {

    //全局日志器
    std::shared_ptr<spdlog::logger> logger();
    //声明日志配置结构体
    struct LogSetting {
        bool async = false;                             //是否异步日志
        int level = 0;                                  //日志级别
        std::string path = "md_log/rm_log_run/test.txt";//日志文件路径
        // 默认带颜色的格式：%^...%$
        std::string format = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v";//日志格式
    };
    //全局日志器初始化
    extern void rmlog_init(const LogSetting& setting);
    extern void rmlog_init();
}// namespace rmlog

#endif