// //#include"rm_log.hpp"
// int main(){
//     // rmlog::CLogger::GetLogger().InitLogger(
//     //     "app_log",           // 日志文件名前缀
//     //     spdlog::level::debug, // 日志级别 (trace, debug, info, warn, err, critical)
//     //     10,                   // 每个日志文件最大 10MB
//     //     5,                    // 最多保留 5 个历史文件
//     //     rmlog::GetDefaultLogPattern() // 使用默认日志格式
//     // );
//     // rmlog::info("info {}",1);
//     // rmlog::error("error  {}",2);
//     // rmlog::warn("warn {}",3);
    
//     return 0;
// }

#include <glog/logging.h>
#include <glog.hpp>

int main(){
    LOG(INFO) << GREEN << " initialized." << RESET;
    gINFO("dddd");
    int i=0;
    LOG(INFO) << GREEN << " Map init done  "<<i << RESET;
    return 0;
}