#pragma once

#ifndef RM_LOG_HPP
#define RM_LOG_HPP

#include <print>
#include <source_location>
#include <string_view>


namespace rm_log {


enum class Color {
    Reset = 0,
    Black = 30,
    Red = 31,
    Green = 32,
    Yellow = 33,
    Blue = 34,
    Magenta = 35,
    Cyan = 36,
    White = 37,
    BoldRed = 31 | (1 << 8)   // 粗体红色
};

inline void print_color(std::string_view text, Color color) {
    int code = static_cast<int>(color);
    if (code & (1 << 8)) {
        std::print("\033[1;{}m", code & 0xFF);
    } else {
        std::print("\033[{}m", code);
    }
    std::print("{}", text);
    std::print("\033[0m");
}


inline void info(std::string_view message,
                 const std::source_location loc = std::source_location::current()) {
    // 输出位置信息（灰色）
    std::print("\033[2m{}:{}:\033[0m ", loc.file_name(), loc.line());
    // 输出带颜色的 [INFO] 和消息
    print_color("[INFO] ", Color::Green);
    std::println("{}", message);
}


inline void debug(std::string_view message,
                 const std::source_location loc = std::source_location::current()) {
    // 输出位置信息（灰色）
    std::print("\033[2m{}:{}:\033[0m ", loc.file_name(), loc.line());
    // 输出带颜色的 [INFO] 和消息
    print_color("[DEHUG] ", Color::Blue);
    std::println("{}", message);
}


inline void warn(std::string_view message,
                 const std::source_location loc = std::source_location::current()) {
    // 输出位置信息（灰色）
    std::print("\033[2m{}:{}:\033[0m ", loc.file_name(), loc.line());
    // 输出带颜色的 [INFO] 和消息
    print_color("[WARN] ", Color::Yellow);
    std::println("{}", message);
}


inline void error(std::string_view message,
                 const std::source_location loc = std::source_location::current()) {
    // 输出位置信息（灰色）
    std::print("\033[2m{}:{}:\033[0m ", loc.file_name(), loc.line());
    // 输出带颜色的 [INFO] 和消息
    print_color("[ERROR] ", Color::BoldRed);
    std::println("{}", message);
}

} // namespace rm_log

#endif