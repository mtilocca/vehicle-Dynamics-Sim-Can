#pragma once
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <string>

namespace utils
{

    enum class LogLevel : int
    {
        Trace = 0,
        Debug = 1,
        Info = 2,
        Warn = 3,
        Error = 4,
        Off = 5
    };

    inline const char *to_string(LogLevel lvl)
    {
        switch (lvl)
        {
        case LogLevel::Trace:
            return "TRACE";
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warn:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
        default:
            return "OFF";
        }
    }

    // Global state
    inline LogLevel &global_level()
    {
        static LogLevel lvl = LogLevel::Info;
        return lvl;
    }

    inline std::ofstream &global_log_file()
    {
        static std::ofstream log_file;
        return log_file;
    }

    inline bool &log_to_file_enabled()
    {
        static bool enabled = false;
        return enabled;
    }

    inline void set_level(LogLevel lvl)
    {
        global_level() = lvl;
    }

    inline bool open_log_file(const std::string &path)
    {
        auto &f = global_log_file();
        if (f.is_open())
        {
            f.close();
        }

        f.open(path, std::ios::out | std::ios::trunc);
        if (!f.is_open())
        {
            std::fprintf(stderr, "[ERROR] Failed to open log file: %s\n", path.c_str());
            return false;
        }

        log_to_file_enabled() = true;
        std::fprintf(stderr, "[INFO] Logging to file: %s\n", path.c_str());
        return true;
    }

    inline void close_log_file()
    {
        auto &f = global_log_file();
        if (f.is_open())
        {
            f.close();
        }
        log_to_file_enabled() = false;
    }

    inline void vlogf(LogLevel lvl, const char *fmt, va_list args)
    {
        if (lvl < global_level() || global_level() == LogLevel::Off)
            return;

        // Timestamp
        std::time_t t = std::time(nullptr);
        std::tm tm{};
        #if defined(_WIN32)
        localtime_s(&tm, &t);
        #else
        localtime_r(&t, &tm);
        #endif

        char ts[32];
        std::snprintf(ts, sizeof(ts), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

        char msg[1024];
        std::vsnprintf(msg, sizeof(msg), fmt, args);

        char log_line[1200];
        std::snprintf(log_line, sizeof(log_line), "[%s] %-5s: %s\n", ts, to_string(lvl), msg); // ← FIXED: Build once

        // Always log to stderr
        std::fprintf(stderr, "%s", log_line);

        // Also log to file if enabled
        if (log_to_file_enabled())
        {
            auto &f = global_log_file();
            if (f.is_open())
            {
                f << log_line;
                f.flush();
            }
        }
    }

    inline void logf(LogLevel lvl, const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        vlogf(lvl, fmt, args);
        va_end(args);
    }

} // namespace utils

// Convenience macros
#define LOG_TRACE(...) ::utils::logf(::utils::LogLevel::Trace, __VA_ARGS__)
#define LOG_DEBUG(...) ::utils::logf(::utils::LogLevel::Debug, __VA_ARGS__)
#define LOG_INFO(...) ::utils::logf(::utils::LogLevel::Info, __VA_ARGS__)
#define LOG_WARN(...) ::utils::logf(::utils::LogLevel::Warn, __VA_ARGS__)
#define LOG_ERROR(...) ::utils::logf(::utils::LogLevel::Error, __VA_ARGS__)