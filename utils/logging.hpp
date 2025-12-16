#pragma once
#include <cstdarg>
#include <cstdio>
#include <ctime>

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

    inline LogLevel &global_level()
    {
        static LogLevel lvl = LogLevel::Info;
        return lvl;
    }

    inline void set_level(LogLevel lvl)
    {
        global_level() = lvl;
    }

    inline void vlogf(LogLevel lvl, const char *fmt, va_list args)
    {
        if (lvl < global_level() || global_level() == LogLevel::Off)
            return;

        // Timestamp (local time)
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif

        char ts[32];
        std::snprintf(ts, sizeof(ts), "%02d:%02d:%02d",
                      tm.tm_hour, tm.tm_min, tm.tm_sec);

        std::fprintf(stderr, "[%s] %-5s: ", ts, to_string(lvl));
        std::vfprintf(stderr, fmt, args);
        std::fprintf(stderr, "\n");
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