#pragma once

#ifdef __ZEPHYR__
// ── Zephyr logging shim ───────────────────────────────────────────────────────
// Maps host LOG_* macros to Zephyr's structured logging backend.
// Each .cpp that uses these macros must have, before any LOG_* call:
//   LOG_MODULE_REGISTER(name, level)   — exactly one TU per module
//   LOG_MODULE_DECLARE(name, level)    — every other TU in that module
// ─────────────────────────────────────────────────────────────────────────────
#include <zephyr/logging/log.h>

#define LOG_TRACE(...) LOG_DBG(__VA_ARGS__)
#define LOG_DEBUG(...) LOG_DBG(__VA_ARGS__)
#define LOG_INFO(...)  LOG_INF(__VA_ARGS__)
#define LOG_WARN(...)  LOG_WRN(__VA_ARGS__)
#define LOG_ERROR(...) LOG_ERR(__VA_ARGS__)

#else  // ── Host (Linux / macOS / Windows) ────────────────────────────────────

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
        Info  = 2,
        Warn  = 3,
        Error = 4,
        Off   = 5
    };

    // =========================================================================
    // ANSI color helpers (terminal only)
    // =========================================================================
    namespace ansi
    {
        constexpr const char* reset  = "\033[0m";
        constexpr const char* dim    = "\033[2m";
        constexpr const char* red    = "\033[31m";
        constexpr const char* green  = "\033[32m";
        constexpr const char* yellow = "\033[33m";
        constexpr const char* blue   = "\033[34m";
        constexpr const char* cyan   = "\033[36m";
        constexpr const char* bold   = "\033[1m";
    }

    inline const char* to_string(LogLevel lvl)
    {
        switch (lvl)
        {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Error: return "ERROR";
        default:              return "OFF";
        }
    }

    inline const char* to_color(LogLevel lvl)
    {
    #if defined(_WIN32)
        // Disable colors on Windows by default (safe)
        (void)lvl;
        return "";
    #else
        switch (lvl)
        {
        case LogLevel::Trace: return ansi::dim;
        case LogLevel::Debug: return ansi::cyan;
        case LogLevel::Info:  return ansi::green;
        case LogLevel::Warn:  return ansi::yellow;
        case LogLevel::Error: return "\033[1;31m"; // bold red
        default:              return "";
        }
    #endif
    }

    // =========================================================================
    // Global state
    // =========================================================================
    inline LogLevel& global_level()
    {
        static LogLevel lvl = LogLevel::Info;
        return lvl;
    }

    inline std::ofstream& global_log_file()
    {
        static std::ofstream log_file;
        return log_file;
    }

    inline bool& log_to_file_enabled()
    {
        static bool enabled = false;
        return enabled;
    }

    inline void set_level(LogLevel lvl)
    {
        global_level() = lvl;
    }

    inline bool open_log_file(const std::string& path)
    {
        auto& f = global_log_file();
        if (f.is_open())
            f.close();

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
        auto& f = global_log_file();
        if (f.is_open())
            f.close();

        log_to_file_enabled() = false;
    }

    // =========================================================================
    // Core logging
    // =========================================================================
    inline void vlogf(LogLevel lvl, const char* fmt, va_list args)
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
        std::snprintf(ts, sizeof(ts), "%02d:%02d:%02d",
                      tm.tm_hour, tm.tm_min, tm.tm_sec);

        char msg[1024];
        std::vsnprintf(msg, sizeof(msg), fmt, args);

        // ---------- Terminal output (colored)
        const char* color = to_color(lvl);
        std::fprintf(
            stderr,
            "%s[%s] %-5s: %s%s\n",
            color,
            ts,
            to_string(lvl),
            msg,
            ansi::reset
        );

        // ---------- File output (NO color codes)
        if (log_to_file_enabled())
        {
            auto& f = global_log_file();
            if (f.is_open())
            {
                f << "[" << ts << "] "
                  << to_string(lvl) << ": "
                  << msg << "\n";
                f.flush();
            }
        }
    }

    inline void logf(LogLevel lvl, const char* fmt, ...)
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
#define LOG_INFO(...)  ::utils::logf(::utils::LogLevel::Info,  __VA_ARGS__)
#define LOG_WARN(...)  ::utils::logf(::utils::LogLevel::Warn,  __VA_ARGS__)
#define LOG_ERROR(...) ::utils::logf(::utils::LogLevel::Error, __VA_ARGS__)

#endif // __ZEPHYR__
