#ifndef VOYAHSLAM_LOGGING_LOGGER_H
#define VOYAHSLAM_LOGGING_LOGGER_H

#include <memory>
#include <string_view>
#include <vector>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace voyahslam {

// Process-wide logging setup. Every logger made by CreateLogger writes through one shared
// distributing sink, so Initialize may run before or after any component is constructed and
// re-targets all of them. All functions are thread-safe.
class Logger {
public:
    // Replaces the sinks every logger writes to (existing and future loggers). An empty
    // vector restores a single coloured stdout sink. May be called any number of times.
    static void Initialize(const std::vector<spdlog::sink_ptr>& sinks);
    // Level of every registered logger and of loggers created later (default: info).
    static void SetLevel(spdlog::level::level_enum level);
    // The logger called `name`; created on first use, the same object afterwards.
    static std::shared_ptr<spdlog::logger> CreateLogger(std::string_view name);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

private:
    Logger() = default;
};

#define CLASS_LOGGER(className) \
std::shared_ptr<spdlog::logger> log_ = Logger::CreateLogger(#className)

} // namespace voyahslam

#endif // VOYAHSLAM_LOGGING_LOGGER_H
