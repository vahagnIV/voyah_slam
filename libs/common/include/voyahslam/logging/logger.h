#ifndef VOYAHSLAM_LOGGING_LOGGER_H
#define VOYAHSLAM_LOGGING_LOGGER_H

#include <memory>
#include <string_view>
#include <vector>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace voyahslam {

class Logger {
public:
    static void Initialize(const std::vector<spdlog::sink_ptr>& sinks);
    static void SetLevel(spdlog::level::level_enum level);
    static std::shared_ptr<spdlog::logger> CreateLogger(std::string_view name);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

private:
    Logger() = default;
    static std::vector<spdlog::sink_ptr> sinks_;
    static spdlog::level::level_enum level_;
};

#define CLASS_LOGGER(className) \
std::shared_ptr<spdlog::logger> log_ = Logger::CreateLogger(#className)

} // namespace voyahslam

#endif // VOYAHSLAM_LOGGING_LOGGER_H
