#include "voyahslam/logging/logger.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace voyahslam {

std::vector<spdlog::sink_ptr> Logger::sinks_;
spdlog::level::level_enum Logger::level_ = spdlog::level::info;

void Logger::Initialize(const std::vector<spdlog::sink_ptr>& sinks) {
    sinks_ = sinks;
    if (sinks_.empty()) {
        sinks_.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }
}

void Logger::SetLevel(spdlog::level::level_enum level) {
    level_ = level;
    spdlog::set_level(level);
}

std::shared_ptr<spdlog::logger> Logger::CreateLogger(std::string_view name) {
    auto log = spdlog::get(std::string(name));
    if (log) {
        return log;
    }

    std::vector<spdlog::sink_ptr> logger_sinks = sinks_;
    if (logger_sinks.empty()) {
        logger_sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }

    log = std::make_shared<spdlog::logger>(std::string(name), logger_sinks.begin(), logger_sinks.end());
    log->set_level(level_);
    spdlog::register_logger(log);

    return log;
}

} // namespace voyahslam
