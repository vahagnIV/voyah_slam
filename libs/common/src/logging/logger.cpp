#include "voyahslam/logging/logger.h"

#include <atomic>
#include <mutex>

#include "spdlog/sinks/dist_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace voyahslam {
namespace {

// The one sink every logger writes through. Function-local statics make the setup independent
// of static initialisation order (a component may be constructed by another static's
// initialiser). Loggers hold the sink by shared_ptr, so it outlives them.
const std::shared_ptr<spdlog::sinks::dist_sink_mt>& SharedSink() {
    static const auto sink = [] {
        auto dist = std::make_shared<spdlog::sinks::dist_sink_mt>();
        dist->add_sink(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        return dist;
    }();
    return sink;
}

std::atomic<spdlog::level::level_enum>& CurrentLevel() {
    static std::atomic<spdlog::level::level_enum> level{spdlog::level::info};
    return level;
}

}  // namespace

void Logger::Initialize(const std::vector<spdlog::sink_ptr>& sinks) {
    if (sinks.empty()) {
        SharedSink()->set_sinks({std::make_shared<spdlog::sinks::stdout_color_sink_mt>()});
    } else {
        SharedSink()->set_sinks(sinks);
    }
}

void Logger::SetLevel(spdlog::level::level_enum level) {
    CurrentLevel().store(level);
    spdlog::set_level(level);
}

std::shared_ptr<spdlog::logger> Logger::CreateLogger(std::string_view name) {
    auto log = spdlog::get(std::string(name));
    if (log) {
        return log;
    }

    log = std::make_shared<spdlog::logger>(std::string(name), SharedSink());
    log->set_level(CurrentLevel().load());
    spdlog::register_logger(log);

    return log;
}

}  // namespace voyahslam
