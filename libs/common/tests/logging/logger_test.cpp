#include <voyahslam/logging/logger.h>

#include <spdlog/sinks/ringbuffer_sink.h>

#include <gtest/gtest.h>

#include <memory>

namespace voyahslam {
namespace {

// A component that owns its logger the way every library class does.
struct LoggerTestComponent {
    CLASS_LOGGER(LoggerTestComponent);
    void Say(const char* text) { log_->info("{}", text); }
};

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> MakeCapture() {
    return std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(64);
}

bool Contains(const std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt>& capture, const std::string& text) {
    for (const auto& line : capture->last_formatted(64)) {
        if (line.find(text) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// Audit #8: a component built before Initialize must log to the sinks given to Initialize.
TEST(LoggerTest, ComponentCreatedBeforeInitializeUsesNewSinks) {
    Logger::Initialize({});
    LoggerTestComponent component;  // constructed first
    auto capture = MakeCapture();
    Logger::Initialize({capture});
    component.Say("after-initialize");
    EXPECT_TRUE(Contains(capture, "after-initialize"));
    EXPECT_TRUE(Contains(capture, "LoggerTestComponent"));
    Logger::Initialize({});
}

TEST(LoggerTest, InitializeTwiceRetargets) {
    LoggerTestComponent component;
    auto first = MakeCapture();
    auto second = MakeCapture();
    Logger::Initialize({first});
    component.Say("to-first");
    Logger::Initialize({second});
    component.Say("to-second");
    EXPECT_TRUE(Contains(first, "to-first"));
    EXPECT_FALSE(Contains(first, "to-second"));
    EXPECT_TRUE(Contains(second, "to-second"));
    EXPECT_FALSE(Contains(second, "to-first"));
    Logger::Initialize({});
}

TEST(LoggerTest, InitializeEmptyRestoresDefault) {
    LoggerTestComponent component;
    auto capture = MakeCapture();
    Logger::Initialize({capture});
    component.Say("captured");
    Logger::Initialize({});
    component.Say("not-captured");  // goes to the default stdout sink; must not crash
    EXPECT_TRUE(Contains(capture, "captured"));
    EXPECT_FALSE(Contains(capture, "not-captured"));
}

}  // namespace
}  // namespace voyahslam
