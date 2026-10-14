#include <voyahslam/logging/logger.h>

#include <spdlog/sinks/ringbuffer_sink.h>

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

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

TEST(LoggerTest, SetLevelAppliesToExistingLoggers) {
    Logger::SetLevel(spdlog::level::info);
    auto before = Logger::CreateLogger("LoggerTestLevelBefore");
    EXPECT_EQ(before->level(), spdlog::level::info);
    Logger::SetLevel(spdlog::level::debug);
    EXPECT_EQ(before->level(), spdlog::level::debug);
    auto after = Logger::CreateLogger("LoggerTestLevelAfter");
    EXPECT_EQ(after->level(), spdlog::level::debug);
    Logger::Initialize({});
    Logger::SetLevel(spdlog::level::info);
}

// register_logger throws on a duplicate name; creation must be serialized.
TEST(LoggerTest, ConcurrentCreateOfOneNameReturnsOneLogger) {
    constexpr int kThreads = 8;
    std::vector<std::shared_ptr<spdlog::logger>> got(kThreads);
    std::atomic<int> ready{0};
    std::atomic<int> thrown{0};
    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; ++i) {
        threads.emplace_back([&, i] {
            ready.fetch_add(1);
            while (ready.load() < kThreads) {
            }
            try {
                got[i] = Logger::CreateLogger("LoggerTestRace");
            } catch (...) {
                thrown.fetch_add(1);
            }
        });
    }
    for (auto& t : threads) {
        t.join();
    }
    EXPECT_EQ(thrown.load(), 0);
    for (int i = 0; i < kThreads; ++i) {
        ASSERT_NE(got[i], nullptr);
        EXPECT_EQ(got[i], got[0]);
    }
}

TEST(LoggerTest, ConcurrentLogAndInitialize) {
    auto capture_a = MakeCapture();
    auto capture_b = MakeCapture();
    LoggerTestComponent component;
    std::atomic<bool> stop{false};
    std::vector<std::thread> loggers;
    for (int t = 0; t < 4; ++t) {
        loggers.emplace_back([&] {
            for (int i = 0; i < 1000; ++i) {
                component.Say("concurrent");
            }
        });
    }
    std::thread switcher([&] {
        for (int i = 0; i < 100; ++i) {
            Logger::Initialize(i % 2 == 0 ? std::vector<spdlog::sink_ptr>{capture_a}
                                          : std::vector<spdlog::sink_ptr>{capture_a, capture_b});
        }
        stop.store(true);
    });
    for (auto& t : loggers) {
        t.join();
    }
    switcher.join();
    EXPECT_TRUE(stop.load());
    Logger::Initialize({});
}

}  // namespace
}  // namespace voyahslam
