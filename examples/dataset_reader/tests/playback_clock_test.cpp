#include <dataset_reader/playback_clock.h>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace voyahslam::dataset_reader {
namespace {

using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

TEST(PlaybackClockTest, WaitsUntilScaledDatasetTime) {
    PlaybackClock clock(2.0);  // double speed
    clock.WaitUntil(100.0);
    auto start = steady_clock::now();
    auto lateness = clock.WaitUntil(100.1);  // 0.1 s of dataset time = 50 ms
    duration<double> waited = steady_clock::now() - start;
    EXPECT_EQ(lateness.count(), 0.0);
    EXPECT_GE(waited.count(), 0.045);
    EXPECT_LT(waited.count(), 0.090);
}

TEST(PlaybackClockTest, ReportsLatenessInsteadOfWaiting) {
    PlaybackClock clock(1.0);
    clock.WaitUntil(5.0);
    std::this_thread::sleep_for(milliseconds(40));
    auto start = steady_clock::now();
    auto lateness = clock.WaitUntil(5.01);  // deadline was 10 ms after anchor
    duration<double> waited = steady_clock::now() - start;
    EXPECT_GE(lateness.count(), 0.025);
    EXPECT_LT(waited.count(), 0.005);
}

TEST(PlaybackClockTest, DeadlinesAreAbsoluteSoWorkDoesNotAccumulate) {
    PlaybackClock clock(1.0);
    auto start = steady_clock::now();
    for (int i = 0; i <= 5; ++i) {
        clock.WaitUntil(i * 0.02);
        std::this_thread::sleep_for(milliseconds(10));  // "processing" < frame gap
    }
    duration<double> total = steady_clock::now() - start;
    // 5 gaps of 20 ms + the last 10 ms of work; drift would add 5 × 10 ms.
    EXPECT_LT(total.count(), 0.140);
}

TEST(PlaybackClockTest, RateZeroNeverWaits) {
    PlaybackClock clock(0.0);
    clock.WaitUntil(0.0);
    auto start = steady_clock::now();
    clock.WaitUntil(1000.0);
    duration<double> waited = steady_clock::now() - start;
    EXPECT_LT(waited.count(), 0.005);
}

} // namespace
} // namespace voyahslam::dataset_reader
