#ifndef VOYAHSLAM_EXAMPLES_DATASET_READER_PLAYBACK_CLOCK_H
#define VOYAHSLAM_EXAMPLES_DATASET_READER_PLAYBACK_CLOCK_H

#include <chrono>

namespace voyahslam::dataset_reader {

// Replays dataset timestamps in wall-clock time on the caller's thread.
// The first WaitUntil() call anchors dataset time to "now"; every later call
// sleeps until  anchor + (timestamp - first_timestamp) / rate.  Deadlines are
// absolute, so time spent between calls (loading, feature extraction) does not
// accumulate drift; it only matters if it exceeds the gap between frames.
// rate == 0 disables waiting (as fast as possible).
class PlaybackClock {
public:
    using TClock = std::chrono::steady_clock;

    explicit PlaybackClock(double rate = 1.0) : rate_(rate) {}

    // Returns how late we already were for `timestamp` (zero if on time).
    std::chrono::duration<double> WaitUntil(double timestamp);

    [[nodiscard]] double rate() const { return rate_; }

private:
    double rate_;
    bool started_ = false;
    double first_timestamp_ = 0.0;
    TClock::time_point anchor_;
};

} // namespace voyahslam::dataset_reader

#endif // VOYAHSLAM_EXAMPLES_DATASET_READER_PLAYBACK_CLOCK_H
