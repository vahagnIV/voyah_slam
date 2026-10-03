#include <dataset_reader/playback_clock.h>
#include <thread>

namespace voyahslam::dataset_reader {

std::chrono::duration<double> PlaybackClock::WaitUntil(double timestamp) {
    TClock::time_point now = TClock::now();
    if (!started_) {
        started_ = true;
        first_timestamp_ = timestamp;
        anchor_ = now;
        return std::chrono::duration<double>::zero();
    }
    if (rate_ <= 0.0) {
        return std::chrono::duration<double>::zero();
    }
    auto deadline = anchor_ + std::chrono::duration_cast<TClock::duration>(
                                  std::chrono::duration<double>((timestamp - first_timestamp_) / rate_));
    if (now >= deadline) {
        return now - deadline;
    }
    std::this_thread::sleep_until(deadline);
    return std::chrono::duration<double>::zero();
}

} // namespace voyahslam::dataset_reader
