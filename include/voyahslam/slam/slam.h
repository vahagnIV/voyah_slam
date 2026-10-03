#ifndef VOYAHSLAM_SLAM_SLAM_H
#define VOYAHSLAM_SLAM_SLAM_H

#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam::slam {

// Owns the Tracker, Mapper and LoopCloser and wires them asynchronously:
//   Tracker    → Mapper
//   Mapper     → Tracker, LoopCloser
//   LoopCloser → Mapper
// Additional observers (e.g. a visualizer) are attached through tracker(),
// mapper() and loop_closer() before Start(). Not movable: the components hold
// pointers to each other's inboxes.
template <typename TFeatureExtractor>
class Slam {
public:
    using TTracker = tracking::Tracker<TFeatureExtractor>;
    using TMapper = mapping::Mapper<TFeatureExtractor>;
    using TLoopCloser = loop_closing::LoopCloser<TFeatureExtractor>;

    explicit Slam(mapping::MapperOptions mapper_options = {});
    // Stops every component thread before any component is destroyed.
    ~Slam();
    Slam(const Slam&) = delete;
    Slam& operator=(const Slam&) = delete;

    // Starts consumers before producers: Mapper, LoopCloser, Tracker.
    void Start();
    // Stops producers before consumers: Tracker, LoopCloser, Mapper.
    void Stop();

    [[nodiscard]] TTracker& tracker() { return tracker_; }
    [[nodiscard]] TMapper& mapper() { return mapper_; }
    [[nodiscard]] TLoopCloser& loop_closer() { return loop_closer_; }

private:
    TTracker tracker_;
    TMapper mapper_;
    TLoopCloser loop_closer_;
};

} // namespace voyahslam::slam

#endif // VOYAHSLAM_SLAM_SLAM_H
