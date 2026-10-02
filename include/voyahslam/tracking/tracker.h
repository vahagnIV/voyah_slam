#ifndef VOYAHSLAM_TRACKING_TRACKER_H
#define VOYAHSLAM_TRACKING_TRACKER_H

#include <concurrentqueue.h>
#include <cstddef>
#include <variant>
#include <voyahslam/events/event_node.h>
#include <voyahslam/features/binocular_measurement.h>
#include <voyahslam/features/monocular_measurement.h>

namespace voyahslam::tracking {

// Estimates the camera pose of every incoming measurement against its own
// replica of the map. Runs on its own thread with two inputs:
//   - inbox():        map diffs from the Mapper,
//   - measurements_:  monocular / binocular measurements from Track().
// Each loop iteration first applies all pending map diffs, then tracks a
// measurement, so the replica is never modified while it is being tracked
// against. Publishes PositionUpdated and NewKeyFrameCandidate; never modifies
// map objects.
template <typename TFeatureExtractor>
class Tracker : public events::EventNode {
public:
    using TMonocularMeasurement =
        features::MonocularMeasurement<typename TFeatureExtractor::TDescriptorScalarType,
                                       TFeatureExtractor::DescriptorSize>;
    using TBinocularMeasurement =
        features::BinocularMeasurement<typename TFeatureExtractor::TDescriptorScalarType,
                                       TFeatureExtractor::DescriptorSize>;

    void Start();
    void Stop();

    // Enqueue the measurement for the Tracker thread and return immediately.
    // Taken by value: pass an rvalue to move the feature matrices instead of
    // copying them.
    void Track(TMonocularMeasurement measurement);
    void Track(TBinocularMeasurement measurement);

    // Approximate number of measurements waiting to be tracked.
    [[nodiscard]] size_t pending_measurements() const { return measurements_.size_approx(); }

private:
    using TMeasurement = std::variant<TMonocularMeasurement, TBinocularMeasurement>;

    moodycamel::ConcurrentQueue<TMeasurement> measurements_;
};

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_TRACKER_H
