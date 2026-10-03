#include <utility>
#include <variant>
#include <voyahslam/events/map_created.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam::tracking {
namespace {

const char* ToString(TrackerState state) {
    switch (state) {
        case TrackerState::WaitingForMap: return "WaitingForMap";
        case TrackerState::Tracking: return "Tracking";
        case TrackerState::Lost: return "Lost";
    }
    return "?";
}

template <typename TMeasurement>
double TimestampOf(const TMeasurement& measurement) {
    return std::visit([](const auto& m) { return m.timestamp(); }, measurement);
}

} // namespace

template <typename TFeatureExtractor>
Tracker<TFeatureExtractor>::~Tracker() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Start() {
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Stop() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Track(TMonocularMeasurement measurement) {
    measurements_.enqueue(TMeasurement(std::move(measurement)));
    wake_signal().Notify();
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Track(TBinocularMeasurement measurement) {
    measurements_.enqueue(TMeasurement(std::move(measurement)));
    wake_signal().Notify();
}

template <typename TFeatureExtractor>
bool Tracker<TFeatureExtractor>::ProcessPending() {
    bool processed = false;
    while (events::Event* event = inbox_listener_.GetNextEvent()) {
        processed = true;
        if (AdoptMapOf(*event)) {
            ApplyEvent(*event);
        }
    }

    TMeasurement measurement;
    if (measurements_.try_dequeue(measurement)) {
        processed = true;
        ProcessMeasurement(measurement);
    }
    return processed;
}

template <typename TFeatureExtractor>
bool Tracker<TFeatureExtractor>::AdoptMapOf(const events::Event& event) {
    if (event.map_id == map::MapId{0}) {
        return true;
    }
    const map::MapId current = current_map_id();
    if (event.map_id < current) {
        return false;
    }
    if (event.map_id > current) {
        // The first event of a newer map: the replica (none yet) is dropped here.
        log_->debug("Adopting map {} (was {})", event.map_id.value, current.value);
        current_map_id_.store(event.map_id, std::memory_order_release);
    }
    return true;
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    switch (event.type) {
        case events::EventType::MapCreated: {
            if (state() == TrackerState::Tracking) {
                return;  // duplicate
            }
            const auto& created = static_cast<const events::MapCreated&>(event);
            last_pose_ = created.pose;
            map_frame_id_ = created.frame_id;
            SetState(TrackerState::Tracking);
            return;
        }
        default:
            return;  // replica updates: none kept yet
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::ProcessMeasurement(TMeasurement& measurement) {
    if (state() != TrackerState::Tracking) {
        PublishInitializationMeasurement(measurement);
        return;
    }

    const PoseEstimate estimate = pose_estimator_.Estimate(measurement, last_pose_);
    const double timestamp = TimestampOf(measurement);
    if (estimate.success) {
        last_pose_ = estimate.pose;
        PublishPositionUpdated(timestamp);
    } else {
        PublishTrackingLost(timestamp);
        SetState(TrackerState::Lost);
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::SetState(TrackerState state) {
    const TrackerState previous = state_.exchange(state, std::memory_order_acq_rel);
    log_->info("{} -> {} (map {})", ToString(previous), ToString(state), current_map_id().value);
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::PublishInitializationMeasurement(TMeasurement& measurement) {
    auto* event = initialization_measurement_pool_.Create();
    event->map_id = map::MapId{0};
    event->measurement = std::move(measurement);
    observable_.Notify(event);
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::PublishPositionUpdated(double timestamp) {
    auto* event = position_updated_pool_.Create();
    event->map_id = current_map_id();
    event->frame_id = map_frame_id_;
    event->pose = last_pose_;
    event->timestamp = timestamp;
    observable_.Notify(event);
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::PublishTrackingLost(double timestamp) {
    auto* event = tracking_lost_pool_.Create();
    event->map_id = current_map_id();
    event->timestamp = timestamp;
    observable_.Notify(event);
}

INSTANTIATE_CLASS(Tracker)

} // namespace voyahslam::tracking
