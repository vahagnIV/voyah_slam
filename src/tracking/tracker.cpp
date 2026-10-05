#include <cmath>
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

// The monocular measurement matched against the map: the left image of a
// binocular one.
template <typename TFeatureExtractor>
const features::TMonocularMeasurement<TFeatureExtractor>& LeftOf(
    const features::TMeasurement<TFeatureExtractor>& measurement) {
    if (const auto* binocular =
            std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement)) {
        return binocular->left();
    }
    return std::get<features::TMonocularMeasurement<TFeatureExtractor>>(measurement);
}

size_t CountMatches(const std::vector<int64_t>& matches) {
    size_t count = 0;
    for (const int64_t id : matches) {
        count += id >= 0 ? 1 : 0;
    }
    return count;
}

bool IsFinite(const geometry::RigidTransformation& T) {
    return T.rotation().allFinite() && T.translation().allFinite();
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
        // The first event of a newer map: the replica clears itself when it
        // applies it; nothing tracked against the old map carries over.
        log_->debug("Adopting map {} (was {})", event.map_id.value, current.value);
        current_map_id_.store(event.map_id, std::memory_order_release);
        last_frame_ = {};
        velocity_.reset();
        tracked_frames_ = 0;
    }
    return true;
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    replica_.Apply(event);
    if (event.type == events::EventType::MapCreated) {
        OnMapCreated(static_cast<const events::MapCreated&>(event));
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::OnMapCreated(const events::MapCreated& created) {
    if (state() == TrackerState::Tracking) {
        return;  // duplicate
    }
    map_frame_id_ = created.frame_id;
    // The keyframe (published before MapCreated) is the last frame; its
    // observations are its matches.
    last_frame_ = {};
    last_frame_.T_cw = created.pose;
    if (const auto* keyframe = replica_.keyframe(created.frame_id)) {
        last_frame_.measurement = keyframe->measurement;
        last_frame_.T_cw = keyframe->T_cw;
        last_frame_.matches.assign(static_cast<size_t>(LeftOf<TFeatureExtractor>(keyframe->measurement)
                                                           .points()
                                                           .cols()),
                                   -1);
        for (const auto& [feature, point] : keyframe->observations) {
            if (feature < last_frame_.matches.size()) {
                last_frame_.matches[feature] = static_cast<int64_t>(point.value);
            }
        }
    } else {
        log_->warn("MapCreated: keyframe {} of map {} is not in the replica",
                   created.frame_id.value, created.map_id.value);
    }
    velocity_.reset();
    tracked_frames_ = 0;
    SetState(TrackerState::Tracking);
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::ProcessMeasurement(TMeasurement& measurement) {
    if (auto* binocular = std::get_if<TBinocularMeasurement>(&measurement)) {
        binocular->SetStereoMatches(
            features::MatchStereo<TFeatureExtractor>(*binocular, options_.stereo_matcher));
    }

    if (state() != TrackerState::Tracking) {
        PublishInitializationMeasurement(measurement);
        return;
    }

    const double timestamp = TimestampOf(measurement);
    std::optional<PoseResult> pose = TrackFrame(measurement);
    if (!pose) {
        last_frame_ = {};
        velocity_.reset();
        // State first: whoever observes TrackingLost also observes Lost.
        SetState(TrackerState::Lost);
        PublishTrackingLost(timestamp);
        return;
    }

    velocity_ = pose->T_cw * last_frame_.T_cw.Inverse();
    last_frame_.measurement = std::move(measurement);
    last_frame_.T_cw = pose->T_cw;
    last_frame_.matches = std::move(pose->matches);
    ++tracked_frames_;
    PublishPositionUpdated(timestamp);

    const auto now = std::chrono::steady_clock::now();
    if (now - last_info_log_ >= std::chrono::seconds(1)) {
        last_info_log_ = now;
        log_->info("Tracking: frame {}, {} inliers", tracked_frames_, pose->num_inliers);
    }
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::TrackFrame(const TMeasurement& measurement)
    -> std::optional<PoseResult> {
    const TrackingOptions& tracking = options_.tracking;
    const double window = std::holds_alternative<TBinocularMeasurement>(measurement)
                              ? tracking.window_stereo_px
                              : tracking.window_mono_px;
    std::optional<PoseResult> pose;
    if (velocity_) {
        pose = TrackFromLastFrame(measurement, *velocity_ * last_frame_.T_cw, window, "motion model");
    }
    if (!pose) {
        // No BoW reference-keyframe tracking: a wide window at zero motion.
        pose = TrackFromLastFrame(measurement, last_frame_.T_cw,
                                  window * tracking.fallback_window_scale, "zero motion");
    }
    if (!pose) {
        return std::nullopt;
    }
    return TrackLocalMap(measurement, std::move(*pose));
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::TrackFromLastFrame(const TMeasurement& measurement,
                                                    const geometry::RigidTransformation& predicted,
                                                    double window_px, const char* step)
    -> std::optional<PoseResult> {
    const TrackingOptions& tracking = options_.tracking;
    const auto& current = LeftOf<TFeatureExtractor>(measurement);
    const auto& last = LeftOf<TFeatureExtractor>(last_frame_.measurement);
    const auto search = [&](double radius_px) {
        return features::SearchFromLastFrame<TFeatureExtractor>(
            current, predicted, last, last_frame_.matches, replica_, radius_px, options_.matcher);
    };

    std::vector<int64_t> matches = search(window_px);
    size_t num_matches = CountMatches(matches);
    if (num_matches < tracking.min_motion_matches) {
        matches = search(2.0 * window_px);
        num_matches = CountMatches(matches);
    }
    if (num_matches < tracking.min_motion_matches) {
        log_->debug("{} failed: {} matches (< {})", step, num_matches, tracking.min_motion_matches);
        return std::nullopt;
    }

    PoseResult pose = OptimizeAndDropOutliers(measurement, predicted, std::move(matches));
    if (pose.num_inliers < tracking.min_motion_inliers) {
        log_->debug("{} failed: {} matches, {} inliers (< {})", step, pose.num_matches,
                    pose.num_inliers, tracking.min_motion_inliers);
        return std::nullopt;
    }
    log_->debug("{}: {} matches, {} inliers", step, pose.num_matches, pose.num_inliers);
    return pose;
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::TrackLocalMap(const TMeasurement& measurement, PoseResult pose)
    -> std::optional<PoseResult> {
    const size_t added = features::SearchFromLocalMap<TFeatureExtractor>(
        LeftOf<TFeatureExtractor>(measurement), pose.T_cw, replica_, pose.matches,
        options_.tracking.local_map_th, options_.matcher);
    PoseResult result = OptimizeAndDropOutliers(measurement, pose.T_cw, std::move(pose.matches));
    if (result.num_inliers < options_.tracking.min_local_map_inliers) {
        log_->debug("local map failed: {} matches ({} new), {} inliers (< {})", result.num_matches,
                    added, result.num_inliers, options_.tracking.min_local_map_inliers);
        return std::nullopt;
    }
    log_->debug("local map: {} matches ({} new), {} inliers", result.num_matches, added,
                result.num_inliers);
    return result;
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::OptimizeAndDropOutliers(
    const TMeasurement& measurement, const geometry::RigidTransformation& initial_T_cw,
    std::vector<int64_t> matches) const -> PoseResult {
    const auto& left = LeftOf<TFeatureExtractor>(measurement);
    const auto* binocular = std::get_if<TBinocularMeasurement>(&measurement);

    std::vector<optimization::PoseObservation> observations;
    std::vector<size_t> feature_of;  // per observation
    for (size_t i = 0; i < matches.size(); ++i) {
        if (matches[i] < 0) {
            continue;
        }
        const map::MapPointId id{static_cast<uint32_t>(matches[i])};
        if (!replica_.IsAlive(id)) {
            matches[i] = -1;
            continue;
        }
        const auto feature = static_cast<Eigen::Index>(i);
        optimization::PoseObservation observation{
            replica_.Position(id), left.distorted_points().col(feature), std::nullopt,
            options_.sigma_px * std::pow(options_.level_scale, left.levels()(feature))};
        if (binocular) {
            const int right = binocular->right_indices()(feature);
            if (right >= 0) {
                observation.right = binocular->right().distorted_points().col(right);
            }
        }
        observations.push_back(observation);
        feature_of.push_back(i);
    }

    PoseResult pose;
    pose.num_matches = observations.size();
    std::optional<optimization::StereoCameras> stereo;
    if (binocular) {
        stereo = optimization::StereoCameras{binocular->right().camera(), binocular->rig().T_rl};
    }
    const optimization::PoseOptimizationResult result = optimization::OptimizePose(
        initial_T_cw, left.camera(), stereo, observations, options_.pose_optimization);
    if (!result.success || !IsFinite(result.T_cw)) {
        pose.T_cw = initial_T_cw;
        pose.matches.assign(matches.size(), -1);
        return pose;
    }
    for (size_t k = 0; k < observations.size(); ++k) {
        if (!result.inlier[k]) {
            matches[feature_of[k]] = -1;
        }
    }
    pose.T_cw = result.T_cw;
    pose.matches = std::move(matches);
    pose.num_inliers = result.num_inliers;
    return pose;
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
    event->pose = last_frame_.T_cw;
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
