#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <variant>
#include <voyahslam/features/bow_matcher.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam::tracking {
namespace {

const char* ToString(events::TrackingStage stage) {
    switch (stage) {
        case events::TrackingStage::MotionModel: return "motion model";
        case events::TrackingStage::ZeroMotion: return "zero motion";
        case events::TrackingStage::ReferenceFrame: return "reference Frame";
        case events::TrackingStage::Relocalization: return "relocalization";
    }
    return "?";
}

const char* ToString(TrackerState state) {
    switch (state) {
        case TrackerState::WaitingForMap: return "WaitingForMap";
        case TrackerState::Tracking: return "Tracking";
        case TrackerState::RecentlyLost: return "RecentlyLost";
        case TrackerState::Lost: return "Lost";
    }
    return "?";
}

template <typename TMeasurement>
double TimestampOf(const TMeasurement& measurement) {
    return std::visit([](const auto& m) { return m.timestamp(); }, measurement);
}

using features::LeftOf;

size_t CountMatches(const std::vector<int64_t>& matches) {
    size_t count = 0;
    for (const int64_t id : matches) {
        count += id >= 0 ? 1 : 0;
    }
    return count;
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
        reference_frame_id_.reset();
        tracked_frames_ = 0;
    }
    return true;
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    replica_.Apply(event);
    if (event.type == events::EventType::MapTransaction) {
        OnMapTransaction(static_cast<const events::MapTransaction<TFeatureExtractor>&>(event));
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::OnMapTransaction(
    const events::MapTransaction<TFeatureExtractor>& transaction) {
    if (transaction.new_map) {
        if (!transaction.start_frame) {
            log_->warn("New map {} without a start Frame", transaction.map_id.value);
        } else if (state() != TrackerState::Tracking) {  // else a duplicate
            StartTracking(*transaction.start_frame);
        }
    }
    for (const events::NewFrame<TFeatureExtractor>& frame : transaction.added_frames) {
        MergeOwnFrame(frame);
    }
    if (transaction.replaced_points.empty() && transaction.removed_points.empty()) {
        return;
    }
    // Loser -> winner; the Mapper resolves chains, so one lookup suffices.
    std::unordered_map<uint32_t, uint32_t> winners;
    winners.reserve(transaction.replaced_points.size());
    for (const events::ReplacedPoint& replaced : transaction.replaced_points) {
        winners.emplace(replaced.loser.value, replaced.winner.value);
    }
    for (int64_t& match : last_frame_.matches) {
        if (match < 0) continue;
        if (const auto it = winners.find(static_cast<uint32_t>(match)); it != winners.end()) {
            match = static_cast<int64_t>(it->second);
        }
        // The replica has dropped the removed points (and a winner removed in
        // the same transaction).
        if (!replica_.IsAlive(map::MapPointId{static_cast<uint32_t>(match)})) match = -1;
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::StartTracking(map::FrameId start_frame) {
    // The start Frame is the last frame; its observations are its matches.
    last_frame_ = {};
    if (const auto* keyframe = replica_.keyframe(start_frame)) {
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
        log_->warn("New map {}: start Frame {} is not in the replica", current_map_id().value,
                   start_frame.value);
    }
    velocity_.reset();
    reference_frame_id_ = start_frame;
    relocalized_at_.reset();
    tracked_frames_ = 0;
    SetState(TrackerState::Tracking);
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::MergeOwnFrame(const events::NewFrame<TFeatureExtractor>& frame) {
    if (last_frame_.matches.empty() ||
        TimestampOf(frame.measurement) != TimestampOf(last_frame_.measurement)) {
        return;
    }
    const size_t num = std::min(frame.observed_features.size(), frame.observed_points.size());
    for (size_t i = 0; i < num; ++i) {
        const uint32_t feature = frame.observed_features[i];
        if (feature < last_frame_.matches.size()) {
            last_frame_.matches[feature] = static_cast<int64_t>(frame.observed_points[i].value);
        }
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::ProcessMeasurement(TMeasurement& measurement) {
    if (auto* binocular = std::get_if<TBinocularMeasurement>(&measurement)) {
        binocular->SetStereoMatches(
            features::MatchStereo<TFeatureExtractor>(*binocular, options_.stereo_matcher));
    }

    switch (state()) {
        case TrackerState::WaitingForMap:
        case TrackerState::Lost:
            PublishInitializationMeasurement(measurement);
            return;
        case TrackerState::RecentlyLost:
            ProcessRecentlyLost(measurement);
            return;
        case TrackerState::Tracking:
            TrackMeasurement(measurement);
            return;
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::TrackMeasurement(TMeasurement& measurement) {
    std::optional<PoseResult> pose = TrackFrame(measurement);
    if (pose) {
        velocity_ = pose->T_cw * last_frame_.T_cw.Inverse();
        AcceptPose(measurement, std::move(*pose));
        return;
    }

    last_frame_ = {};
    velocity_.reset();
    const double timestamp = TimestampOf(measurement);
    // State first: whoever observes TrackingLost also observes the new state.
    if (vocabulary_ &&
        replica_.num_keyframes() > options_.relocalization.min_map_frames_to_relocalize) {
        recently_lost_since_ = timestamp;
        SetState(TrackerState::RecentlyLost);
        PublishTrackingLost(timestamp, /*final=*/false);
    } else {
        SetState(TrackerState::Lost);
        PublishTrackingLost(timestamp, /*final=*/true);
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::ProcessRecentlyLost(TMeasurement& measurement) {
    if (std::optional<PoseResult> pose = TryRelocalize(measurement)) {
        relocalized_at_ = TimestampOf(measurement);
        velocity_.reset();
        SetState(TrackerState::Tracking);
        AcceptPose(measurement, std::move(*pose));
        return;
    }
    const double timestamp = TimestampOf(measurement);
    if (timestamp - recently_lost_since_ >= options_.relocalization.recently_lost_seconds) {
        SetState(TrackerState::Lost);
        PublishTrackingLost(timestamp, /*final=*/true);
    }
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::AcceptPose(TMeasurement& measurement, PoseResult pose) {
    last_frame_.measurement = std::move(measurement);
    last_frame_.T_cw = pose.T_cw;
    last_frame_.matches = std::move(pose.matches);
    if (pose.reference_frame_id) {
        reference_frame_id_ = pose.reference_frame_id;
    }
    ++tracked_frames_;
    PublishFrameTracked(pose);

    const auto now = std::chrono::steady_clock::now();
    if (now - last_info_log_ >= std::chrono::seconds(1)) {
        last_info_log_ = now;
        log_->info("Tracking: frame {}, {} inliers", tracked_frames_, pose.num_inliers);
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
        pose = TrackFromLastFrame(measurement, *velocity_ * last_frame_.T_cw, window,
                                  events::TrackingStage::MotionModel);
    }
    if (!pose) {
        // A wide window at zero motion.
        pose = TrackFromLastFrame(measurement, last_frame_.T_cw,
                                  window * tracking.fallback_window_scale,
                                  events::TrackingStage::ZeroMotion);
    }
    if (!pose && vocabulary_) {
        pose = TrackReferenceFrame(measurement);
    }
    if (!pose) {
        return std::nullopt;
    }
    return TrackLocalMap(measurement, std::move(*pose));
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::TrackFromLastFrame(const TMeasurement& measurement,
                                                    const geometry::RigidTransformation& predicted,
                                                    double window_px, events::TrackingStage stage)
    -> std::optional<PoseResult> {
    const TrackingOptions& tracking = options_.tracking;
    const char* step = ToString(stage);
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
    const size_t min_inliers = stage == events::TrackingStage::MotionModel
                                   ? tracking.min_motion_model_inliers
                                   : tracking.min_motion_inliers;
    if (pose.num_inliers < min_inliers) {
        log_->debug("{} failed: {} matches, {} inliers (< {})", step, pose.num_matches,
                    pose.num_inliers, min_inliers);
        return std::nullopt;
    }
    log_->debug("{}: {} matches, {} inliers", step, pose.num_matches, pose.num_inliers);
    pose.stage = stage;
    return pose;
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::TrackReferenceFrame(const TMeasurement& measurement)
    -> std::optional<PoseResult> {
    const TrackingOptions& tracking = options_.tracking;
    const auto* reference =
        reference_frame_id_ ? replica_.keyframe(*reference_frame_id_) : nullptr;
    if (reference == nullptr) {
        log_->debug("reference Frame failed: no reference Frame");
        return std::nullopt;
    }
    const auto& current = LeftOf<TFeatureExtractor>(measurement);
    const auto& frame = LeftOf<TFeatureExtractor>(reference->measurement);
    bow::BowVector current_bow;
    bow::FeatureVector current_features;
    // The Mapper's levels: the Frames' and the current frame's FeatureVectors
    // must be comparable.
    vocabulary_->Transform(current.descriptors(), mapping::Mapper<TFeatureExtractor>::kBowLevelsUp,
                           current_bow, current_features);

    // The Frame's live points, per (left) feature.
    std::vector<std::optional<map::MapPointId>> frame_points(
        static_cast<size_t>(frame.points().cols()));
    for (const auto& [feature, point] : reference->observations) {
        if (feature < frame_points.size() && replica_.IsAlive(point)) {
            frame_points[feature] = point;
        }
    }
    features::BowMatchOptions bow_options;
    bow_options.nn_ratio = tracking.reference_frame_nn_ratio;
    const std::vector<std::optional<map::MapPointId>> found = features::SearchByBoW<TFeatureExtractor>(
        frame, reference->features, frame_points, current, current_features, bow_options);

    std::vector<int64_t> matches(found.size(), -1);
    size_t num_matches = 0;
    for (size_t i = 0; i < found.size(); ++i) {
        if (found[i]) {
            matches[i] = static_cast<int64_t>(found[i]->value);
            ++num_matches;
        }
    }
    if (num_matches < tracking.min_reference_frame_matches) {
        log_->debug("reference Frame failed: {} matches (< {})", num_matches,
                    tracking.min_reference_frame_matches);
        return std::nullopt;
    }

    PoseResult pose = OptimizeAndDropOutliers(measurement, last_frame_.T_cw, std::move(matches));
    if (pose.num_inliers < tracking.min_reference_frame_inliers) {
        log_->debug("reference Frame failed: {} matches, {} inliers (< {})", pose.num_matches,
                    pose.num_inliers, tracking.min_reference_frame_inliers);
        return std::nullopt;
    }
    log_->debug("reference Frame: {} matches, {} inliers", pose.num_matches, pose.num_inliers);
    pose.stage = events::TrackingStage::ReferenceFrame;
    return pose;
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::TryRelocalize(const TMeasurement& measurement)
    -> std::optional<PoseResult> {
    if (!vocabulary_) {
        return std::nullopt;
    }
    RelocalizationResult relocalized = Relocalize(replica_, measurement, *vocabulary_,
                                                  options_.relocalization, Refinement(), options_.matcher);
    if (!relocalized.success) {
        log_->debug("relocalization failed: {} candidates, {} with BoW matches, {} hypotheses, best {} inliers",
                    relocalized.num_candidates, relocalized.num_matched_candidates,
                    relocalized.num_hypotheses, relocalized.best_verified_inliers);
        return std::nullopt;
    }
    log_->debug("relocalization: Frame {}, {} inliers", relocalized.frame.value, relocalized.inliers);
    PoseResult pose;
    pose.T_cw = relocalized.T_cw;
    pose.matches = std::move(relocalized.matches);
    pose.num_matches = relocalized.inliers;
    pose.num_inliers = relocalized.inliers;
    pose.stage = events::TrackingStage::Relocalization;
    std::optional<PoseResult> tracked = TrackLocalMap(measurement, std::move(pose));
    if (tracked) {
        tracked->reference_frame_id = relocalized.frame;
    }
    return tracked;
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::TrackLocalMap(const TMeasurement& measurement, PoseResult pose)
    -> std::optional<PoseResult> {
    const TrackingOptions& tracking = options_.tracking;
    LocalMap local = BuildLocalMap(replica_, pose.matches, tracking.local_neighbours,
                                   tracking.max_local_frames);
    std::vector<map::MapPointId> visible;
    const size_t added = features::SearchFromLocalMap<TFeatureExtractor>(
        LeftOf<TFeatureExtractor>(measurement), pose.T_cw,
        LocalPointSource<TFeatureExtractor>(replica_, local), pose.matches, tracking.local_map_th,
        options_.matcher, &visible);
    PoseResult result = OptimizeAndDropOutliers(measurement, pose.T_cw, std::move(pose.matches));
    // Stricter right after a relocalization, the relocalized measurement
    // included (ORB-SLAM3's 50 within mMaxFrames of mnLastRelocFrameId).
    const RelocalizationOptions& relocalization = options_.relocalization;
    const bool after_relocalization =
        pose.stage == events::TrackingStage::Relocalization ||
        (relocalized_at_ &&
         TimestampOf(measurement) - *relocalized_at_ < relocalization.post_relocalization_seconds);
    const size_t min_inliers = after_relocalization ? relocalization.post_relocalization_min_inliers
                                                    : tracking.min_local_map_inliers;
    if (result.num_inliers < min_inliers) {
        log_->debug("local map failed: {} matches ({} new), {} inliers (< {})", result.num_matches,
                    added, result.num_inliers, min_inliers);
        return std::nullopt;
    }
    log_->debug("local map: {} matches ({} new), {} inliers", result.num_matches, added,
                result.num_inliers);
    result.stage = pose.stage;  // the initial pose's source
    result.local_frames = std::move(local.frames);
    result.visible_points = std::move(visible);
    result.reference_frame_id = ReferenceFrame(replica_, result.matches);
    return result;
}

template <typename TFeatureExtractor>
auto Tracker<TFeatureExtractor>::OptimizeAndDropOutliers(
    const TMeasurement& measurement, const geometry::RigidTransformation& initial_T_cw,
    std::vector<int64_t> matches) const -> PoseResult {
    MatchedPose optimized =
        OptimizeMatchedPose(replica_, measurement, initial_T_cw, std::move(matches), Refinement());
    PoseResult pose;
    pose.T_cw = optimized.T_cw;
    pose.matches = std::move(optimized.matches);
    pose.num_matches = optimized.num_matches;
    pose.num_inliers = optimized.num_inliers;
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
    event->priority = false;
    event->measurement = std::move(measurement);
    observable_.Notify(event);
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::PublishFrameTracked(const PoseResult& pose) {
    auto* event = frame_tracked_pool_.Create();
    event->map_id = current_map_id();
    event->priority = false;
    event->timestamp = TimestampOf(last_frame_.measurement);
    event->pose = last_frame_.T_cw;
    event->measurement = last_frame_.measurement;
    for (size_t feature = 0; feature < last_frame_.matches.size(); ++feature) {
        const int64_t point = last_frame_.matches[feature];
        if (point >= 0) {
            event->observed_features.push_back(static_cast<uint32_t>(feature));
            event->observed_points.push_back(map::MapPointId{static_cast<uint32_t>(point)});
        }
    }
    event->visible_points = pose.visible_points;
    event->reference_frame_id = pose.reference_frame_id;
    event->local_frames = pose.local_frames;
    event->stage = pose.stage;
    event->num_matches = static_cast<uint32_t>(pose.num_matches);
    event->num_inliers = static_cast<uint32_t>(pose.num_inliers);
    event->urgent = options_.tracking.IsUrgent(pose.num_inliers);
    event->priority = event->urgent;
    event->map_epoch = replica_.map_epoch();
    observable_.Notify(event);
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::PublishTrackingLost(double timestamp, bool final) {
    auto* event = tracking_lost_pool_.Create();
    event->map_id = current_map_id();
    event->priority = false;
    event->timestamp = timestamp;
    event->final = final;
    observable_.Notify(event);
}

INSTANTIATE_CLASS(Tracker)

} // namespace voyahslam::tracking
