#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <variant>
#include <vector>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/map/point_statistics.h>
#include <voyahslam/mapping/mapper.h>
#include "mapping/mapper_internal.h"

namespace voyahslam::mapping {
namespace {

const char* ToString(MapperState state) {
    switch (state) {
        case MapperState::Initializing: return "Initializing";
        case MapperState::Mapping: return "Mapping";
    }
    return "?";
}

using detail::LeftOf;
using detail::RigBaseline;

using Clock = std::chrono::steady_clock;
using detail::MillisecondsSince;

template <typename TFeatureExtractor>
double TimestampOf(const features::TMeasurement<TFeatureExtractor>& measurement) {
    return std::visit([](const auto& m) { return m.timestamp(); }, measurement);
}

} // namespace

template <typename TFeatureExtractor>
Mapper<TFeatureExtractor>::Mapper(MapperOptions options,
                                  std::shared_ptr<const TVocabulary> vocabulary)
    : options_(options)
    , vocabulary_(std::move(vocabulary))
    , initializer_(std::move(options.initializer)) {}

template <typename TFeatureExtractor>
Mapper<TFeatureExtractor>::~Mapper() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Start() {
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Stop() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
bool Mapper<TFeatureExtractor>::ProcessPending() {
    // The whole batch is drained first, so a FrameTracked can see whether a
    // newer one follows it (drain to latest, spec section 3.1).
    events::Event* event = nullptr;
    while (inbox_.TryPop(event)) {
        batch_.push_back(event);
    }
    if (batch_.empty()) {
        return false;
    }
    MarkSupersededFrameTracked();
    for (size_t i = 0; i < batch_.size(); ++i) {
        if (superseded_[i]) {
            SkipFrameTracked(*batch_[i]);
        } else {
            ApplyEvent(*batch_[i]);
        }
        batch_[i]->recycle_(batch_[i]);  // acknowledges the event
    }
    batch_.clear();
    return true;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::MarkSupersededFrameTracked() {
    superseded_.assign(batch_.size(), false);
    // Maps with a FrameTracked or final TrackingLost later in the batch; a
    // batch rarely spans more than one or two maps. A non-final TrackingLost
    // keeps the map, so it supersedes nothing.
    std::vector<map::MapId> later;
    for (size_t i = batch_.size(); i-- > 0;) {
        const events::Event& event = *batch_[i];
        const bool final_lost = event.type == events::EventType::TrackingLost &&
                                static_cast<const events::TrackingLost&>(event).final;
        if (event.type != events::EventType::FrameTracked && !final_lost) {
            continue;
        }
        const bool seen = std::find(later.begin(), later.end(), event.map_id) != later.end();
        if (event.type == events::EventType::FrameTracked) {
            superseded_[i] = seen;
        }
        if (!seen) {
            later.push_back(event.map_id);
        }
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    switch (event.type) {
        case events::EventType::InitializationMeasurement:
            OnInitializationMeasurement(
                static_cast<const events::InitializationMeasurement<TFeatureExtractor>&>(event));
            return;
        case events::EventType::FrameTracked:
            OnFrameTracked(static_cast<const events::FrameTracked<TFeatureExtractor>&>(event));
            return;
        case events::EventType::TrackingLost:
            OnTrackingLost(static_cast<const events::TrackingLost&>(event));
            return;
        case events::EventType::LoopDetected:
            OnLoopDetected(static_cast<const events::LoopDetected&>(event));
            return;
        case events::EventType::GlobalBAFinished:
            OnGlobalBaFinished(static_cast<const events::GlobalBAFinished&>(event));
            return;
        default:
            return;
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::SkipFrameTracked(const events::Event& event) {
    if (state() != MapperState::Mapping || event.map_id != current_map_id()) {
        return;  // would not have been evaluated anyway
    }
    ++backlog_;
    backlog_skipped_.fetch_add(1, std::memory_order_acq_rel);
    CountTrackedPoints(static_cast<const events::FrameTracked<TFeatureExtractor>&>(event));
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::CountTrackedPoints(
    const events::FrameTracked<TFeatureExtractor>& event) {
    if (event.stage == events::TrackingStage::Relocalization) {
        relocalized_at_ = event.timestamp;
    }
    // Ids of a replica that is behind may be unknown here: skipped.
    for (const map::MapPointId point : event.visible_points) {
        if (point.value < num_visible_.size()) {
            ++num_visible_[point.value];
        }
    }
    for (const map::MapPointId point : event.observed_points) {
        if (point.value < num_found_.size()) {
            ++num_found_[point.value];
        }
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnInitializationMeasurement(
    const events::InitializationMeasurement<TFeatureExtractor>& event) {
    if (state() != MapperState::Initializing) {
        return;
    }
    // A copy: the event is shared with the other consumers.
    auto initial = initializer_.TryInitialize(TMeasurement(event.measurement));
    if (!initial || initial->keyframes.empty()) {
        return;
    }

    for (auto& keyframe : initial->keyframes) {
        // The initial Frames' measurements come from InitializationMeasurement
        // events, not from FrameTracked: they become shared here, once.
        AddKeyFrame(next_frame_id_, keyframe.T_cw,
                    std::make_shared<const TMeasurement>(std::move(keyframe.measurement)));
        ++next_frame_id_.value;
    }
    map_points_ = {};
    covisibility_.Clear();
    references_.clear();
    num_visible_.clear();
    num_found_.clear();
    recent_points_.clear();
    AddInitialMapPoints(initial->map_points);
    for (size_t k = 1; k < keyframes_.size(); ++k) {
        ConnectToParent(keyframes_[k]->frame.id());
    }
    map_epoch_ = 0;
    num_keyframes_.store(keyframes_.size(), std::memory_order_release);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    current_map_id_.store(next_map_id_, std::memory_order_release);
    ++next_map_id_.value;

    // The state changes before the transaction that announces it is
    // published, so anyone who observes the new map also observes
    // state() == Mapping.
    SetState(MapperState::Mapping);
    PublishNewMap();
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnTrackingLost(const events::TrackingLost& event) {
    if (!event.final) {
        return;  // recently lost: the Tracker relocalizes in this map
    }
    if (event.map_id != current_map_id() || state() != MapperState::Mapping) {
        return;  // stale: an older map
    }
    // The map is discarded: one transaction drops it from every replica's
    // Atlas and leaves none active until the next new map.
    auto* transaction = CreateTransaction();
    transaction->removed_maps.push_back(current_map_id());
    transaction->active_map.reset();
    ClearMap();
    initializer_.Reset();
    SetState(MapperState::Initializing);
    observable_.Notify(transaction);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ClearMap() {
    keyframes_.clear();
    map_points_ = {};
    covisibility_.Clear();
    references_.clear();
    num_visible_.clear();
    num_found_.clear();
    recent_points_.clear();
    backlog_ = 0;
    relocalized_at_.reset();
    map_epoch_ = 0;
    num_keyframes_.store(0, std::memory_order_release);
    num_map_points_.store(0, std::memory_order_release);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnFrameTracked(
    const events::FrameTracked<TFeatureExtractor>& event) {
    if (state() != MapperState::Mapping || event.map_id != current_map_id()) {
        return;  // no map yet, or an older map's
    }
    CountTrackedPoints(event);
    if (event.map_epoch < map_epoch_) {
        // Tracked against the map before a loop correction: its pose and
        // observations are of the uncorrected map (loop-closing spec section 3).
        log_->debug("FrameTracked t={:.3f}: epoch {} before the map's {} -> no Frame", event.timestamp,
                    event.map_epoch, map_epoch_);
        stale_epoch_skipped_.fetch_add(1, std::memory_order_acq_rel);
        return;
    }
    const KeyFrameSignals signals = MeasureSignals(event);
    const bool guarded = InRelocalizationGuard(event);
    const bool create = !guarded && ShouldCreateFrame(signals, options_.keyframe);
    if (guarded) {
        log_->debug("FrameTracked t={:.3f}: {:.3f} s after the relocalization at t={:.3f} -> no Frame",
                    event.timestamp, event.timestamp - *relocalized_at_, *relocalized_at_);
    }
    log_->debug(
        "FrameTracked t={:.3f}: stereo={} frames={} ref_points={} since_last={:.3f} backlog={} "
        "inliers={} close_tracked={} close_untracked={} -> {}",
        event.timestamp, signals.stereo, signals.num_frames, signals.ref_points,
        signals.since_last, signals.backlog, signals.inliers, signals.close_tracked,
        signals.close_untracked, create ? "create Frame" : "no Frame");
    backlog_ = 0;
    if (create) {
        CreateFrame(event);
    }
    // Counted once the Frame it created, if any, is published and refined.
    num_frame_evaluations_.fetch_add(1, std::memory_order_acq_rel);
}

template <typename TFeatureExtractor>
bool Mapper<TFeatureExtractor>::InRelocalizationGuard(
    const events::FrameTracked<TFeatureExtractor>& event) const {
    const KeyFrameOptions& options = options_.keyframe;
    return relocalized_at_ &&
           num_keyframes_.load(std::memory_order_relaxed) > options.relocalization_guard_min_frames &&
           event.timestamp - *relocalized_at_ < options.relocalization_guard_seconds;
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::AddKeyFrame(map::FrameId id,
                                            const geometry::RigidTransformation& pose,
                                            TSharedMeasurement measurement) -> const TKeyFrame& {
    KeyFrameRecord& record =
        *keyframes_.emplace_back(KeyFrameRecord{TKeyFrame(id, pose, std::move(measurement)), {}, {}});
    if (vocabulary_) {
        const auto start = Clock::now();
        vocabulary_->Transform(LeftOf<TFeatureExtractor>(record.frame.measurement()).descriptors(),
                               kBowLevelsUp, record.bow, record.features);
        log_->debug("Frame {}: BoW {:.2f} ms ({} words, {} nodes)", id.value,
                    MillisecondsSince(start), record.bow.entries.size(),
                    record.features.nodes.size());
    }
    return record.frame;
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::FindRecord(map::FrameId id) const -> const KeyFrameRecord* {
    // A map's Frame ids are consecutive from its first Frame's, which is
    // never culled.
    if (keyframes_.empty() || id < FirstKeyFrame().id()) {
        return nullptr;
    }
    const size_t index = id.value - FirstKeyFrame().id().value;
    if (index >= keyframes_.size() || !keyframes_[index]) {
        return nullptr;
    }
    assert(keyframes_[index]->frame.id() == id);
    return &*keyframes_[index];
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::FindRecord(map::FrameId id) -> KeyFrameRecord* {
    return const_cast<KeyFrameRecord*>(std::as_const(*this).FindRecord(id));
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::FindKeyFrame(map::FrameId id) const -> const TKeyFrame* {
    const KeyFrameRecord* record = FindRecord(id);
    return record != nullptr ? &record->frame : nullptr;
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::FindKeyFrame(map::FrameId id) -> TKeyFrame* {
    return const_cast<TKeyFrame*>(std::as_const(*this).FindKeyFrame(id));
}

template <typename TFeatureExtractor>
const bow::FeatureVector* Mapper<TFeatureExtractor>::BowFeaturesOf(map::FrameId id) const {
    if (!vocabulary_) {
        return nullptr;
    }
    // A Frame not in the map (never the case today): brute force.
    const KeyFrameRecord* record = FindRecord(id);
    assert(record != nullptr);
    return record != nullptr ? &record->features : nullptr;
}

template <typename TFeatureExtractor>
const bow::BowVector* Mapper<TFeatureExtractor>::BowVectorOf(map::FrameId frame) const {
    const KeyFrameRecord* record = vocabulary_ ? FindRecord(frame) : nullptr;
    return record != nullptr ? &record->bow : nullptr;
}

template <typename TFeatureExtractor>
std::optional<map::FrameId> Mapper<TFeatureExtractor>::Parent(map::FrameId frame) const {
    const KeyFrameRecord* record = FindRecord(frame);
    return record != nullptr ? record->parent : std::nullopt;
}

template <typename TFeatureExtractor>
std::set<map::FrameId> Mapper<TFeatureExtractor>::LoopEdges(map::FrameId frame) const {
    const KeyFrameRecord* record = FindRecord(frame);
    return record != nullptr ? record->loop_edges : std::set<map::FrameId>{};
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::CreateTransaction() -> events::MapTransaction<TFeatureExtractor>* {
    auto* transaction = transaction_pool_.Create();
    transaction->map_id = current_map_id();
    transaction->priority = false;
    transaction->map_epoch = map_epoch_;
    // The Atlas: the map being built is the active one.
    transaction->active_map = current_map_id();
    return transaction;
}

template <typename TFeatureExtractor>
map::FrameId Mapper<TFeatureExtractor>::ConnectToParent(map::FrameId frame) {
    KeyFrameRecord& record = *FindRecord(frame);
    assert(&record == &*keyframes_.back() && keyframes_.size() > 1);
    // Sorted by shared points descending, then by id.
    const auto covisible = covisibility_.Covisible(frame, 1);
    map::FrameId parent{};
    if (!covisible.empty()) {
        parent = covisible.front().first;
    } else {
        // The previous newest Frame is never culled before this one exists.
        const auto previous = std::find_if(std::next(keyframes_.rbegin()), keyframes_.rend(),
                                           [](const auto& other) { return other.has_value(); });
        assert(previous != keyframes_.rend());
        parent = (*previous)->frame.id();
    }
    record.parent = parent;
    FindRecord(parent)->children.insert(frame);
    return parent;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ChangeParent(map::FrameId frame, map::FrameId parent,
                                             events::MapTransaction<TFeatureExtractor>& out_event) {
    KeyFrameRecord& record = *FindRecord(frame);
    if (record.parent) {
        FindRecord(*record.parent)->children.erase(frame);
    }
    record.parent = parent;
    FindRecord(parent)->children.insert(frame);
    out_event.parent_changes.emplace_back(frame, parent);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::AddLoopEdge(map::FrameId a, map::FrameId b,
                                            events::MapTransaction<TFeatureExtractor>& out_event) {
    KeyFrameRecord* record_a = FindRecord(a);
    KeyFrameRecord* record_b = FindRecord(b);
    assert(record_a != nullptr && record_b != nullptr && a != b);
    record_a->loop_edges.insert(b);
    record_b->loop_edges.insert(a);
    out_event.added_loop_edges.emplace_back(a, b);
}

template <typename TFeatureExtractor>
size_t Mapper<TFeatureExtractor>::WeightedObservations(map::MapPointId point) const {
    size_t count = 0;
    for (const map::PointObservation& observation : covisibility_.Observations(point)) {
        const TKeyFrame* keyframe = FindKeyFrame(observation.frame);
        assert(keyframe != nullptr);
        const auto* binocular =
            std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&keyframe->measurement());
        const bool stereo_matched =
            binocular && binocular->right_indices()(static_cast<int>(observation.feature)) >= 0;
        count += stereo_matched ? 2 : 1;
    }
    return count;
}

template <typename TFeatureExtractor>
KeyFrameSignals Mapper<TFeatureExtractor>::MeasureSignals(
    const events::FrameTracked<TFeatureExtractor>& event) const {
    assert(!keyframes_.empty());
    KeyFrameSignals signals;
    signals.num_frames = num_keyframes_.load(std::memory_order_relaxed);
    signals.backlog = backlog_;
    signals.inliers = event.num_inliers;
    signals.since_last =
        event.timestamp - TimestampOf<TFeatureExtractor>(LastKeyFrame().measurement());

    // The reference Frame, or the newest one if it is unknown or gone.
    const TKeyFrame* reference =
        event.reference_frame_id ? FindKeyFrame(*event.reference_frame_id) : nullptr;
    if (reference == nullptr) {
        reference = &LastKeyFrame();
    }
    const size_t min_observations = signals.num_frames <= 2 ? 2 : 3;
    for (const map::MapPointId point : covisibility_.Points(reference->id())) {
        if (map_points_.IsValid(point) &&
            WeightedObservations(point) >= min_observations) {
            ++signals.ref_points;
        }
    }

    const auto* binocular =
        std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(event.measurement.get());
    signals.stereo = binocular != nullptr;
    if (binocular) {
        const double close_depth =
            options_.keyframe.close_depth_baselines * RigBaseline(binocular->rig());
        const auto num_features = static_cast<size_t>(binocular->left().points().cols());
        std::vector<bool> tracked(num_features, false);
        for (const uint32_t feature : event.observed_features) {
            if (feature < num_features) {
                tracked[feature] = true;
            }
        }
        for (size_t i = 0; i < num_features; ++i) {
            const auto index = static_cast<int>(i);
            const double depth = binocular->depth(index);
            if (binocular->right_indices()(index) < 0 || depth <= 0.0 || depth >= close_depth) {
                continue;
            }
            ++(tracked[i] ? signals.close_tracked : signals.close_untracked);
        }
    }
    return signals;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::CreateFrame(const events::FrameTracked<TFeatureExtractor>& event) {
    auto start = Clock::now();
    auto* created = CreateTransaction();
    ResetTransactionScratch();

    // The Frame and its tracked observations (ORB-SLAM3 ProcessNewKeyFrame).
    // The Tracker's replica may be behind or the pairs malformed: a point
    // that is not alive or is already observed by this Frame, and a feature
    // out of range or already paired, is skipped.
    const map::FrameId id = next_frame_id_;
    ++next_frame_id_.value;
    const TKeyFrame& keyframe = AddKeyFrame(id, event.pose, event.measurement);
    const auto num_features =
        static_cast<size_t>(LeftOf<TFeatureExtractor>(keyframe.measurement()).points().cols());
    AddFramePart(keyframe, *created);
    std::vector<bool> has_point(num_features, false);
    assert(event.observed_features.size() == event.observed_points.size());
    std::vector<std::pair<uint32_t, map::MapPointId>> tracked;
    for (size_t i = 0; i < event.observed_points.size(); ++i) {
        const map::MapPointId point = event.observed_points[i];
        const uint32_t feature = event.observed_features[i];
        if (!map_points_.IsValid(point) || covisibility_.Contains(id, point) ||
            feature >= num_features || has_point[feature]) {
            continue;
        }
        AddObservation(keyframe, feature, point, *created);
        has_point[feature] = true;
        touched_.push_back(point);
        tracked.emplace_back(feature, point);
    }

    // Recent points, judged with the new Frame's observations counted
    // (ORB-SLAM3 MapPointCulling). A culled tracked point frees its feature.
    const size_t num_points_before = map_points_.size();
    CullRecentPoints(*created);
    const size_t num_culled = num_points_before - map_points_.size();
    size_t num_tracked = 0;
    for (const auto& [feature, point] : tracked) {
        if (map_points_.IsValid(point)) {
            ++num_tracked;
        } else {
            has_point[feature] = false;
        }
    }

    // New points take the contiguous ids from here.
    const map::MapPointId first_new{map_points_.num_ids()};
    created->new_points_start_id = first_new;

    // New points: stereo close points first, so triangulation skips their
    // features.
    const bool stereo = detail::BinocularOf<TFeatureExtractor>(keyframe.measurement()) != nullptr;
    if (stereo) {
        AddClosePoints(keyframe, has_point, *created);
    }
    const size_t num_close = map_points_.num_ids() - first_new.value;
    TriangulateWithNeighbours(keyframe, std::move(has_point), *created);
    const uint32_t num_new = map_points_.num_ids() - first_new.value;
    for (uint32_t i = 0; i < num_new; ++i) {
        recent_points_.push_back({map::MapPointId{first_new.value + i}, id, stereo});
    }

    const double create_ms = MillisecondsSince(start);

    // Fusion with the neighbours.
    start = Clock::now();
    Fuse(keyframe, *created);
    FinishTransaction(first_new, *created);
    // The parent shares the most points with the Frame, fused ones included.
    created->added_frames.front().parent = ConnectToParent(id);
    const double fuse_ms = MillisecondsSince(start);
    num_keyframes_.fetch_add(1, std::memory_order_acq_rel);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    observable_.Notify(created);

    // Transaction 2: local BA and Frame culling.
    const Refinement refinement = RefineAroundFrame(id);
    const LocalBaSummary& ba = last_local_ba_;
    log_->debug(
        "Frame {}: create {:.1f} ms ({} points culled, {} tracked observations, {} close points, "
        "{} triangulated points), fuse {:.1f} ms ({} points merged), local BA {:.1f} ms ({}: "
        "{} free / {} fixed Frames, {} points, {} iterations, interrupted {}, stage {}, {} "
        "outliers), Frame culling {:.1f} ms ({} Frames culled), {} points removed after BA",
        id.value, create_ms, num_culled, num_tracked, num_close, num_new - num_close, fuse_ms,
        losers_.size(), refinement.ba_ms, ba.ran ? "ran" : "skipped", ba.num_free, ba.num_fixed,
        ba.num_points, ba.iterations, ba.interrupted ? "yes" : "no", ba.stage_reached,
        ba.num_outliers, refinement.cull_ms, refinement.num_culled_frames,
        refinement.num_removed_points);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ResetTransactionScratch() {
    added_keys_.clear();
    replaced_.clear();
    losers_.clear();
    touched_.clear();
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::AddObservation(
    const TKeyFrame& frame, uint32_t feature, map::MapPointId point,
    events::MapTransaction<TFeatureExtractor>& out_event) {
    events::TransactionObservation added{frame.id(), feature, point};
    if (const auto* binocular = detail::BinocularOf<TFeatureExtractor>(frame.measurement())) {
        const auto f = static_cast<int>(feature);
        const int right = binocular->right_indices()(f);
        assert(right <= 32767);
        if (right >= 0) {
            added.depth = binocular->depth(f);
            added.right_index = static_cast<int16_t>(right);
        }
    }
    covisibility_.Add(frame.id(), feature, point);
    added_keys_.insert(detail::ObservationKey(frame.id(), point));
    if (!references_[point.value]) {
        SetReference(point, frame, feature);
    }
    if (!out_event.added_frames.empty() && out_event.added_frames.back().id == frame.id()) {
        events::NewFrame<TFeatureExtractor>& part = out_event.added_frames.back();
        part.observed_features.push_back(feature);
        part.observed_points.push_back(point);
    } else {
        out_event.added_observations.push_back(added);
    }
}

template <typename TFeatureExtractor>
map::MapPointId Mapper<TFeatureExtractor>::AddNewPoint(
    const geometry::TPoint3DF& position, const TKeyFrame& keyframe, uint32_t feature,
    const std::vector<geometry::TPoint3DF>& centres,
    events::MapTransaction<TFeatureExtractor>& out_event) {
    const map::MapPointId point = map_points_.Add(
        position, LeftOf<TFeatureExtractor>(keyframe.measurement()).descriptors().col(feature),
        map::MeanViewingDirection(position, centres));
    OnPointAdded(point);
    AddObservation(keyframe, feature, point, out_event);
    return point;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnPointAdded(map::MapPointId point) {
    // Ids are issued in order, so the per-point vectors grow by one.
    assert(point.value == references_.size());
    references_.emplace_back();
    num_visible_.push_back(1);
    num_found_.push_back(1);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::SetReference(map::MapPointId point, const TKeyFrame& frame,
                                             uint32_t feature) {
    const features::TLevels& levels = LeftOf<TFeatureExtractor>(frame.measurement()).levels();
    const geometry::TPoint3DF centre = frame.pose().Inverse().translation();
    const geometry::TPoint3DF position = map_points_.Position(point);
    references_[point.value] =
        map::PointReference{frame.id(), static_cast<Eigen::Index>(feature),
                            static_cast<int>(levels(static_cast<Eigen::Index>(feature))),
                            static_cast<double>((position - centre).norm())};
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::RemovePoint(map::MapPointId point,
                                            events::MapTransaction<TFeatureExtractor>& out_event) {
    covisibility_.RemovePoint(point);
    map_points_.Remove(point);
    references_[point.value].reset();
    out_event.removed_points.push_back(point);
}

template <typename TFeatureExtractor>
std::optional<map::PointReference> Mapper<TFeatureExtractor>::Reference(
    map::MapPointId point) const {
    return point.value < references_.size() ? references_[point.value] : std::nullopt;
}

template <typename TFeatureExtractor>
uint32_t Mapper<TFeatureExtractor>::NumVisible(map::MapPointId point) const {
    return point.value < num_visible_.size() ? num_visible_[point.value] : 0;
}

template <typename TFeatureExtractor>
uint32_t Mapper<TFeatureExtractor>::NumFound(map::MapPointId point) const {
    return point.value < num_found_.size() ? num_found_[point.value] : 0;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::AddClosePoints(
    const TKeyFrame& keyframe, std::vector<bool>& out_has_point,
    events::MapTransaction<TFeatureExtractor>& out_event) {
    const auto& binocular =
        std::get<features::TBinocularMeasurement<TFeatureExtractor>>(keyframe.measurement());
    const double close_depth =
        options_.keyframe.close_depth_baselines * RigBaseline(binocular.rig());
    // (depth, feature) of the untracked stereo-matched features, closest first.
    std::vector<std::pair<double, uint32_t>> candidates;
    for (size_t i = 0; i < out_has_point.size(); ++i) {
        const auto f = static_cast<int>(i);
        const double depth = binocular.depth(f);
        if (!out_has_point[i] && binocular.right_indices()(f) >= 0 && depth > 0.0) {
            candidates.emplace_back(depth, static_cast<uint32_t>(i));
        }
    }
    std::sort(candidates.begin(), candidates.end());

    // Placed like the stereo initialization's points: the left camera's
    // back-projection at the stereo depth.
    const geometry::RigidTransformation T_wc = keyframe.pose().Inverse();
    const std::vector<geometry::TPoint3DF> centres = {T_wc.translation()};
    size_t count = 0;
    for (const auto& [depth, feature] : candidates) {
        if (depth >= close_depth && count >= options_.min_close_points) {
            break;
        }
        const geometry::TPoint3DF position = T_wc.Transform(geometry::TPoint3DF(
            binocular.left().normalized_points().col(feature) * depth));
        AddNewPoint(position, keyframe, feature, centres, out_event);
        out_has_point[feature] = true;
        ++count;
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::TriangulateWithNeighbours(
    const TKeyFrame& keyframe, std::vector<bool> has_point,
    events::MapTransaction<TFeatureExtractor>& out_event) {
    TriangulationView<TFeatureExtractor> view = ViewOf(keyframe, std::move(has_point));
    const bool stereo = std::holds_alternative<features::TBinocularMeasurement<TFeatureExtractor>>(
        keyframe.measurement());
    const geometry::TPoint3DF centre = keyframe.pose().Inverse().translation();
    const size_t max_neighbours = stereo ? options_.neighbours_stereo : options_.neighbours_mono;
    // The neighbours are fixed before any new point changes covisibility.
    for (const auto& [neighbour_id, shared] : covisibility_.Covisible(keyframe.id(), max_neighbours)) {
        const TKeyFrame* neighbour = FindKeyFrame(neighbour_id);
        assert(neighbour != nullptr);
        const geometry::TPoint3DF neighbour_centre = neighbour->pose().Inverse().translation();
        const double baseline = (centre - neighbour_centre).norm();
        if (stereo) {
            if (baseline < view.baseline) {
                continue;
            }
        } else {
            const std::optional<double> median_depth = SceneMedianDepth(*neighbour);
            if (!median_depth || !(*median_depth > 0.0) ||
                baseline / *median_depth < options_.min_baseline_depth_ratio) {
                continue;
            }
        }

        const TriangulationView<TFeatureExtractor> neighbour_view =
            ViewOf(*neighbour, FeaturesWithPoints(*neighbour));
        const auto matches =
            SearchForTriangulation(view, neighbour_view, options_.triangulation,
                                   BowFeaturesOf(keyframe.id()), BowFeaturesOf(neighbour_id));
        const std::vector<geometry::TPoint3DF> centres = {centre, neighbour_centre};
        for (const TriangulatedPoint& triangulated :
             TriangulateMatches(view, neighbour_view, matches, options_.triangulation)) {
            const map::MapPointId point = AddNewPoint(triangulated.position, keyframe,
                                                      triangulated.feature_a, centres, out_event);
            AddObservation(*neighbour, triangulated.feature_b, point, out_event);
            // Taken for the next neighbours.
            view.has_point[triangulated.feature_a] = true;
        }
    }
}

template <typename TFeatureExtractor>
TriangulationView<TFeatureExtractor> Mapper<TFeatureExtractor>::ViewOf(
    const TKeyFrame& frame, std::vector<bool> has_point) const {
    TriangulationView<TFeatureExtractor> view{&LeftOf<TFeatureExtractor>(frame.measurement()),
                                              frame.pose(), std::move(has_point), {}, 0.0};
    if (const auto* binocular =
            std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&frame.measurement())) {
        const auto num_features = static_cast<size_t>(binocular->left().points().cols());
        view.stereo_depth.resize(num_features);
        for (size_t i = 0; i < num_features; ++i) {
            const auto f = static_cast<int>(i);
            view.stereo_depth[i] = binocular->right_indices()(f) >= 0 ? binocular->depth(f) : -1.0;
        }
        view.baseline = RigBaseline(binocular->rig());
    }
    return view;
}

template <typename TFeatureExtractor>
std::vector<bool> Mapper<TFeatureExtractor>::FeaturesWithPoints(const TKeyFrame& frame) const {
    std::vector<bool> has_point(
        static_cast<size_t>(LeftOf<TFeatureExtractor>(frame.measurement()).points().cols()), false);
    for (const map::MapPointId point : covisibility_.Points(frame.id())) {
        for (const map::PointObservation& observation : covisibility_.Observations(point)) {
            if (observation.frame == frame.id()) {
                has_point[observation.feature] = true;
            }
        }
    }
    return has_point;
}

template <typename TFeatureExtractor>
std::optional<double> Mapper<TFeatureExtractor>::SceneMedianDepth(const TKeyFrame& frame) const {
    std::vector<double> depths;
    for (const map::MapPointId point : covisibility_.Points(frame.id())) {
        if (map_points_.IsValid(point)) {
            depths.push_back(frame.pose().Transform(geometry::TPoint3DF(map_points_.Position(point))).z());
        }
    }
    if (depths.empty()) {
        return std::nullopt;
    }
    const auto median = depths.begin() + static_cast<std::ptrdiff_t>(depths.size() / 2);
    std::nth_element(depths.begin(), median, depths.end());
    return *median;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::UpdatePointStatistics(map::MapPointId point) {
    std::vector<geometry::TPoint3DF> centres;
    std::vector<typename TFeatureExtractor::TDescriptor> descriptors;
    for (const map::PointObservation& observation : covisibility_.Observations(point)) {
        const TKeyFrame* keyframe = FindKeyFrame(observation.frame);
        assert(keyframe != nullptr);
        centres.push_back(keyframe->pose().Inverse().translation());
        descriptors.push_back(
            LeftOf<TFeatureExtractor>(keyframe->measurement()).descriptors().col(observation.feature));
    }
    map_points_.Normal(point) = map::MeanViewingDirection(map_points_.Position(point), centres);
    map_points_.Descriptor(point) = map::DistinctiveDescriptor<TFeatureExtractor>(descriptors);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::SetState(MapperState state) {
    const MapperState previous = state_.exchange(state, std::memory_order_acq_rel);
    log_->info("{} -> {} (map {}, {} keyframes, {} points)", ToString(previous), ToString(state),
               current_map_id().value, num_keyframes(), map_points_.size());
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::AddInitialMapPoints(
    const std::vector<InitialMapPoint>& map_points) {
    if (map_points.empty()) {
        return;
    }
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TBinocularMeasurement = features::TBinocularMeasurement<TFeatureExtractor>;
    const TKeyFrame& first = FirstKeyFrame();
    const auto* binocular = std::get_if<TBinocularMeasurement>(&first.measurement());
    // Monocular: a two-view initialization; binocular: one stereo keyframe.
    assert(keyframes_.size() == (binocular ? 1u : 2u));
    const auto& descriptors = binocular
                                  ? binocular->left().descriptors()
                                  : std::get<TMonocularMeasurement>(first.measurement()).descriptors();
    std::vector<geometry::TVector3DF> centres;
    for (const std::optional<KeyFrameRecord>& keyframe : keyframes_) {
        centres.push_back(keyframe->frame.pose().Inverse().translation());
    }

    for (const InitialMapPoint& point : map_points) {
        assert(point.feature_in_first >= 0);
        // The normalized mean of the viewing directions of the keyframes.
        geometry::TPoint3DF normal = geometry::TPoint3DF::Zero();
        for (const geometry::TVector3DF& centre : centres) {
            normal += (point.position - centre).normalized();
        }
        normal.normalize();
        const map::MapPointId id =
            map_points_.Add(point.position, descriptors.col(point.feature_in_first), normal);
        OnPointAdded(id);
        const auto feature = static_cast<uint32_t>(point.feature_in_first);
        covisibility_.Add(first.id(), feature, id);
        SetReference(id, first, feature);
        if (!binocular) {
            assert(point.feature_in_second >= 0);
            covisibility_.Add(keyframes_[1]->frame.id(), static_cast<uint32_t>(point.feature_in_second),
                              id);
        }
    }
}

template <typename TFeatureExtractor>
events::NewFrame<TFeatureExtractor>& Mapper<TFeatureExtractor>::AddFramePart(
    const TKeyFrame& keyframe, events::MapTransaction<TFeatureExtractor>& out_event) const {
    events::NewFrame<TFeatureExtractor>& part = out_event.added_frames.emplace_back();
    part.id = keyframe.id();
    part.T_cw = keyframe.pose();
    part.measurement = keyframe.shared_measurement();
    const KeyFrameRecord* record = FindRecord(keyframe.id());
    assert(record != nullptr);
    part.parent = record->parent;
    if (vocabulary_) {
        part.bow = record->bow;
        part.features = record->features;
    }
    return part;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishNewMap() {
    auto* event = CreateTransaction();
    event->new_map = true;
    event->start_frame = LastKeyFrame().id();
    for (const std::optional<KeyFrameRecord>& record : keyframes_) {
        const TKeyFrame& keyframe = record->frame;
        events::NewFrame<TFeatureExtractor>& part = AddFramePart(keyframe, *event);
        // (left) feature index, point id; a stereo keyframe's depths and
        // right indices are in its measurement.
        for (const map::MapPointId point : covisibility_.Points(keyframe.id())) {
            part.observed_features.push_back(*FeatureOf(keyframe.id(), point));
            part.observed_points.push_back(point);
        }
    }
    // Fresh storage of a new map: the ids are exactly [0, num_ids()).
    const uint32_t count = map_points_.num_ids();
    event->new_points_start_id = map::MapPointId{0};
    event->ResizeNewPoints(count);
    for (uint32_t i = 0; i < count; ++i) {
        const map::MapPointId id{i};
        event->new_positions.col(i) = map_points_.Position(id);
        event->new_descriptors.col(i) = map_points_.Descriptor(id);
        event->new_normals.col(i) = map_points_.Normal(id);
    }
    observable_.Notify(event);
}

INSTANTIATE_CLASS(Mapper)

} // namespace voyahslam::mapping
