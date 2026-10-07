#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/map/point_statistics.h>
#include <voyahslam/mapping/mapper.h>

namespace voyahslam::mapping {
namespace {

const char* ToString(MapperState state) {
    switch (state) {
        case MapperState::Initializing: return "Initializing";
        case MapperState::Mapping: return "Mapping";
    }
    return "?";
}

// The left image of a measurement (the image itself for a monocular one).
template <typename TFeatureExtractor>
const features::TMonocularMeasurement<TFeatureExtractor>& LeftOf(
    const features::TMeasurement<TFeatureExtractor>& measurement) {
    if (const auto* binocular =
            std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement)) {
        return binocular->left();
    }
    return std::get<features::TMonocularMeasurement<TFeatureExtractor>>(measurement);
}

// The distance between a rig's two cameras.
inline double RigBaseline(const features::StereoRig& rig) {
    return static_cast<double>(rig.T_rl.translation().norm());
}

template <typename TFeatureExtractor>
double TimestampOf(const features::TMeasurement<TFeatureExtractor>& measurement) {
    return std::visit([](const auto& m) { return m.timestamp(); }, measurement);
}

} // namespace

template <typename TFeatureExtractor>
Mapper<TFeatureExtractor>::Mapper(MapperOptions options)
    : options_(options)
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
    while (inbox_.queue().try_dequeue(event)) {
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
    // Maps with a FrameTracked or TrackingLost later in the batch; a batch
    // rarely spans more than one or two maps.
    std::vector<map::MapId> later;
    for (size_t i = batch_.size(); i-- > 0;) {
        const events::Event& event = *batch_[i];
        if (event.type != events::EventType::FrameTracked &&
            event.type != events::EventType::TrackingLost) {
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
        default:
            return;  // LoopDetected: handled later
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::SkipFrameTracked(const events::Event& event) {
    if (state() != MapperState::Mapping || event.map_id != current_map_id()) {
        return;  // would not have been evaluated anyway
    }
    ++backlog_;
    backlog_skipped_.fetch_add(1, std::memory_order_acq_rel);
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
        keyframes_.emplace_back(next_frame_id_, keyframe.T_cw, std::move(keyframe.measurement));
        ++next_frame_id_.value;
    }
    map_points_ = {};
    observations_ = {};
    stereo_observations_ = {};
    covisibility_.Clear();
    AddInitialMapPoints(initial->map_points);
    num_keyframes_.store(keyframes_.size(), std::memory_order_release);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    current_map_id_.store(next_map_id_, std::memory_order_release);
    ++next_map_id_.value;

    // The state changes before the events that announce it are published, so
    // anyone who observes MapCreated also observes state() == Mapping.
    SetState(MapperState::Mapping);
    // The first keyframe carries every initial point; a monocular second
    // keyframe only its observations.
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        PublishKeyFrameCreated(keyframes_[i], i == 0);
    }
    PublishMapCreated(keyframes_.back());
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnTrackingLost(const events::TrackingLost& event) {
    if (event.map_id != current_map_id() || state() != MapperState::Mapping) {
        return;  // stale: an older map
    }
    ClearMap();
    initializer_.Reset();
    SetState(MapperState::Initializing);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ClearMap() {
    keyframes_.clear();
    map_points_ = {};
    observations_ = {};
    stereo_observations_ = {};
    covisibility_.Clear();
    backlog_ = 0;
    num_keyframes_.store(0, std::memory_order_release);
    num_map_points_.store(0, std::memory_order_release);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnFrameTracked(
    const events::FrameTracked<TFeatureExtractor>& event) {
    if (state() != MapperState::Mapping || event.map_id != current_map_id()) {
        return;  // no map yet, or an older map's
    }
    const KeyFrameSignals signals = MeasureSignals(event);
    const bool create = ShouldCreateFrame(signals, options_.keyframe);
    num_frame_evaluations_.fetch_add(1, std::memory_order_acq_rel);
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
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::FindKeyFrame(map::FrameId id) const -> const TKeyFrame* {
    // A map's Frame ids are consecutive from its first Frame's.
    if (keyframes_.empty() || id < keyframes_.front().id()) {
        return nullptr;
    }
    const size_t index = id.value - keyframes_.front().id().value;
    if (index >= keyframes_.size()) {
        return nullptr;
    }
    assert(keyframes_[index].id() == id);
    return &keyframes_[index];
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
    signals.num_frames = keyframes_.size();
    signals.backlog = backlog_;
    signals.inliers = event.num_inliers;
    signals.since_last = event.timestamp - TimestampOf<TFeatureExtractor>(keyframes_.back().measurement());

    // The reference Frame, or the newest one if it is unknown or gone.
    const TKeyFrame* reference =
        event.reference_frame_id ? FindKeyFrame(*event.reference_frame_id) : nullptr;
    if (reference == nullptr) {
        reference = &keyframes_.back();
    }
    const size_t min_observations = keyframes_.size() <= 2 ? 2 : 3;
    for (const map::MapPointId point : covisibility_.Points(reference->id())) {
        if (map_points_.IsValid(point) &&
            WeightedObservations(point) >= min_observations) {
            ++signals.ref_points;
        }
    }

    const auto* binocular =
        std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&event.measurement);
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
    const map::FrameId id = next_frame_id_;
    ++next_frame_id_.value;
    keyframes_.emplace_back(id, event.pose, TMeasurement(event.measurement));
    const TKeyFrame& keyframe = keyframes_.back();
    const auto num_features =
        static_cast<size_t>(LeftOf<TFeatureExtractor>(keyframe.measurement()).points().cols());

    auto* created = key_frame_created_pool_.Create();
    created->map_id = current_map_id();
    created->frame_id = id;
    created->pose = keyframe.pose();
    created->measurement = keyframe.measurement();
    // New points take the contiguous ids from here.
    const map::MapPointId first_new{map_points_.num_ids()};
    created->new_points_start_id = first_new;

    // The tracked observations. The Tracker's replica may be behind or the
    // pairs malformed: a point that is not alive or is already observed by
    // this Frame, and a feature out of range or already paired, is skipped.
    std::vector<map::MapPointId> touched;
    touched.reserve(event.observed_points.size());
    std::vector<bool> has_point(num_features, false);
    assert(event.observed_features.size() == event.observed_points.size());
    for (size_t i = 0; i < event.observed_points.size(); ++i) {
        const map::MapPointId point = event.observed_points[i];
        const uint32_t feature = event.observed_features[i];
        if (!map_points_.IsValid(point) || covisibility_.Contains(id, point) ||
            feature >= num_features || has_point[feature]) {
            continue;
        }
        AddObservation(keyframe, feature, point);
        has_point[feature] = true;
        created->observed_features.push_back(feature);
        created->observed_points.push_back(point);
        touched.push_back(point);
    }
    const size_t num_tracked = created->observed_points.size();

    // New points: stereo close points first, so triangulation skips their
    // features.
    if (std::holds_alternative<features::TBinocularMeasurement<TFeatureExtractor>>(
            keyframe.measurement())) {
        AddClosePoints(keyframe, has_point, *created);
    }
    const size_t num_close = map_points_.num_ids() - first_new.value;
    TriangulateWithNeighbours(keyframe, std::move(has_point), *created);

    UpdatePointStatistics(touched, *created);
    const uint32_t num_new = map_points_.num_ids() - first_new.value;
    created->ResizeNewPoints(num_new);
    for (uint32_t i = 0; i < num_new; ++i) {
        const map::MapPointId point{first_new.value + i};
        created->new_positions.col(i) = map_points_.Position(point);
        created->new_descriptors.col(i) = map_points_.Descriptor(point);
        created->new_normals.col(i) = map_points_.Normal(point);
    }
    log_->debug("Frame {}: {} tracked observations, {} close points, {} triangulated points",
                id.value, num_tracked, num_close, num_new - num_close);
    num_keyframes_.store(keyframes_.size(), std::memory_order_release);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    observable_.Notify(created);
}

template <typename TFeatureExtractor>
events::AddedObservation Mapper<TFeatureExtractor>::AddObservation(const TKeyFrame& frame,
                                                                   uint32_t feature,
                                                                   map::MapPointId point) {
    assert(feature < 65536);
    const auto feature_index = static_cast<uint16_t>(feature);
    events::AddedObservation added{frame.id(), feature, point};
    if (const auto* binocular =
            std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&frame.measurement())) {
        const auto f = static_cast<int>(feature);
        const int right = binocular->right_indices()(f);
        assert(right <= 32767);
        if (right >= 0) {
            added.depth = binocular->depth(f);
            added.right_index = static_cast<int16_t>(right);
            stereo_observations_.Add(point, frame.id(), feature_index, added.depth,
                                     added.right_index);
        } else {
            stereo_observations_.Add(point, frame.id(), feature_index);
        }
    } else {
        observations_.Add(point, frame.id(), feature_index);
    }
    covisibility_.Add(frame.id(), feature, point);
    return added;
}

template <typename TFeatureExtractor>
map::MapPointId Mapper<TFeatureExtractor>::AddNewPoint(
    const geometry::TPoint3DF& position, const TKeyFrame& keyframe, uint32_t feature,
    const std::vector<geometry::TPoint3DF>& centres,
    events::KeyFrameCreated<TFeatureExtractor>& out_event) {
    const map::MapPointId point = map_points_.Add(
        position, LeftOf<TFeatureExtractor>(keyframe.measurement()).descriptors().col(feature),
        map::MeanViewingDirection(position, centres));
    AddObservation(keyframe, feature, point);
    out_event.observed_features.push_back(feature);
    out_event.observed_points.push_back(point);
    return point;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::AddClosePoints(
    const TKeyFrame& keyframe, std::vector<bool>& out_has_point,
    events::KeyFrameCreated<TFeatureExtractor>& out_event) {
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
    events::KeyFrameCreated<TFeatureExtractor>& out_event) {
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
        const auto matches = SearchForTriangulation(view, neighbour_view, options_.triangulation);
        const std::vector<geometry::TPoint3DF> centres = {centre, neighbour_centre};
        for (const TriangulatedPoint& triangulated :
             TriangulateMatches(view, neighbour_view, matches, options_.triangulation)) {
            const map::MapPointId point =
                AddNewPoint(triangulated.position, keyframe, triangulated.feature_a, centres, out_event);
            out_event.added_observations.push_back(
                AddObservation(*neighbour, triangulated.feature_b, point));
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
void Mapper<TFeatureExtractor>::UpdatePointStatistics(
    const std::vector<map::MapPointId>& points,
    events::KeyFrameCreated<TFeatureExtractor>& out_event) {
    out_event.ResizeUpdatedPoints(static_cast<uint32_t>(points.size()));
    std::vector<geometry::TPoint3DF> centres;
    std::vector<typename TFeatureExtractor::TDescriptor> descriptors;
    for (size_t i = 0; i < points.size(); ++i) {
        const map::MapPointId point = points[i];
        centres.clear();
        descriptors.clear();
        for (const map::PointObservation& observation : covisibility_.Observations(point)) {
            const TKeyFrame* keyframe = FindKeyFrame(observation.frame);
            assert(keyframe != nullptr);
            centres.push_back(keyframe->pose().Inverse().translation());
            descriptors.push_back(
                LeftOf<TFeatureExtractor>(keyframe->measurement()).descriptors().col(observation.feature));
        }
        const geometry::TPoint3DF position = map_points_.Position(point);
        map_points_.Normal(point) = map::MeanViewingDirection(position, centres);
        map_points_.Descriptor(point) = map::DistinctiveDescriptor<TFeatureExtractor>(descriptors);

        out_event.updated_ids[i] = point;
        out_event.updated_positions.col(i) = position;
        out_event.updated_normals.col(i) = map_points_.Normal(point);
        out_event.updated_descriptors.col(i) = map_points_.Descriptor(point);
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::SetState(MapperState state) {
    const MapperState previous = state_.exchange(state, std::memory_order_acq_rel);
    log_->info("{} -> {} (map {}, {} keyframes, {} points)", ToString(previous), ToString(state),
               current_map_id().value, keyframes_.size(), map_points_.size());
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::AddInitialMapPoints(
    const std::vector<InitialMapPoint>& map_points) {
    if (map_points.empty()) {
        return;
    }
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TBinocularMeasurement = features::TBinocularMeasurement<TFeatureExtractor>;
    const TKeyFrame& first = keyframes_[0];
    const auto* binocular = std::get_if<TBinocularMeasurement>(&first.measurement());
    // Monocular: a two-view initialization; binocular: one stereo keyframe.
    assert(keyframes_.size() == (binocular ? 1u : 2u));
    const auto& descriptors = binocular
                                  ? binocular->left().descriptors()
                                  : std::get<TMonocularMeasurement>(first.measurement()).descriptors();
    std::vector<geometry::TVector3DF> centres;
    for (const TKeyFrame& keyframe : keyframes_) {
        centres.push_back(keyframe.pose().Inverse().translation());
    }

    for (const InitialMapPoint& point : map_points) {
        assert(point.feature_in_first >= 0 && point.feature_in_first < 65536);
        // The normalized mean of the viewing directions of the keyframes.
        geometry::TPoint3DF normal = geometry::TPoint3DF::Zero();
        for (const geometry::TVector3DF& centre : centres) {
            normal += (point.position - centre).normalized();
        }
        normal.normalize();
        const map::MapPointId id =
            map_points_.Add(point.position, descriptors.col(point.feature_in_first), normal);
        const auto feature = static_cast<uint16_t>(point.feature_in_first);
        if (binocular) {
            const auto i = static_cast<int>(point.feature_in_first);
            const int right = binocular->right_indices()(i);
            assert(right <= 32767);
            stereo_observations_.Add(id, first.id(), feature, binocular->depth(i),
                                     static_cast<int16_t>(right));
            covisibility_.Add(first.id(), feature, id);
        } else {
            assert(point.feature_in_second >= 0 && point.feature_in_second < 65536);
            const auto feature_in_second = static_cast<uint16_t>(point.feature_in_second);
            observations_.Add(id, first.id(), feature);
            observations_.Add(id, keyframes_[1].id(), feature_in_second);
            covisibility_.Add(first.id(), feature, id);
            covisibility_.Add(keyframes_[1].id(), feature_in_second, id);
        }
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishKeyFrameCreated(const TKeyFrame& keyframe,
                                                       bool with_new_points) {
    auto* event = key_frame_created_pool_.Create();
    event->map_id = current_map_id();
    event->frame_id = keyframe.id();
    event->pose = keyframe.pose();
    event->measurement = keyframe.measurement();
    // (left) feature index, point id; a stereo keyframe's depths and right
    // indices are in its measurement.
    const auto add_observations = [&](const auto& observations) {
        for (size_t i = 0; i < observations.size(); ++i) {
            if (observations.frame_id(i) == keyframe.id()) {
                event->observed_features.push_back(observations.feature_index(i));
                event->observed_points.push_back(observations.mappoint_id(i));
            }
        }
    };
    add_observations(observations_);
    add_observations(stereo_observations_);
    if (with_new_points) {
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
    }
    observable_.Notify(event);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishMapCreated(const TKeyFrame& keyframe) {
    auto* event = map_created_pool_.Create();
    event->map_id = current_map_id();
    event->frame_id = keyframe.id();
    event->pose = keyframe.pose();
    observable_.Notify(event);
}

INSTANTIATE_CLASS(Mapper)

} // namespace voyahslam::mapping
