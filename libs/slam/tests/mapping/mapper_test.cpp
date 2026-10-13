#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

namespace voyahslam::mapping {

// The Mapper's private steps for tests: publishes a transaction that adds a
// loop edge between Frames `a` and `b` of the current map; runs the loop
// correction's CorrectWindow (nothing published) and reads the map back.
struct MapperTestAccess {
    template <typename TFeatureExtractor>
    static void PublishLoopEdge(Mapper<TFeatureExtractor>& out_mapper, map::FrameId a,
                                map::FrameId b) {
        auto event = out_mapper.CreateTransaction();
        out_mapper.AddLoopEdge(a, b, *event);
        out_mapper.observable_.Notify(std::move(event));
    }
    template <typename TFeatureExtractor>
    static void CorrectWindow(Mapper<TFeatureExtractor>& out_mapper, const std::vector<map::FrameId>& window,
                              const geometry::Sim3& S_query_world) {
        out_mapper.CorrectWindow(window, S_query_world);
    }
    // Steps 1-5 of a correction of `query` with `match` that fuses only
    // `matched_points` (no loop points): the loop connections. Nothing is
    // published.
    template <typename TFeatureExtractor>
    static std::vector<std::pair<map::FrameId, map::FrameId>> LoopConnections(
        Mapper<TFeatureExtractor>& out_mapper, map::FrameId query, map::FrameId match,
        const geometry::Sim3& S_query_match,
        const std::vector<std::pair<map::MapPointId, map::MapPointId>>& matched_points) {
        std::vector<map::FrameId> window = {query};
        for (const map::FrameId frame : out_mapper.Connected(query)) window.push_back(frame);
        const geometry::RigidTransformation& T_match = out_mapper.FindKeyFrame(match)->pose();
        const auto correction = out_mapper.CorrectWindow(
            window, geometry::Compose(S_query_match, geometry::Sim3{1.0, T_match.rotation(), T_match.translation()}));
        const auto scratch = out_mapper.CreateTransaction();
        out_mapper.FuseLoopPoints(correction, matched_points, {}, *scratch);
        return out_mapper.LoopConnections(correction, match);
    }
    // Culls Frame `frame` of the current map through Frame culling's path
    // (RemoveFrame, FinishRefinement) and publishes it as one transaction.
    template <typename TFeatureExtractor>
    static void CullFrame(Mapper<TFeatureExtractor>& out_mapper, map::FrameId frame) {
        auto event = out_mapper.CreateTransaction();
        std::vector<map::MapPointId> changed;
        out_mapper.RemoveFrame(frame, *event, changed);
        out_mapper.FinishRefinement(std::move(changed), *event);
        out_mapper.num_keyframes_.fetch_sub(1, std::memory_order_acq_rel);
        out_mapper.num_map_points_.store(out_mapper.Active().map_points.size(), std::memory_order_release);
        out_mapper.observable_.Notify(std::move(event));
    }
    // Map merging step 1: the window of `frame` in map `map` (it must be live).
    template <typename TFeatureExtractor>
    static std::vector<map::FrameId> MergeWindowOf(const Mapper<TFeatureExtractor>& mapper, map::MapId map,
                                                   map::FrameId frame) {
        return mapper.MergeWindowOf(mapper.maps_.at(map), frame);
    }
    template <typename TFeatureExtractor>
    static geometry::RigidTransformation Pose(const Mapper<TFeatureExtractor>& mapper, map::FrameId frame) {
        return mapper.FindKeyFrame(frame)->pose();
    }
    template <typename TFeatureExtractor>
    static geometry::TPoint3DF Position(const Mapper<TFeatureExtractor>& mapper, map::MapPointId point) {
        return mapper.Active().map_points.Position(point);
    }
    // Map `map`'s points and their visible / found counters (it must be live).
    template <typename TFeatureExtractor>
    static size_t NumPoints(const Mapper<TFeatureExtractor>& mapper, map::MapId map) {
        return mapper.maps_.at(map).map_points.size();
    }
    template <typename TFeatureExtractor>
    static std::vector<uint32_t> NumVisible(const Mapper<TFeatureExtractor>& mapper, map::MapId map) {
        return mapper.maps_.at(map).num_visible;
    }
    template <typename TFeatureExtractor>
    static std::vector<uint32_t> NumFound(const Mapper<TFeatureExtractor>& mapper, map::MapId map) {
        return mapper.maps_.at(map).num_found;
    }
};

namespace {

using Extractor = test::TestExtractor;
using TMapper = Mapper<Extractor>;
using TTransaction = events::MapTransaction<Extractor>;
using TFrameTracked = events::FrameTracked<Extractor>;
using TInitializationMeasurement = events::InitializationMeasurement<Extractor>;
using TMonocularMeasurement = features::TMonocularMeasurement<Extractor>;
using TBinocularMeasurement = features::TBinocularMeasurement<Extractor>;
using TDescriptors = typename Extractor::TDescriptors;
using TVocabulary = bow::Vocabulary<Extractor>;

// One Frame a transaction adds.
struct PublishedFrame {
    map::FrameId frame_id;
    double timestamp = 0.0;  // of the Frame's measurement
    geometry::RigidTransformation pose;
    // Its own observations and the measurement's (left image's) descriptors;
    // per left feature stereo depths (binocular only).
    std::vector<uint32_t> observed_features;
    std::vector<map::MapPointId> observed_points;
    TDescriptors descriptors;
    geometry::TRowVectorF depths;
    bow::BowVector bow;  // the transaction's BoW data of the Frame
    bow::FeatureVector features;
    std::optional<map::FrameId> parent;
};

// One published MapTransaction.
struct Published {
    events::EventType type;
    map::MapId map_id;
    bool new_map = false;
    std::optional<map::FrameId> start_frame;
    std::vector<PublishedFrame> frames;
    // The new points, the number of updated points and of added observations.
    map::MapPointId start_id{};
    uint32_t count = 0;
    geometry::TPoints3DF positions;
    TDescriptors point_descriptors;
    geometry::TPoints3DF normals;
    size_t num_updated = 0;
    size_t num_added_observations = 0;
    std::vector<events::TransactionObservation> added_observations;
    // The updated points.
    std::vector<map::MapPointId> updated_ids;
    geometry::TPoints3DF updated_positions;
    TDescriptors updated_descriptors;
    geometry::TPoints3DF updated_normals;
    // The removal parts: removed observations, replaced and removed points,
    // updated poses, removed Frames (their total count), and the first three.
    size_t num_other_parts = 0;
    std::vector<events::RemovedObservation> removed_observations;
    std::vector<events::ReplacedPoint> replaced_points;
    std::vector<map::MapPointId> removed_points;
    std::vector<std::pair<map::FrameId, geometry::RigidTransformation>> updated_poses;
    std::vector<map::FrameId> removed_frames;
    // The map graph and the correction epoch.
    std::vector<std::pair<map::FrameId, map::FrameId>> parent_changes;
    std::vector<std::pair<map::FrameId, map::FrameId>> added_loop_edges;
    uint32_t map_epoch = 0;
    // The Atlas parts.
    std::optional<map::MapId> active_map;
    std::vector<map::MapId> removed_maps;
    std::optional<map::MapId> absorbed_map;

    // The `k`th added Frame.
    [[nodiscard]] const PublishedFrame& frame(size_t k = 0) const { return frames.at(k); }
};

// Feeds the Mapper's inbox from `tracker_` (stands in for the Tracker) and
// captures what the Mapper publishes in `capture_`; every captured event is
// also applied to `atlas_`, as the Tracker would, and must leave every map
// consistent; `replica()` is its active map.
class MapperTest : public ::testing::Test {
protected:
    explicit MapperTest(MapperOptions options = {})
        : mapper_(options) {
        subscriptions_.push_back(tracker_.ConnectAsync(mapper_.inbox()));
        subscriptions_.push_back(mapper_.AddObserver(capture_));
    }
    MapperTest(MapperOptions options, std::shared_ptr<const TVocabulary> vocabulary)
        : mapper_(options, std::move(vocabulary)) {
        subscriptions_.push_back(tracker_.ConnectAsync(mapper_.inbox()));
        subscriptions_.push_back(mapper_.AddObserver(capture_));
    }

    void Drain() {
        while (mapper_.ProcessPending()) {
        }
    }

    template <typename TMeasurement>
    void PublishInitializationMeasurement(TMeasurement measurement) {
        auto event = std::make_shared<TInitializationMeasurement>();
        event->map_id = map::MapId{0};
        event->measurement = std::move(measurement);
        tracker_.Notify(std::move(event));
    }

    void PublishTrackingLost(uint32_t map, double timestamp, bool final = true) {
        auto event = std::make_shared<events::TrackingLost>();
        event->map_id = map::MapId{map};
        event->timestamp = timestamp;
        event->final = final;
        tracker_.Notify(std::move(event));
    }

    // Everything captured so far; `after_each` runs once each event has been
    // applied to the replica.
    std::vector<Published> TakeCaptured(
        const std::function<void(const Published&)>& after_each = {}) {
        std::vector<Published> published;
        while (events::EventPtr event = listener_.GetNextEvent()) {
            if (event->type == events::EventType::MapTransaction) {
                atlas_.Apply(static_cast<const TTransaction&>(*event));
            }
            for (const map::MapId id : atlas_.map_ids()) {
                EXPECT_TRUE(atlas_.map(id)->CheckConsistency()) << "map " << id.value;
            }
            Published p{event->type, event->map_id};
            if (event->type == events::EventType::MapTransaction) {
                const auto* transaction = static_cast<const TTransaction*>(event.get());
                p.new_map = transaction->new_map;
                p.start_frame = transaction->start_frame;
                for (const events::NewFrame<Extractor>& added : transaction->added_frames) {
                    PublishedFrame& frame = p.frames.emplace_back();
                    frame.frame_id = added.id;
                    frame.pose = added.T_cw;
                    frame.timestamp =
                        std::visit([](const auto& m) { return m.timestamp(); }, *added.measurement);
                    frame.observed_features = added.observed_features;
                    frame.observed_points = added.observed_points;
                    frame.bow = added.bow;
                    frame.features = added.features;
                    frame.parent = added.parent;
                    if (const auto* mono = std::get_if<TMonocularMeasurement>(added.measurement.get())) {
                        frame.descriptors = mono->descriptors();
                    } else if (const auto* binocular =
                                   std::get_if<TBinocularMeasurement>(added.measurement.get())) {
                        frame.descriptors = binocular->left().descriptors();
                        frame.depths = binocular->depths();
                    }
                }
                p.start_id = transaction->new_points_start_id;
                p.count = transaction->num_new_points();
                p.positions = transaction->new_positions.matrix();
                p.point_descriptors = transaction->new_descriptors.matrix();
                p.normals = transaction->new_normals.matrix();
                p.num_updated = transaction->updated_ids.size();
                p.num_added_observations = transaction->added_observations.size();
                p.added_observations = transaction->added_observations;
                p.updated_ids = transaction->updated_ids;
                p.updated_positions = transaction->updated_positions.matrix();
                p.updated_descriptors = transaction->updated_descriptors.matrix();
                p.updated_normals = transaction->updated_normals.matrix();
                p.num_other_parts = transaction->removed_observations.size() +
                                    transaction->replaced_points.size() +
                                    transaction->removed_points.size() +
                                    transaction->updated_poses.size() +
                                    transaction->removed_frames.size();
                p.removed_observations = transaction->removed_observations;
                p.replaced_points = transaction->replaced_points;
                p.removed_points = transaction->removed_points;
                p.updated_poses = transaction->updated_poses;
                p.removed_frames = transaction->removed_frames;
                p.parent_changes = transaction->parent_changes;
                p.added_loop_edges = transaction->added_loop_edges;
                p.map_epoch = transaction->map_epoch;
                p.active_map = transaction->active_map;
                p.removed_maps = transaction->removed_maps;
                p.absorbed_map = transaction->absorbed_map;
                // Every transaction names the active map: its own, or none
                // when it drops (removes or deactivates) its map; it removes
                // no other map but the one a merge absorbed.
                if (p.absorbed_map) {
                    EXPECT_EQ(p.active_map, p.map_id);
                    EXPECT_EQ(p.removed_maps, std::vector<map::MapId>{*p.absorbed_map});
                } else if (p.active_map) {
                    EXPECT_EQ(p.active_map, p.map_id);
                    EXPECT_TRUE(p.removed_maps.empty());
                } else if (!p.removed_maps.empty()) {
                    EXPECT_EQ(p.removed_maps, std::vector<map::MapId>{p.map_id});
                }
            }
            ExpectApplied(p);
            if (after_each) {
                after_each(p);
            }
            published.push_back(p);
        }
        return published;
    }

    // The replica's active map; empty without one.
    [[nodiscard]] const tracking::MapReplica<Extractor>& replica() const { return atlas_.active_or_empty(); }

    // Checks the replica just after `published` was applied to it.
    virtual void ExpectApplied(const Published& /*published*/) const {}

    // A transaction of map `map` that adds exactly Frames `frames` and
    // removes nothing; a new map starts tracking from its last Frame.
    static void ExpectPublished(const Published& actual, bool new_map, uint32_t map,
                                const std::vector<uint32_t>& frames) {
        EXPECT_EQ(actual.type, events::EventType::MapTransaction);
        EXPECT_EQ(actual.map_id, map::MapId{map});
        EXPECT_EQ(actual.new_map, new_map);
        std::vector<uint32_t> ids;
        for (const PublishedFrame& frame : actual.frames) ids.push_back(frame.frame_id.value);
        EXPECT_EQ(ids, frames);
        if (new_map && !frames.empty()) {
            EXPECT_EQ(actual.start_frame, map::FrameId{frames.back()});
        } else {
            EXPECT_FALSE(actual.start_frame.has_value());
        }
        EXPECT_EQ(actual.num_other_parts, 0u);
    }

    // The Mapper dropping map `map` after a final loss: a transaction of that
    // map with removed_maps = {map}, no active map and nothing else.
    static void ExpectRemoval(const Published& actual, uint32_t map) {
        EXPECT_EQ(actual.type, events::EventType::MapTransaction);
        EXPECT_EQ(actual.map_id, map::MapId{map});
        EXPECT_EQ(actual.removed_maps, std::vector<map::MapId>{map::MapId{map}});
        EXPECT_FALSE(actual.active_map.has_value());
        EXPECT_FALSE(actual.new_map);
        EXPECT_TRUE(actual.frames.empty());
        EXPECT_EQ(actual.count, 0u);
        EXPECT_EQ(actual.num_updated, 0u);
        EXPECT_EQ(actual.num_added_observations, 0u);
        EXPECT_EQ(actual.num_other_parts, 0u);
        EXPECT_TRUE(actual.added_loop_edges.empty());
    }

    // A final TrackingLost of the current map `map`: the Mapper drops it with
    // one transaction, taken and checked here.
    void LoseMap(uint32_t map, double timestamp) {
        PublishTrackingLost(map, timestamp);
        Drain();
        const std::vector<Published> removal = TakeCaptured();
        ASSERT_EQ(removal.size(), 1u);
        ExpectRemoval(removal[0], map);
    }

    // Index of the first frame that initializes against frame 0, measured with
    // a standalone MapInitializer (default options, like the Mapper's).
    size_t FirstSuccessIndex() const {
        MapInitializer<Extractor> initializer;
        for (size_t k = 0; k < sequence_.frames.size(); ++k) {
            if (initializer.TryInitialize(sequence_.frames[k].measurement)) {
                return k;
            }
        }
        ADD_FAILURE() << "the synthetic sequence never initializes";
        return sequence_.frames.size();
    }

    // Feeds frames [begin, ...) until the Mapper leaves Initializing (frame 0
    // of the run becomes the reference); returns everything published.
    std::vector<Published> InitializeMono(size_t begin) {
        for (size_t i = begin;
             i < sequence_.frames.size() && mapper_.state() == MapperState::Initializing; ++i) {
            PublishInitializationMeasurement(sequence_.frames[i].measurement);
            Drain();
        }
        EXPECT_EQ(mapper_.state(), MapperState::Mapping);
        return TakeCaptured();
    }

    // A stereo-matched synthetic stereo measurement (the Tracker matches
    // before forwarding).
    static TBinocularMeasurement StereoMeasurement(double timestamp) {
        test::SyntheticStereoOptions options;
        options.timestamp = timestamp;
        return test::MakeMatchedSyntheticStereoFrame(options).measurement;
    }

    static size_t NumDepths(const TBinocularMeasurement& measurement) {
        return static_cast<size_t>((measurement.depths().array() > 0.0).count());
    }

    // Initializes a map from one stereo measurement and discards the output.
    void InitializeStereo(double timestamp) {
        PublishInitializationMeasurement(StereoMeasurement(timestamp));
        Drain();
        ASSERT_EQ(mapper_.state(), MapperState::Mapping);
        TakeCaptured();
    }

    // Declared first, so it is destroyed after the Mapper: a Subscription must
    // not be destroyed while its producer runs.
    std::vector<events::Subscription> subscriptions_;
    events::Observable tracker_;
    test::SyntheticSequence sequence_ = test::MakeSyntheticSequence();
    TMapper mapper_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
    tracking::AtlasReplica<Extractor> atlas_;
};

TEST_F(MapperTest, StereoInitializesFromOneMeasurement) {
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{0});

    PublishInitializationMeasurement(StereoMeasurement(0.0));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    ExpectPublished(published[0], /*new_map=*/true, 1, {0});
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
}

TEST_F(MapperTest, StereoPublishesMapPoints) {
    const TBinocularMeasurement measurement = StereoMeasurement(0.0);
    const size_t num_depths = NumDepths(measurement);
    PublishInitializationMeasurement(measurement);
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    // The new map carries the Frame and its points.
    const Published& points = published[0];
    const PublishedFrame& keyframe = points.frame();
    ASSERT_EQ(points.type, events::EventType::MapTransaction);
    EXPECT_EQ(points.start_id, map::MapPointId{0});
    EXPECT_GE(points.count, 100u);
    EXPECT_EQ(points.count, num_depths);
    EXPECT_EQ(mapper_.num_map_points(), points.count);
    ASSERT_EQ(keyframe.observed_features.size(), points.count);
    ASSERT_EQ(keyframe.observed_points.size(), points.count);
    // A point's descriptor is the left image's descriptor of its feature, and
    // the feature has a stereo depth.
    for (size_t o = 0; o < keyframe.observed_points.size(); ++o) {
        ASSERT_LT(keyframe.observed_points[o].value, points.count);
        ASSERT_LT(keyframe.observed_features[o], static_cast<uint32_t>(keyframe.descriptors.cols()));
        EXPECT_EQ(points.point_descriptors.col(keyframe.observed_points[o].value),
                  keyframe.descriptors.col(keyframe.observed_features[o]))
            << "observation " << o;
        EXPECT_GT(keyframe.depths(keyframe.observed_features[o]), 0.0) << "observation " << o;
    }
}

TEST_F(MapperTest, StereoPublishesNewMapWithFrameAndPoints) {
    PublishInitializationMeasurement(StereoMeasurement(0.0));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    ExpectPublished(published[0], /*new_map=*/true, 1, {0});

    const Published& points = published[0];
    const PublishedFrame& keyframe = points.frame();
    EXPECT_EQ(points.num_updated, 0u);
    EXPECT_EQ(points.num_added_observations, 0u);
    EXPECT_GE(points.count, 100u);
    EXPECT_EQ(points.count, mapper_.num_map_points());
    ASSERT_EQ(points.positions.cols(), points.count);
    ASSERT_EQ(points.normals.cols(), points.count);

    // Every point is observed exactly once, by a valid feature.
    ASSERT_EQ(keyframe.observed_points.size(), points.count);
    std::vector<int> times_observed(points.count, 0);
    for (size_t o = 0; o < keyframe.observed_points.size(); ++o) {
        ASSERT_LT(keyframe.observed_points[o].value, points.count);
        ASSERT_LT(keyframe.observed_features[o], static_cast<uint32_t>(keyframe.descriptors.cols()));
        ++times_observed[keyframe.observed_points[o].value];
    }
    for (uint32_t i = 0; i < points.count; ++i) {
        EXPECT_EQ(times_observed[i], 1) << "point " << i;
    }

    const geometry::TVector3DF centre = keyframe.pose.Inverse().translation();
    for (uint32_t i = 0; i < points.count; ++i) {
        EXPECT_NEAR(points.normals.col(i).norm(), 1.0, 1e-5) << "point " << i;
        const geometry::TVector3DF position = points.positions.col(i);
        EXPECT_GT(points.normals.col(i).dot(position - centre), 0.0) << "point " << i;
    }
}

TEST_F(MapperTest, StereoMapAfterMonoMap) {
    const auto first = InitializeMono(0);
    ASSERT_FALSE(first.empty());
    ASSERT_GT(mapper_.num_map_points(), 0u);

    const double timestamp = sequence_.frames.back().measurement.timestamp() + 1.0;
    LoseMap(1, timestamp);
    ASSERT_EQ(mapper_.state(), MapperState::Initializing);

    const TBinocularMeasurement measurement = StereoMeasurement(timestamp + 1.0);
    const size_t num_depths = NumDepths(measurement);
    PublishInitializationMeasurement(measurement);
    Drain();

    const auto second = TakeCaptured();
    ASSERT_EQ(second.size(), 1u);
    ExpectPublished(second[0], /*new_map=*/true, 2, {2});
    EXPECT_EQ(second[0].start_id, map::MapPointId{0});
    EXPECT_EQ(second[0].count, num_depths);
    EXPECT_EQ(mapper_.num_map_points(), num_depths);
    EXPECT_EQ(second[0].frame().observed_points.size(), num_depths);
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
}

TEST_F(MapperTest, MonoInitializesWithParallax) {
    const size_t k = FirstSuccessIndex();
    ASSERT_LT(k, sequence_.frames.size());
    for (size_t i = 0; i < k; ++i) {
        PublishInitializationMeasurement(sequence_.frames[i].measurement);
    }
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);

    PublishInitializationMeasurement(sequence_.frames[k].measurement);
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    ExpectPublished(published[0], /*new_map=*/true, 1, {0, 1});
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.num_keyframes(), 2u);
}

TEST_F(MapperTest, MonoPublishesNewMapWithBothFramesAndPoints) {
    const auto published = InitializeMono(0);

    // One transaction: both Frames and every point.
    ASSERT_EQ(published.size(), 1u);
    ExpectPublished(published[0], /*new_map=*/true, 1, {0, 1});
    const Published& points = published[0];
    EXPECT_EQ(points.num_updated, 0u);
    EXPECT_EQ(points.num_added_observations, 0u);
    EXPECT_EQ(points.start_id, map::MapPointId{0});
    EXPECT_GT(points.count, 0u);
    EXPECT_EQ(points.count, mapper_.num_map_points());
    ASSERT_EQ(points.positions.cols(), points.count);
    ASSERT_EQ(points.point_descriptors.cols(), points.count);
    ASSERT_EQ(points.normals.cols(), points.count);
    for (uint32_t i = 0; i < points.count; ++i) {
        EXPECT_NEAR(points.normals.col(i).norm(), 1.0, 1e-5) << "point " << i;
        // The normal points from the scene towards both camera centres.
        for (size_t k = 0; k <= 1; ++k) {
            const geometry::TVector3DF centre = points.frame(k).pose.Inverse().translation();
            const geometry::TVector3DF position = points.positions.col(i);
            EXPECT_GT(points.normals.col(i).dot(position - centre), 0.0)
                << "point " << i << " keyframe " << k;
        }
    }

    for (size_t k = 0; k <= 1; ++k) {
        const PublishedFrame& keyframe = points.frame(k);
        ASSERT_EQ(keyframe.observed_features.size(), points.count);
        ASSERT_EQ(keyframe.observed_points.size(), points.count);
        for (size_t o = 0; o < keyframe.observed_points.size(); ++o) {
            EXPECT_LT(keyframe.observed_points[o].value, points.count);
            EXPECT_LT(keyframe.observed_features[o],
                      static_cast<uint32_t>(keyframe.descriptors.cols()));
        }
    }

    // Per point id, the two observed features are the same synthetic point.
    const size_t second_index = FirstSuccessIndex();
    ASSERT_LT(second_index, sequence_.frames.size());
    const auto& first_of_feature = sequence_.frames[0].point_of_feature;
    const auto& second_of_feature = sequence_.frames[second_index].point_of_feature;
    std::vector<int64_t> scene_point_in_first(points.count, -2);
    for (size_t o = 0; o < points.frame(0).observed_points.size(); ++o) {
        scene_point_in_first[points.frame(0).observed_points[o].value] =
            first_of_feature[points.frame(0).observed_features[o]];
    }
    for (size_t o = 0; o < points.frame(1).observed_points.size(); ++o) {
        const int64_t in_second = second_of_feature[points.frame(1).observed_features[o]];
        const int64_t in_first = scene_point_in_first[points.frame(1).observed_points[o].value];
        EXPECT_GE(in_first, 0) << "observation " << o;
        EXPECT_EQ(in_first, in_second) << "observation " << o;
    }

    // A point's descriptor is the first keyframe's descriptor of its feature.
    const PublishedFrame& first = points.frame(0);
    for (size_t o = 0; o < first.observed_points.size(); ++o) {
        EXPECT_EQ(points.point_descriptors.col(first.observed_points[o].value),
                  first.descriptors.col(first.observed_features[o]))
            << "observation " << o;
    }
}

TEST_F(MapperTest, NewMapStartsAtSecondFrame) {
    const auto published = InitializeMono(0);

    ASSERT_EQ(published.size(), 1u);
    ASSERT_EQ(published[0].frames.size(), 2u);
    EXPECT_EQ(published[0].start_frame, published[0].frame(1).frame_id);
}

TEST_F(MapperTest, PointIdsRestartAfterTrackingLost) {
    const auto first = InitializeMono(0);
    ASSERT_FALSE(first.empty());
    ASSERT_GT(mapper_.num_map_points(), 0u);

    LoseMap(1, sequence_.frames.back().measurement.timestamp());
    EXPECT_EQ(mapper_.num_map_points(), 0u);

    // A later stretch of the sequence, its first frame the new reference.
    const auto second = InitializeMono(17);
    ASSERT_EQ(second.size(), 1u);
    EXPECT_EQ(second[0].type, events::EventType::MapTransaction);
    EXPECT_TRUE(second[0].new_map);
    EXPECT_EQ(second[0].map_id, map::MapId{2});
    EXPECT_EQ(second[0].start_id, map::MapPointId{0});
    EXPECT_EQ(second[0].count, mapper_.num_map_points());
}

TEST_F(MapperTest, FewFeaturesReplacesReference) {
    PublishInitializationMeasurement(sequence_.frames[0].measurement);
    PublishInitializationMeasurement(test::MakeMonocular(50, 0.01));
    PublishInitializationMeasurement(sequence_.frames[3].measurement);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);

    for (size_t i = 4; i <= 25 && mapper_.state() == MapperState::Initializing; ++i) {
        PublishInitializationMeasurement(sequence_.frames[i].measurement);
        Drain();
    }

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    EXPECT_TRUE(published[0].new_map);
    ASSERT_EQ(published[0].frames.size(), 2u);
    EXPECT_DOUBLE_EQ(published[0].frame(0).timestamp, sequence_.frames[3].measurement.timestamp());
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
}

TEST_F(MapperTest, IgnoresInitializationMeasurementWhileMapping) {
    InitializeStereo(0.0);

    PublishInitializationMeasurement(StereoMeasurement(1.0));
    Drain();

    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
}

TEST_F(MapperTest, TrackingLostStartsNewMap) {
    InitializeStereo(0.0);

    PublishTrackingLost(1, 1.0);
    Drain();
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_EQ(mapper_.num_keyframes(), 0u);
    EXPECT_EQ(mapper_.num_map_points(), 0u);
    // The discard, published: map 1 removed, no active map.
    const auto removal = TakeCaptured();
    ASSERT_EQ(removal.size(), 1u);
    ExpectRemoval(removal[0], 1);

    PublishInitializationMeasurement(StereoMeasurement(2.0));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    ExpectPublished(published[0], /*new_map=*/true, 2, {1});
    EXPECT_EQ(published[0].start_id, map::MapPointId{0});
    EXPECT_GE(published[0].count, 100u);
    EXPECT_EQ(published[0].count, mapper_.num_map_points());
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{2});
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
}

TEST_F(MapperTest, IgnoresStaleTrackingLost) {
    InitializeStereo(0.0);
    PublishTrackingLost(1, 1.0);
    Drain();
    InitializeStereo(2.0);
    ASSERT_EQ(mapper_.current_map_id(), map::MapId{2});

    PublishTrackingLost(1, 3.0);
    Drain();

    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{2});
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
}

TEST_F(MapperTest, IgnoresNonFinalTrackingLost) {
    // Initializing: nothing to discard; initialization goes on.
    PublishTrackingLost(0, 0.0, /*final=*/false);
    Drain();
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_TRUE(TakeCaptured().empty());
    InitializeStereo(0.5);
    ASSERT_EQ(mapper_.current_map_id(), map::MapId{1});
    const size_t num_points = mapper_.num_map_points();

    // Mapping: the Tracker is only recently lost; the map stays.
    PublishTrackingLost(1, 1.0, /*final=*/false);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
    EXPECT_EQ(mapper_.num_map_points(), num_points);

    // Stale: a non-final loss of map 1 does not touch map 2.
    PublishTrackingLost(1, 2.0);
    Drain();
    ASSERT_EQ(mapper_.state(), MapperState::Initializing);
    InitializeStereo(3.0);
    ASSERT_EQ(mapper_.current_map_id(), map::MapId{2});
    PublishTrackingLost(1, 4.0, /*final=*/false);
    Drain();
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{2});
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
}

TEST_F(MapperTest, StateIsMappingWhenNewMapIsPublished) {
    // A synchronous observer runs on the publishing thread, inside Notify, so
    // it sees the state at the exact moment the new map is published.
    struct Probe {
        TMapper* mapper;
        bool seen = false;
        MapperState state = MapperState::Initializing;
    } probe{&mapper_};
    const events::Subscription probe_subscription = mapper_.AddSyncObserver(
        [](void* ctx, const events::Event& event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event.type == events::EventType::MapTransaction &&
                static_cast<const TTransaction&>(event).new_map) {
                p->seen = true;
                p->state = p->mapper->state();
            }
        },
        &probe);

    PublishInitializationMeasurement(StereoMeasurement(0.0));
    Drain();

    ASSERT_TRUE(probe.seen);
    EXPECT_EQ(probe.state, MapperState::Mapping);
}

TEST_F(MapperTest, DoesNotConsumeSharedMeasurement) {
    events::AsyncObserver other;
    events::Listener other_listener = other.GetListener();
    subscriptions_.push_back(tracker_.ConnectAsync(other));

    PublishInitializationMeasurement(test::MakeMonocular(200, 0.0));
    Drain();

    const events::EventPtr event = other_listener.GetNextEvent();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::InitializationMeasurement);
    const auto& init = static_cast<const TInitializationMeasurement&>(*event);
    ASSERT_TRUE(std::holds_alternative<TMonocularMeasurement>(init.measurement));
    EXPECT_EQ(std::get<TMonocularMeasurement>(init.measurement).points().cols(), 200);
}

// A monocular map (Frames 0 and 1) and FrameTracked events of later sequence
// frames that re-observe its points.
class MapperFrameTrackedTest : public MapperTest {
protected:
    using MapperTest::MapperTest;

    struct Pair {
        uint32_t feature;
        map::MapPointId point;
    };

    void SetUpMonoMap() {
        const auto published = InitializeMono(0);
        ASSERT_EQ(published.size(), 1u);
        initial_ = published[0];
        first_ = initial_.frame(0);
        second_ = initial_.frame(1);
        num_points_ = mapper_.num_map_points();
        ASSERT_GE(num_points_, 100u);
        const size_t second_index = FirstSuccessIndex();
        ASSERT_LT(second_index + 3, sequence_.frames.size());
        point_of_scene_.assign(static_cast<size_t>(sequence_.points_w.cols()), -1);
        const auto& scene_of_feature = sequence_.frames[second_index].point_of_feature;
        for (size_t o = 0; o < second_.observed_points.size(); ++o) {
            const Eigen::Index scene = scene_of_feature[second_.observed_features[o]];
            ASSERT_GE(scene, 0);
            point_of_scene_[static_cast<size_t>(scene)] = second_.observed_points[o].value;
        }
        next_ = second_index + 1;
        // The map's world is sequence frame 0's camera, scaled by the
        // initialization.
        map_scale_ = second_.pose.translation().norm() /
                     sequence_.frames[second_index].T_cw.translation().norm();
    }

    // A noise-free sequence: the initial map is then exact up to scale.
    void UseNoiseFreeSequence() {
        test::SyntheticSequenceOptions options;
        options.pixel_noise = 0.0;
        sequence_ = test::MakeSyntheticSequence(options);
    }

    // Sequence frame `index`'s true pose in the map (SetUpMonoMap first).
    geometry::RigidTransformation MapPose(size_t index) const {
        const geometry::RigidTransformation& T_cw = sequence_.frames[index].T_cw;
        return geometry::RigidTransformation(T_cw.rotation(), map_scale_ * T_cw.translation());
    }

    // The first `count` (feature, map point) pairs of sequence frame `index`.
    std::vector<Pair> TrackedPairs(size_t index, size_t count) const {
        std::vector<Pair> pairs;
        const auto& scene_of_feature = sequence_.frames[index].point_of_feature;
        for (size_t f = 0; f < scene_of_feature.size() && pairs.size() < count; ++f) {
            const Eigen::Index scene = scene_of_feature[f];
            if (scene >= 0 && point_of_scene_[static_cast<size_t>(scene)] >= 0) {
                pairs.push_back({static_cast<uint32_t>(f),
                                 map::MapPointId{static_cast<uint32_t>(
                                     point_of_scene_[static_cast<size_t>(scene)])}});
            }
        }
        EXPECT_EQ(pairs.size(), count) << "frame " << index;
        return pairs;
    }

    // Features of sequence frame `index` that observe no scene point.
    std::vector<uint32_t> ExtraFeatures(size_t index) const {
        std::vector<uint32_t> extras;
        const auto& scene_of_feature = sequence_.frames[index].point_of_feature;
        for (size_t f = 0; f < scene_of_feature.size(); ++f) {
            if (scene_of_feature[f] < 0) {
                extras.push_back(static_cast<uint32_t>(f));
            }
        }
        return extras;
    }

    // Sequence frame `index`, tracked in map `map` with `pairs` as its
    // inliers (num_inliers = pairs.size() unless given) and `visible` as its
    // visible points, posed at Frame 1 unless `pose` is given, at the frame's
    // timestamp unless `timestamp` is given.
    void PublishFrameTracked(uint32_t map, size_t index, const std::vector<Pair>& pairs,
                             std::optional<map::FrameId> reference = map::FrameId{1},
                             std::optional<size_t> num_inliers = std::nullopt,
                             std::optional<geometry::RigidTransformation> pose = std::nullopt,
                             const std::vector<map::MapPointId>& visible = {},
                             events::TrackingStage stage = events::TrackingStage::MotionModel,
                             std::optional<double> timestamp = std::nullopt) {
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = map::MapId{map};
        event->timestamp = timestamp.value_or(sequence_.frames[index].measurement.timestamp());
        event->pose = pose.value_or(second_.pose);
        event->measurement = test::Share(sequence_.frames[index].measurement);
        for (const Pair& pair : pairs) {
            event->observed_features.push_back(pair.feature);
            event->observed_points.push_back(pair.point);
        }
        event->visible_points = visible;
        event->reference_frame_id = reference;
        event->stage = stage;
        event->num_matches = static_cast<uint32_t>(pairs.size());
        event->num_inliers = static_cast<uint32_t>(num_inliers.value_or(pairs.size()));
        tracker_.Notify(std::move(event));
    }

    Published initial_;      // the new map
    PublishedFrame first_;   // its Frames
    PublishedFrame second_;
    size_t num_points_ = 0;
    std::vector<int64_t> point_of_scene_;  // map point id per scene point, -1 if none
    size_t next_ = 0;                      // the first sequence frame after the map's
    double map_scale_ = 1.0;               // map units per sequence unit
};

class MapperInboxTest : public MapperFrameTrackedTest {};
// Frame creation alone: no fusion target, no recent point culled.
class MapperFrameCreationTest : public MapperFrameTrackedTest {
protected:
    MapperFrameCreationTest()
        : MapperFrameCreationTest(CreationOnlyOptions()) {}
    explicit MapperFrameCreationTest(MapperOptions options)
        : MapperFrameTrackedTest(options) {}
    MapperFrameCreationTest(MapperOptions options, std::shared_ptr<const TVocabulary> vocabulary)
        : MapperFrameTrackedTest(options, std::move(vocabulary)) {}

    static MapperOptions CreationOnlyOptions() {
        MapperOptions options;
        options.map_quality.fuse_neighbours_mono = 0;
        options.map_quality.fuse_neighbours_stereo = 0;
        options.map_quality.min_found_ratio = 0.0;
        options.map_quality.min_obs_mono = 0;
        options.map_quality.min_obs_stereo = 0;
        return options;
    }

    // Initializes a stereo map from StereoMeasurement(0.0) and returns its
    // transaction.
    Published InitializeStereoMap() {
        PublishInitializationMeasurement(StereoMeasurement(0.0));
        Drain();
        const auto initial = TakeCaptured();
        EXPECT_EQ(initial.size(), 1u);
        EXPECT_GE(initial.at(0).frame().observed_points.size(), 100u);
        return initial.at(0);
    }

    // `measurement` tracked at `first`'s pose, with `first`'s first `count`
    // observations as its inliers: weak enough to become a Frame.
    void PublishStereoFrameTracked(const PublishedFrame& first, TBinocularMeasurement measurement,
                                   size_t count) {
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = map::MapId{1};
        event->timestamp = measurement.timestamp();
        event->pose = first.pose;
        event->measurement = test::Share(std::move(measurement));
        event->observed_features.assign(first.observed_features.begin(),
                                        first.observed_features.begin() + count);
        event->observed_points.assign(first.observed_points.begin(),
                                      first.observed_points.begin() + count);
        event->reference_frame_id = first.frame_id;
        event->num_matches = event->num_inliers = static_cast<uint32_t>(count);
        tracker_.Notify(std::move(event));
    }

    // Covisibility of the Mapper and of the replica agree for every Frame.
    void ExpectReplicaConsistent() {
        const map::CovisibilityIndex& mapper = mapper_.covisibility();
        const map::CovisibilityIndex& replicated = replica().covisibility();
        const std::vector<map::FrameId> frames = replica().keyframe_ids();
        ASSERT_EQ(frames.size(), mapper_.num_keyframes());
        for (const map::FrameId frame : frames) {
            std::vector<map::MapPointId> expected = mapper.Points(frame);
            std::vector<map::MapPointId> actual = replicated.Points(frame);
            std::sort(expected.begin(), expected.end());
            std::sort(actual.begin(), actual.end());
            EXPECT_EQ(actual, expected) << "Frame " << frame.value;
            for (const map::MapPointId point : expected) {
                const auto features = [&](const map::CovisibilityIndex& index) {
                    std::vector<std::pair<uint32_t, uint32_t>> result;
                    for (const map::PointObservation& o : index.Observations(point)) {
                        result.emplace_back(o.frame.value, o.feature);
                    }
                    std::sort(result.begin(), result.end());
                    return result;
                };
                EXPECT_EQ(features(replicated), features(mapper)) << "point " << point.value;
            }
            const size_t max_count = frames.size();
            EXPECT_EQ(replicated.Covisible(frame, max_count), mapper.Covisible(frame, max_count))
                << "Frame " << frame.value;
            // Same map graph.
            EXPECT_EQ(replica().parent(frame), mapper_.Parent(frame)) << "Frame " << frame.value;
            EXPECT_EQ(replica().loop_edges(frame), mapper_.LoopEdges(frame)) << "Frame " << frame.value;
        }
        // Same alive points, same references.
        for (uint32_t p = 0; p < replica().num_points(); ++p) {
            const map::MapPointId point{p};
            const std::optional<map::PointReference> expected = mapper_.Reference(point);
            ASSERT_EQ(replica().IsAlive(point), expected.has_value()) << "point " << p;
            if (!expected) {
                continue;
            }
            const std::optional<map::PointReference>& actual = replica().Reference(point);
            ASSERT_TRUE(actual.has_value()) << "point " << p;
            EXPECT_EQ(actual->keyframe, expected->keyframe) << "point " << p;
            EXPECT_EQ(actual->feature, expected->feature) << "point " << p;
            EXPECT_EQ(actual->level, expected->level) << "point " << p;
            EXPECT_NEAR(actual->distance, expected->distance, 1e-9) << "point " << p;
        }
    }

    // Every transaction leaves no dead ids behind.
    void ExpectApplied(const Published& published) const override { ExpectNoDeadIds(published); }

    // Every point `created` names as observed, updated or a merge winner is
    // alive once it is applied; every loser and removed point is not.
    void ExpectNoDeadIds(const Published& created) const {
        for (const PublishedFrame& frame : created.frames) {
            for (const map::MapPointId point : frame.observed_points) {
                EXPECT_TRUE(replica().IsAlive(point)) << "Frame point " << point.value;
            }
        }
        for (const events::TransactionObservation& added : created.added_observations) {
            EXPECT_TRUE(replica().IsAlive(added.point)) << "added point " << added.point.value;
        }
        for (const map::MapPointId point : created.updated_ids) {
            EXPECT_TRUE(replica().IsAlive(point)) << "updated point " << point.value;
        }
        for (const events::ReplacedPoint& replaced : created.replaced_points) {
            EXPECT_TRUE(replica().IsAlive(replaced.winner)) << "winner " << replaced.winner.value;
            EXPECT_FALSE(replica().IsAlive(replaced.loser)) << "loser " << replaced.loser.value;
            EXPECT_NE(std::find(created.removed_points.begin(), created.removed_points.end(),
                                replaced.loser),
                      created.removed_points.end())
                << "loser " << replaced.loser.value;
        }
        for (const map::MapPointId point : created.removed_points) {
            EXPECT_FALSE(replica().IsAlive(point)) << "removed point " << point.value;
        }
        // Posed Frames exist, removed ones do not.
        for (const auto& [frame, pose] : created.updated_poses) {
            EXPECT_NE(replica().keyframe(frame), nullptr) << "posed Frame " << frame.value;
        }
        for (const map::FrameId frame : created.removed_frames) {
            EXPECT_EQ(replica().keyframe(frame), nullptr) << "removed Frame " << frame.value;
        }
    }
};

TEST_F(MapperInboxTest, DrainsToLatest) {
    SetUpMonoMap();
    const size_t evaluations = mapper_.num_frame_evaluations();
    const size_t skipped = mapper_.backlog_skipped();

    for (size_t k = 0; k < 3; ++k) {
        PublishFrameTracked(1, next_ + k, TrackedPairs(next_ + k, num_points_ / 2));
    }
    ASSERT_TRUE(mapper_.ProcessPending());

    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations + 1);
    EXPECT_EQ(mapper_.backlog_skipped(), skipped + 2);
}

TEST_F(MapperInboxTest, TrackingLostAfterFrameTrackedCreatesNoFrame) {
    SetUpMonoMap();
    const size_t evaluations = mapper_.num_frame_evaluations();

    PublishFrameTracked(1, next_, TrackedPairs(next_, num_points_ / 2));
    PublishTrackingLost(1, sequence_.frames[next_].measurement.timestamp());
    Drain();

    // Only the discard of map 1 is published.
    const auto captured = TakeCaptured();
    ASSERT_EQ(captured.size(), 1u);
    ExpectRemoval(captured[0], 1);
    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations);
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_EQ(mapper_.num_keyframes(), 0u);
}

TEST_F(MapperInboxTest, NonFinalTrackingLostDoesNotSupersedeFrameTracked) {
    // The map survives a non-final loss, so the last FrameTracked before it
    // is still evaluated.
    SetUpMonoMap();
    const size_t evaluations = mapper_.num_frame_evaluations();

    PublishFrameTracked(1, next_, TrackedPairs(next_, num_points_ / 2));
    PublishTrackingLost(1, sequence_.frames[next_].measurement.timestamp(), /*final=*/false);
    Drain();

    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations + 1);
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_GE(mapper_.num_keyframes(), 2u);
}

TEST_F(MapperInboxTest, IgnoresFrameTrackedOfOldMap) {
    SetUpMonoMap();
    const std::vector<Pair> pairs = TrackedPairs(next_, num_points_ / 2);
    PublishTrackingLost(1, sequence_.frames.back().measurement.timestamp());
    Drain();
    ASSERT_FALSE(InitializeMono(17).empty());
    ASSERT_EQ(mapper_.current_map_id(), map::MapId{2});
    const size_t evaluations = mapper_.num_frame_evaluations();

    // Weak tracking, but against map 1: its point ids mean nothing in map 2.
    PublishFrameTracked(1, next_, pairs);
    Drain();

    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations);
    EXPECT_EQ(mapper_.num_keyframes(), 2u);
}

TEST_F(MapperFrameCreationTest, CreatesFrameWhenWeak) {
    SetUpMonoMap();
    const size_t evaluations = mapper_.num_frame_evaluations();
    // An idle Mapper (backlog 0) and inliers at 50 % of the reference's points.
    const std::vector<Pair> pairs = TrackedPairs(next_, num_points_ / 2);
    PublishFrameTracked(1, next_, pairs);
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    const Published& created = published[0];
    ExpectPublished(created, /*new_map=*/false, 1, {second_.frame_id.value + 1});
    const PublishedFrame& frame = created.frame();
    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations + 1);
    EXPECT_EQ(mapper_.num_keyframes(), 3u);
    // Points triangulated against Frame 0 (Frame 1 shares the pose) follow
    // the tracked observations, each also observed by Frame 0.
    EXPECT_EQ(mapper_.num_map_points(), num_points_ + created.count);
    EXPECT_EQ(created.start_id, map::MapPointId{static_cast<uint32_t>(num_points_)});
    EXPECT_EQ(created.num_added_observations, created.count);
    EXPECT_DOUBLE_EQ(frame.timestamp, sequence_.frames[next_].measurement.timestamp());
    EXPECT_TRUE(frame.pose.ToMatrix34().isApprox(second_.pose.ToMatrix34()));

    ASSERT_EQ(frame.observed_features.size(), pairs.size() + created.count);
    ASSERT_EQ(frame.observed_points.size(), pairs.size() + created.count);
    ASSERT_EQ(created.updated_ids.size(), pairs.size());
    ASSERT_EQ(created.updated_positions.cols(), static_cast<Eigen::Index>(pairs.size()));
    ASSERT_EQ(created.updated_descriptors.cols(), static_cast<Eigen::Index>(pairs.size()));
    ASSERT_EQ(created.updated_normals.cols(), static_cast<Eigen::Index>(pairs.size()));

    // Each point's (feature) in Frames 0 and 1.
    std::vector<int64_t> in_first(num_points_, -1);
    std::vector<int64_t> in_second(num_points_, -1);
    for (size_t o = 0; o < first_.observed_points.size(); ++o) {
        in_first[first_.observed_points[o].value] = first_.observed_features[o];
    }
    for (size_t o = 0; o < second_.observed_points.size(); ++o) {
        in_second[second_.observed_points[o].value] = second_.observed_features[o];
    }
    const std::vector<geometry::TVector3DF> centres = {
        first_.pose.Inverse().translation(), second_.pose.Inverse().translation()};
    for (size_t i = 0; i < pairs.size(); ++i) {
        EXPECT_EQ(frame.observed_features[i], pairs[i].feature) << "pair " << i;
        EXPECT_EQ(frame.observed_points[i], pairs[i].point) << "pair " << i;
        const map::MapPointId id = created.updated_ids[i];
        EXPECT_EQ(id, pairs[i].point) << "pair " << i;
        // Positions are unchanged; the normal faces every observing camera.
        const geometry::TVector3DF position = initial_.positions.col(id.value);
        EXPECT_TRUE(created.updated_positions.col(i).isApprox(position)) << "pair " << i;
        EXPECT_NEAR(created.updated_normals.col(i).norm(), 1.0, 1e-5) << "pair " << i;
        for (const geometry::TVector3DF& centre : centres) {
            EXPECT_GT(created.updated_normals.col(i).dot(position - centre), 0.0) << "pair " << i;
        }
        // The descriptor is one of the three observations'.
        const auto descriptor = created.updated_descriptors.col(i);
        ASSERT_GE(in_first[id.value], 0);
        ASSERT_GE(in_second[id.value], 0);
        EXPECT_TRUE(descriptor == first_.descriptors.col(in_first[id.value]) ||
                    descriptor == second_.descriptors.col(in_second[id.value]) ||
                    descriptor == frame.descriptors.col(pairs[i].feature))
            << "pair " << i;
    }
}

TEST_F(MapperFrameCreationTest, NoFrameWhenStrong) {
    SetUpMonoMap();
    const size_t evaluations = mapper_.num_frame_evaluations();
    // Every map point is observed by the reference, so ref_points = all of them.
    PublishFrameTracked(1, next_, TrackedPairs(next_, num_points_ / 2), map::FrameId{1},
                        num_points_);
    Drain();

    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations + 1);
    EXPECT_EQ(mapper_.num_keyframes(), 2u);
}

// The relocalization guard on any map size (the synthetic maps have 2 Frames).
class MapperRelocalizationGuardTest : public MapperFrameCreationTest {
protected:
    MapperRelocalizationGuardTest() : MapperFrameCreationTest(GuardOptions()) {}

    static MapperOptions GuardOptions() {
        MapperOptions options = CreationOnlyOptions();
        options.keyframe.relocalization_guard_min_frames = 0;
        return options;
    }

    // Sequence frame `index` tracked weakly enough to become a Frame, with
    // `stage`, at `timestamp`.
    void PublishWeak(size_t index, events::TrackingStage stage, double timestamp) {
        PublishFrameTracked(1, index, TrackedPairs(index, num_points_ / 2), map::FrameId{1},
                            std::nullopt, std::nullopt, {}, stage, timestamp);
    }
};

TEST_F(MapperRelocalizationGuardTest, NoFrameRightAfterRelocalization) {
    // ORB-SLAM3's post-relocalization guard (Ruling J): no Frame from the
    // relocalized FrameTracked, nor from any within
    // relocalization_guard_seconds of it; weak tracking would create one.
    EXPECT_DOUBLE_EQ(KeyFrameOptions{}.relocalization_guard_seconds, 1.0);
    EXPECT_EQ(KeyFrameOptions{}.relocalization_guard_min_frames, 20u);
    SetUpMonoMap();
    // After a 2 s blackout: since_last alone would make each a candidate.
    const double relocalized_at = sequence_.frames[next_].measurement.timestamp() + 2.0;

    const size_t evaluations = mapper_.num_frame_evaluations();
    PublishWeak(next_, events::TrackingStage::Relocalization, relocalized_at);
    Drain();
    PublishWeak(next_ + 1, events::TrackingStage::ZeroMotion, relocalized_at + 0.5);
    Drain();
    PublishWeak(next_ + 2, events::TrackingStage::MotionModel, relocalized_at + 0.999);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations + 3);
    EXPECT_EQ(mapper_.num_keyframes(), 2u);

    // 1 s after: the normal decision.
    PublishWeak(next_ + 3, events::TrackingStage::MotionModel, relocalized_at + 1.0);
    Drain();
    EXPECT_EQ(TakeCaptured().size(), 2u);
    EXPECT_EQ(mapper_.num_keyframes(), 3u);
}

TEST_F(MapperRelocalizationGuardTest, SupersededRelocalizationStillGuards) {
    // A relocalized FrameTracked drained behind a newer one is not evaluated,
    // but it still starts the guard.
    SetUpMonoMap();
    const double relocalized_at = sequence_.frames[next_].measurement.timestamp() + 2.0;
    PublishWeak(next_, events::TrackingStage::Relocalization, relocalized_at);
    PublishWeak(next_ + 1, events::TrackingStage::ZeroMotion, relocalized_at + 0.5);
    Drain();

    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_keyframes(), 2u);
}

TEST_F(MapperRelocalizationGuardTest, MapClearResetsGuard) {
    SetUpMonoMap();
    const double relocalized_at = sequence_.frames[next_].measurement.timestamp() + 2.0;
    PublishWeak(next_, events::TrackingStage::Relocalization, relocalized_at);
    PublishTrackingLost(1, relocalized_at + 0.1);
    Drain();
    TakeCaptured();
    ASSERT_EQ(mapper_.state(), MapperState::Initializing);

    // Map 2 from the same sequence: frame next_'s points are map 2's too.
    SetUpMonoMap();
    ASSERT_EQ(mapper_.current_map_id(), map::MapId{2});
    PublishFrameTracked(2, next_, TrackedPairs(next_, num_points_ / 2), second_.frame_id,
                        std::nullopt, std::nullopt, {}, events::TrackingStage::MotionModel,
                        relocalized_at + 0.5);
    Drain();
    EXPECT_EQ(TakeCaptured().size(), 2u);
    EXPECT_EQ(mapper_.num_keyframes(), 3u);
}

TEST_F(MapperFrameCreationTest, NoRelocalizationGuardOnSmallMap) {
    // ORB-SLAM3 guards only a map with more than mMaxFrames keyframes: this
    // 2-Frame map keeps growing right after a relocalization.
    SetUpMonoMap();
    const double relocalized_at = sequence_.frames[next_].measurement.timestamp() + 2.0;
    PublishFrameTracked(1, next_, TrackedPairs(next_, num_points_ / 2), map::FrameId{1},
                        std::nullopt, std::nullopt, {}, events::TrackingStage::Relocalization,
                        relocalized_at);
    Drain();

    EXPECT_EQ(TakeCaptured().size(), 2u);
    EXPECT_EQ(mapper_.num_keyframes(), 3u);
}

TEST_F(MapperFrameCreationTest, SkipsInvalidAndDuplicatePoints) {
    SetUpMonoMap();
    const std::vector<Pair> valid = TrackedPairs(next_, num_points_ / 2);
    const std::vector<uint32_t> extras = ExtraFeatures(next_);
    ASSERT_GE(extras.size(), 2u);
    std::vector<Pair> pairs = valid;
    pairs.insert(pairs.begin() + 1, {extras[0], map::MapPointId{static_cast<uint32_t>(num_points_)}});
    pairs.push_back({extras[1], valid[0].point});  // the same point again
    // A point the valid pairs leave out, through a feature out of range and
    // through a feature already paired.
    const map::MapPointId unused = TrackedPairs(next_, num_points_ / 2 + 1).back().point;
    const auto num_features = static_cast<uint32_t>(sequence_.frames[next_].point_of_feature.size());
    pairs.push_back({num_features, unused});
    pairs.push_back({valid[1].feature, unused});
    PublishFrameTracked(1, next_, pairs);
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    const Published& created = published[0];
    ASSERT_EQ(created.type, events::EventType::MapTransaction);
    const PublishedFrame& frame = created.frame();
    // The tracked observations come first; new points follow.
    ASSERT_EQ(frame.observed_points.size(), valid.size() + created.count);
    ASSERT_EQ(frame.observed_features.size(), valid.size() + created.count);
    ASSERT_EQ(created.updated_ids.size(), valid.size());
    for (size_t i = 0; i < valid.size(); ++i) {
        EXPECT_EQ(frame.observed_features[i], valid[i].feature) << "pair " << i;
        EXPECT_EQ(frame.observed_points[i], valid[i].point) << "pair " << i;
        EXPECT_EQ(created.updated_ids[i], valid[i].point) << "pair " << i;
    }
    EXPECT_EQ(mapper_.num_map_points(), num_points_ + created.count);
}

TEST_F(MapperFrameCreationTest, RefPointsUsesMinObs) {
    SetUpMonoMap();
    // Frame 2 re-observes half the points; they now have 3 observations.
    const size_t half = num_points_ / 2;
    PublishFrameTracked(1, next_, TrackedPairs(next_, half));
    Drain();
    ASSERT_EQ(TakeCaptured().size(), 2u);
    ASSERT_EQ(mapper_.num_keyframes(), 3u);
    const size_t evaluations = mapper_.num_frame_evaluations();

    // Against Frame 1 (observes every point): with 3 Frames only the points
    // observed >= 3 times count, so ref_points = half and 0.7 * all inliers
    // is strong (>= 0.9 * half). Counting all points would make it weak.
    const size_t inliers = num_points_ * 7 / 10;
    PublishFrameTracked(1, next_ + 1, TrackedPairs(next_ + 1, half), map::FrameId{1}, inliers);
    Drain();

    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations + 1);
    EXPECT_EQ(mapper_.num_keyframes(), 3u);
}

TEST_F(MapperFrameCreationTest, CreatesStereoFrame) {
    const TBinocularMeasurement measurement = StereoMeasurement(0.0);
    PublishInitializationMeasurement(measurement);
    Drain();
    const auto initial = TakeCaptured();
    ASSERT_EQ(initial.size(), 1u);
    const PublishedFrame& first = initial[0].frame();
    ASSERT_GE(first.observed_points.size(), 100u);

    // The same view a second later, tracking only 30 of its points: weak
    // (30 < 0.4 * ref_points), and need_close holds too (close_tracked <= 30
    // < 100, close_untracked > 70).
    const size_t count = 30;
    auto event = std::make_shared<TFrameTracked>();
    event->map_id = map::MapId{1};
    event->timestamp = 1.0;
    event->pose = first.pose;
    event->measurement = test::Share(StereoMeasurement(1.0));
    event->observed_features.assign(first.observed_features.begin(),
                                    first.observed_features.begin() + count);
    event->observed_points.assign(first.observed_points.begin(),
                                  first.observed_points.begin() + count);
    event->reference_frame_id = first.frame_id;
    event->num_matches = event->num_inliers = static_cast<uint32_t>(count);
    tracker_.Notify(std::move(event));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    ExpectPublished(published[0], /*new_map=*/false, 1, {first.frame_id.value + 1});
    // The tracked observations, then the close points (Frame 0 shares the
    // pose: nothing is triangulated).
    EXPECT_GT(published[0].count, 0u);
    EXPECT_EQ(published[0].num_added_observations, 0u);
    EXPECT_EQ(published[0].frame().observed_points.size(), count + published[0].count);
    EXPECT_EQ(published[0].updated_ids.size(), count);
    for (size_t i = 0; i < count; ++i) {
        EXPECT_EQ(published[0].frame().observed_points[i], first.observed_points[i]) << "point " << i;
        EXPECT_EQ(mapper_.covisibility().NumObservations(first.observed_points[i]), 2u)
            << "point " << i;
    }
    EXPECT_EQ(mapper_.covisibility().Points(published[0].frame().frame_id).size(),
              count + published[0].count);
}

TEST_F(MapperFrameCreationTest, RefPointsCountsStereoObservationsTwice) {
    const TBinocularMeasurement measurement = StereoMeasurement(0.0);
    PublishInitializationMeasurement(measurement);
    Drain();
    const auto initial = TakeCaptured();
    ASSERT_EQ(initial.size(), 1u);
    const PublishedFrame& first = initial[0].frame();
    const size_t num_points = first.observed_points.size();
    ASSERT_GE(num_points, 100u);
    // Every point of a stereo initialization is stereo-matched: one Frame
    // observation, counted 2 (>= min_obs 2), so ref_points = num_points.
    for (const uint32_t feature : first.observed_features) {
        ASSERT_GE(measurement.right_indices()(static_cast<int>(feature)), 0);
    }

    // Inliers at 0.3 of ref_points: weak (< ratio_stereo_early 0.4).
    const size_t count = num_points * 3 / 10;
    // need_close does not hold (>= 100 tracked close points), so only the
    // ref_points comparison can create the Frame.
    const KeyFrameOptions options;
    const double close_depth =
        options.close_depth_baselines * measurement.rig().T_rl.translation().norm();
    size_t close_tracked = 0;
    for (size_t i = 0; i < count; ++i) {
        const auto feature = static_cast<int>(first.observed_features[i]);
        const double depth = measurement.depth(feature);
        close_tracked += measurement.right_indices()(feature) >= 0 && depth > 0.0 &&
                         depth < close_depth;
    }
    ASSERT_GE(close_tracked, options.close_tracked_max);

    auto event = std::make_shared<TFrameTracked>();
    event->map_id = map::MapId{1};
    event->timestamp = 0.05;  // well within max_interval: c1b (idle) decides
    event->pose = first.pose;
    event->measurement = test::Share(StereoMeasurement(0.05));
    event->observed_features.assign(first.observed_features.begin(),
                                    first.observed_features.begin() + count);
    event->observed_points.assign(first.observed_points.begin(),
                                  first.observed_points.begin() + count);
    event->reference_frame_id = first.frame_id;
    event->num_matches = event->num_inliers = static_cast<uint32_t>(count);
    tracker_.Notify(std::move(event));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    ExpectPublished(published[0], /*new_map=*/false, 1, {first.frame_id.value + 1});
    EXPECT_EQ(published[0].frame().observed_points.size(), count + published[0].count);
    EXPECT_EQ(mapper_.num_keyframes(), 2u);
}

TEST_F(MapperFrameCreationTest, StereoClosePointsFromDepth) {
    const Published initial = InitializeStereoMap();
    const PublishedFrame& first = initial.frame();
    // The same view, noise-free (same seed: same features in the same order).
    test::SyntheticStereoOptions options;
    options.pixel_noise = 0.0;
    options.timestamp = 1.0;
    const test::SyntheticStereoFrame frame = test::MakeMatchedSyntheticStereoFrame(options);
    const TBinocularMeasurement& measurement = frame.measurement;
    const size_t count = 30;
    PublishStereoFrameTracked(first, measurement, count);
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    const Published& created = published[0];
    ExpectPublished(created, /*new_map=*/false, 1, {first.frame_id.value + 1});
    const PublishedFrame& added = created.frame();

    // Expected: the untracked stereo-matched features, every close one and at
    // least the 100 closest.
    std::vector<bool> tracked(frame.right_of_left.size(), false);
    for (size_t i = 0; i < count; ++i) {
        tracked[first.observed_features[i]] = true;
    }
    const double close_depth = KeyFrameOptions{}.close_depth_baselines *
                               measurement.rig().T_rl.translation().norm();
    size_t num_matched = 0;
    size_t num_close = 0;
    for (size_t f = 0; f < tracked.size(); ++f) {
        const auto i = static_cast<int>(f);
        if (tracked[f] || measurement.right_indices()(i) < 0) {
            continue;
        }
        ++num_matched;
        num_close += measurement.depth(i) < close_depth;
    }
    ASSERT_GT(num_close, 0u);
    ASSERT_GT(num_matched, num_close);
    const size_t expected = std::max(num_close, std::min<size_t>(100, num_matched));
    ASSERT_EQ(created.count, expected);
    EXPECT_EQ(created.start_id, map::MapPointId{static_cast<uint32_t>(initial.count)});
    EXPECT_EQ(created.num_added_observations, 0u);  // Frame 0 is too close: no triangulation
    EXPECT_EQ(mapper_.num_map_points(), initial.count + expected);

    // The Frame observes its tracked points, then each new point once.
    ASSERT_EQ(added.observed_points.size(), count + expected);
    const geometry::RigidTransformation T_wc = added.pose.Inverse();
    const geometry::TVector3DF centre = T_wc.translation();
    double max_chosen = 0.0;
    std::vector<bool> chosen(tracked.size(), false);
    for (size_t k = count; k < added.observed_points.size(); ++k) {
        const uint32_t feature = added.observed_features[k];
        const uint32_t i = added.observed_points[k].value - created.start_id.value;
        ASSERT_EQ(i, k - count) << "observation " << k;
        ASSERT_FALSE(tracked[feature]) << "observation " << k;
        ASSERT_GE(measurement.right_indices()(static_cast<int>(feature)), 0) << "observation " << k;
        ASSERT_GT(frame.depth_of_left[feature], 0.0) << "observation " << k;
        chosen[feature] = true;
        max_chosen = std::max(max_chosen, static_cast<double>(measurement.depth(static_cast<int>(feature))));
        const geometry::TVector3DF truth = T_wc.Transform(geometry::TPoint3DF(frame.points_left.col(feature)));
        const geometry::TVector3DF position = created.positions.col(i);
        EXPECT_LT((position - truth).norm(), 1e-9) << "observation " << k;
        EXPECT_EQ(created.point_descriptors.col(i), added.descriptors.col(feature))
            << "observation " << k;
        EXPECT_NEAR(created.normals.col(i).norm(), 1.0, 1e-9) << "observation " << k;
        EXPECT_GT(created.normals.col(i).dot(position - centre), 0.0) << "observation " << k;
        // The replica's reference is (new Frame, feature).
        const auto& reference = replica().Reference(added.observed_points[k]);
        ASSERT_TRUE(reference.has_value()) << "observation " << k;
        EXPECT_EQ(reference->keyframe, added.frame_id) << "observation " << k;
        EXPECT_EQ(reference->feature, static_cast<Eigen::Index>(feature)) << "observation " << k;
    }
    // The closest are chosen.
    for (size_t f = 0; f < tracked.size(); ++f) {
        const auto i = static_cast<int>(f);
        if (!tracked[f] && !chosen[f] && measurement.right_indices()(i) >= 0) {
            EXPECT_GE(measurement.depth(i), max_chosen) << "feature " << f;
        }
    }
}

TEST_F(MapperFrameCreationTest, StereoWithoutMatchesCreatesNoClosePoints) {
    const Published initial = InitializeStereoMap();
    const PublishedFrame& first = initial.frame();
    // Not stereo-matched: no feature has a depth.
    test::SyntheticStereoOptions options;
    options.timestamp = 1.0;
    TBinocularMeasurement measurement = test::MakeSyntheticStereoFrame(options).measurement;
    ASSERT_EQ(NumDepths(measurement), 0u);
    const size_t count = 30;
    PublishStereoFrameTracked(first, std::move(measurement), count);
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    ExpectPublished(published[0], /*new_map=*/false, 1, {first.frame_id.value + 1});
    EXPECT_EQ(published[0].count, 0u);
    EXPECT_EQ(published[0].num_added_observations, 0u);
    EXPECT_EQ(published[0].frame().observed_points.size(), count);
    EXPECT_EQ(mapper_.num_map_points(), initial.count);
}

TEST_F(MapperFrameCreationTest, MonoTriangulatesAgainstNeighbours) {
    UseNoiseFreeSequence();
    SetUpMonoMap();
    const size_t second_index = next_ - 1;
    // A noise-free initialization maps every scene point, so Frames 0 and 1
    // leave nothing to triangulate. Frame 2 tracks half of the points; the
    // third Frame tracks a different half and triangulates, against Frame 2,
    // the points neither tracks. Frames are 5 sequence frames apart: a 0.25
    // baseline at depths 4-8, about 4 % parallax.
    const size_t frame_2 = second_index + 5;
    const size_t third = second_index + 10;
    ASSERT_LT(third, sequence_.frames.size());
    PublishFrameTracked(1, frame_2, TrackedPairs(frame_2, num_points_ / 2), map::FrameId{1},
                        std::nullopt, MapPose(frame_2));
    Drain();
    const auto before = TakeCaptured();
    ASSERT_EQ(before.size(), 2u);
    const map::FrameId frame_2_id = before[0].frame().frame_id;
    const size_t num_points = mapper_.num_map_points();

    // The mapped points the third frame sees, all but the first half.
    size_t num_visible = 0;
    for (const Eigen::Index scene : sequence_.frames[third].point_of_feature) {
        num_visible += scene >= 0 && point_of_scene_[static_cast<size_t>(scene)] >= 0;
    }
    std::vector<Pair> pairs = TrackedPairs(third, num_visible);
    pairs.erase(pairs.begin(), pairs.begin() + static_cast<std::ptrdiff_t>(num_visible / 2));
    PublishFrameTracked(1, third, pairs, frame_2_id, 20, MapPose(third));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    const Published& created = published[0];
    const PublishedFrame& added_frame = created.frame();
    ASSERT_GT(created.count, 0u);
    EXPECT_EQ(created.start_id, map::MapPointId{static_cast<uint32_t>(num_points)});
    EXPECT_EQ(mapper_.num_map_points(), num_points + created.count);
    ASSERT_EQ(added_frame.observed_points.size(), pairs.size() + created.count);
    ASSERT_EQ(created.num_added_observations, created.count);

    // Sequence frame of each existing Frame.
    const auto sequence_index = [&](map::FrameId frame) -> size_t {
        return frame == first_.frame_id ? 0 : frame == second_.frame_id ? second_index : frame_2;
    };
    const auto& scene_in_third = sequence_.frames[third].point_of_feature;
    std::vector<int> in_frame(created.count, 0);
    std::vector<int> in_neighbour(created.count, 0);
    for (size_t k = pairs.size(); k < added_frame.observed_points.size(); ++k) {
        const uint32_t i = added_frame.observed_points[k].value - created.start_id.value;
        ASSERT_LT(i, created.count) << "observation " << k;
        ++in_frame[i];
        const Eigen::Index scene = scene_in_third[added_frame.observed_features[k]];
        ASSERT_GE(scene, 0) << "observation " << k;
        const geometry::TVector3DF truth = map_scale_ * sequence_.points_w.col(scene);
        EXPECT_LT((created.positions.col(i) - truth).norm(), 1e-6) << "observation " << k;
        EXPECT_EQ(created.point_descriptors.col(i),
                  added_frame.descriptors.col(added_frame.observed_features[k]))
            << "observation " << k;
        EXPECT_NEAR(created.normals.col(i).norm(), 1.0, 1e-9) << "observation " << k;
        // The neighbour's half observes the same scene point.
        for (const events::TransactionObservation& added : created.added_observations) {
            if (added.point == added_frame.observed_points[k]) {
                ++in_neighbour[i];
                const auto& scene_in_neighbour =
                    sequence_.frames[sequence_index(added.frame)].point_of_feature;
                EXPECT_EQ(scene_in_neighbour[added.feature], scene) << "observation " << k;
                EXPECT_EQ(added.depth, -1.0);
                EXPECT_EQ(added.right_index, -1);
            }
        }
    }
    for (uint32_t i = 0; i < created.count; ++i) {
        EXPECT_EQ(in_frame[i], 1) << "new point " << i;
        EXPECT_EQ(in_neighbour[i], 1) << "new point " << i;
    }
}

TEST_F(MapperFrameCreationTest, ReplicaStaysConsistent) {
    UseNoiseFreeSequence();
    SetUpMonoMap();
    ExpectReplicaConsistent();
    const size_t second_index = next_ - 1;
    size_t num_added = 0;
    // Three more Frames, each weak against Frame 1 (20 inliers).
    for (size_t index = second_index + 4; index <= second_index + 12; index += 4) {
        ASSERT_LT(index, sequence_.frames.size());
        PublishFrameTracked(1, index, TrackedPairs(index, num_points_ / 2), map::FrameId{1}, 20,
                            MapPose(index));
        Drain();
        const auto published = TakeCaptured();
        ASSERT_EQ(published.size(), 2u) << "frame " << index;
        num_added += published[0].num_added_observations;
        ExpectReplicaConsistent();
        // Every new point's reference is (new Frame, its feature there).
        const PublishedFrame& frame = published[0].frame();
        for (size_t k = 0; k < frame.observed_points.size(); ++k) {
            const map::MapPointId point = frame.observed_points[k];
            if (point < published[0].start_id) {
                continue;
            }
            const auto& reference = replica().Reference(point);
            ASSERT_TRUE(reference.has_value());
            EXPECT_EQ(reference->keyframe, frame.frame_id);
            EXPECT_EQ(reference->feature, static_cast<Eigen::Index>(frame.observed_features[k]));
        }
    }
    EXPECT_EQ(mapper_.num_keyframes(), 5u);
    EXPECT_GT(num_added, 0u);
}

TEST_F(MapperFrameCreationTest, StereoNeighbourObservationCarriesDepth) {
    // A stereo sequence moving 0.05 per frame, with enough features to
    // initialize; the map's world is frame 0's left camera, metric.
    test::SyntheticStereoSequenceOptions options;
    options.trajectory.num_points = 600;
    const test::SyntheticStereoSequence stereo = test::MakeSyntheticStereoSequence(options);
    const auto matched = [&](size_t index) {
        TBinocularMeasurement measurement = stereo.frames[index].measurement;
        measurement.SetStereoMatches(
            features::MatchStereo<Extractor>(measurement, features::StereoMatcherOptions{}));
        return measurement;
    };
    PublishInitializationMeasurement(matched(0));
    Drain();
    const auto initial = TakeCaptured();
    ASSERT_EQ(initial.size(), 1u);
    const PublishedFrame& first = initial[0].frame();
    std::vector<int64_t> point_of_scene(static_cast<size_t>(stereo.points_w.cols()), -1);
    for (size_t o = 0; o < first.observed_points.size(); ++o) {
        const Eigen::Index scene = stereo.frames[0].point_of_feature[first.observed_features[o]];
        ASSERT_GE(scene, 0);
        point_of_scene[static_cast<size_t>(scene)] = first.observed_points[o].value;
    }

    // Two Frames 0.2 apart (> the 0.11 rig baseline), each tracking 30 points.
    std::vector<TBinocularMeasurement> measurements;
    std::vector<Published> frames;
    for (const size_t index : {size_t{4}, size_t{8}}) {
        measurements.push_back(matched(index));
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = map::MapId{1};
        event->timestamp = measurements.back().timestamp();
        event->pose = stereo.frames[index].T_cw;
        event->measurement = test::Share(measurements.back());
        const auto& scene_of_feature = stereo.frames[index].point_of_feature;
        for (size_t f = 0; f < scene_of_feature.size() && event->observed_points.size() < 30; ++f) {
            const Eigen::Index scene = scene_of_feature[f];
            if (scene >= 0 && point_of_scene[static_cast<size_t>(scene)] >= 0) {
                event->observed_features.push_back(static_cast<uint32_t>(f));
                event->observed_points.push_back(
                    map::MapPointId{static_cast<uint32_t>(point_of_scene[static_cast<size_t>(scene)])});
            }
        }
        event->reference_frame_id = first.frame_id;
        event->num_matches = event->num_inliers = static_cast<uint32_t>(event->observed_points.size());
        tracker_.Notify(std::move(event));
        Drain();
        const auto published = TakeCaptured();
        ASSERT_EQ(published.size(), 2u) << "frame " << index;
        frames.push_back(published[0]);
        ExpectReplicaConsistent();
    }

    // The second Frame's points triangulated against the first: the first's
    // stereo-matched features carry their depth and right index.
    size_t with_depth = 0;
    for (const events::TransactionObservation& added : frames[1].added_observations) {
        if (added.frame != frames[0].frame().frame_id) {
            continue;
        }
        const auto feature = static_cast<int>(added.feature);
        const int right = measurements[0].right_indices()(feature);
        EXPECT_EQ(added.right_index, right) << "feature " << feature;
        EXPECT_EQ(added.depth, right >= 0 ? measurements[0].depth(feature) : -1.0)
            << "feature " << feature;
        with_depth += right >= 0;
    }
    EXPECT_GT(with_depth, 0u);
}

// Fusion and recent-point culling around Frame creation (spec sections 3, 4.1).
class MapperQualityTest : public MapperFrameCreationTest {
protected:
    explicit MapperQualityTest(MapperOptions options = {})
        : MapperFrameCreationTest(options) {}
    MapperQualityTest(MapperOptions options, std::shared_ptr<const TVocabulary> vocabulary)
        : MapperFrameCreationTest(options, std::move(vocabulary)) {}

    using MapperFrameCreationTest::PublishStereoFrameTracked;

    // Sequence frame `index`'s pairs with every initial map point it sees.
    std::vector<Pair> AllTrackedPairs(size_t index) const {
        size_t count = 0;
        for (const Eigen::Index scene : sequence_.frames[index].point_of_feature) {
            count += scene >= 0 && point_of_scene_[static_cast<size_t>(scene)] >= 0;
        }
        return TrackedPairs(index, count);
    }

    // `frame`'s observations as pairs.
    static std::vector<Pair> PairsOf(const PublishedFrame& frame) {
        std::vector<Pair> pairs;
        for (size_t o = 0; o < frame.observed_points.size(); ++o) {
            pairs.push_back({frame.observed_features[o], frame.observed_points[o]});
        }
        return pairs;
    }

    // StereoMeasurement(timestamp) tracked in map 1 at `pose` with `pairs` as
    // its inliers, reported as `num_inliers` against `reference`.
    void PublishStereoFrameTracked(double timestamp, const geometry::RigidTransformation& pose,
                                   const std::vector<Pair>& pairs, map::FrameId reference,
                                   size_t num_inliers) {
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = map::MapId{1};
        event->timestamp = timestamp;
        event->pose = pose;
        event->measurement = test::Share(StereoMeasurement(timestamp));
        for (const Pair& pair : pairs) {
            event->observed_features.push_back(pair.feature);
            event->observed_points.push_back(pair.point);
        }
        event->reference_frame_id = reference;
        event->num_matches = static_cast<uint32_t>(pairs.size());
        event->num_inliers = static_cast<uint32_t>(num_inliers);
        tracker_.Notify(std::move(event));
    }

    // Creates one Frame from `PublishFrameTracked(1, index, ...)`, weak at 20
    // inliers and posed truly, and returns its transaction 1.
    Published CreateMonoFrame(size_t index) {
        const std::vector<Published> published = CreateMonoFrame(index, AllTrackedPairs(index));
        return published.empty() ? Published{} : published[0];
    }
    // Creates one Frame of sequence frame `index` tracking `pairs` at `pose`
    // (the true one by default), weak at 20 inliers against `reference`;
    // returns its two transactions.
    std::vector<Published> CreateMonoFrame(
        size_t index, const std::vector<Pair>& pairs,
        std::optional<geometry::RigidTransformation> pose = std::nullopt,
        map::FrameId reference = map::FrameId{1},
        const std::function<void(const Published&)>& after_each = {}) {
        EXPECT_LT(index, sequence_.frames.size());
        PublishFrameTracked(1, index, pairs, reference, 20, pose.value_or(MapPose(index)));
        Drain();
        std::vector<Published> published = TakeCaptured(after_each);
        EXPECT_EQ(published.size(), 2u) << "frame " << index;
        return published.size() == 2 ? published : std::vector<Published>(2);
    }

    // The mean raw-pixel reprojection error, in the replica, of every
    // observation of the alive points among `points`.
    double MeanReprojectionError(const std::vector<map::MapPointId>& points) const {
        double sum = 0.0;
        size_t count = 0;
        for (const map::MapPointId point : points) {
            if (!replica().IsAlive(point)) continue;
            for (const map::PointObservation& o : replica().covisibility().Observations(point)) {
                const tracking::ReplicaKeyFrame<Extractor>* keyframe = replica().keyframe(o.frame);
                const auto& left = std::get<TMonocularMeasurement>(keyframe->measurement());
                const geometry::TPoint2DF projected = left.camera().Project(
                    keyframe->T_cw.Transform(geometry::TPoint3DF(replica().Position(point))));
                sum += (projected - left.distorted_points().col(o.feature)).norm();
                ++count;
            }
        }
        EXPECT_GT(count, 0u);
        return count > 0 ? sum / static_cast<double>(count) : 0.0;
    }

    // Sync observer of the Mapper: on the first transaction 1 it publishes an
    // urgent FrameTracked into the Mapper's inbox, as the Tracker would while
    // the Mapper is about to run local BA.
    static void PublishUrgentOnFrameCreated(void* ctx, const events::Event& event) {
        auto* self = static_cast<MapperQualityTest*>(ctx);
        if (self->urgent_sent_ || event.type != events::EventType::MapTransaction) return;
        const auto& transaction = static_cast<const TTransaction&>(event);
        if (transaction.new_map || transaction.added_frames.empty()) return;
        self->urgent_sent_ = true;
        auto urgent = std::make_shared<TFrameTracked>();
        urgent->map_id = map::MapId{0};  // an old map's: ignored once drained
        urgent->priority = true;
        self->urgent_.Notify(std::move(urgent));
    }

    events::Observable urgent_;
    bool urgent_sent_ = false;
};
using MapperFusionTest = MapperQualityTest;
using MapperCullingTest = MapperQualityTest;

// Recent-point culling without fusion: a new point keeps exactly the
// observations of its creation unless a Frame tracks it.
class MapperCullingWithoutFusionTest : public MapperQualityTest {
protected:
    MapperCullingWithoutFusionTest()
        : MapperQualityTest(WithoutFusion()) {}

    static MapperOptions WithoutFusion() {
        MapperOptions options;
        options.map_quality.fuse_neighbours_mono = 0;
        options.map_quality.fuse_neighbours_stereo = 0;
        return options;
    }
};

TEST_F(MapperFusionTest, MergesDuplicatePoints) {
    const Published initial = InitializeStereoMap();
    const PublishedFrame& first = initial.frame();
    // Frame 1 sees the same view (the same features) but tracks only 30 of
    // Frame 0's points: its close points duplicate Frame 0's (planted pairs).
    const size_t count = 30;
    PublishStereoFrameTracked(first, StereoMeasurement(1.0), count);
    Drain();
    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    const Published& created = published[0];
    ExpectReplicaConsistent();

    // Each new point lost to Frame 0's point of its feature (one stereo
    // observation each: the lower id wins) and was removed; except on a
    // feature at the edge of the features' bounding box, which projection
    // misses by rounding (the gate's box is that of the raw points).
    const TBinocularMeasurement measurement = StereoMeasurement(1.0);
    const geometry::TPoints2DF& pixels = measurement.left().points();
    const geometry::TPoint2DF low = pixels.rowwise().minCoeff();
    const geometry::TPoint2DF high = pixels.rowwise().maxCoeff();
    const auto on_edge = [&](uint32_t feature) {
        const geometry::TPoint2DF pixel = pixels.col(feature);
        return (pixel.array() == low.array()).any() || (pixel.array() == high.array()).any();
    };
    ASSERT_GT(created.count, 0u);
    size_t num_edge = 0;
    for (const map::MapPointId point : mapper_.covisibility().Points(created.frame().frame_id)) {
        if (point >= created.start_id) {
            ++num_edge;
            ASSERT_EQ(mapper_.covisibility().NumObservations(point), 1u);
            EXPECT_TRUE(on_edge(mapper_.covisibility().Observations(point).front().feature))
                << "unmerged point " << point.value;
        }
    }
    EXPECT_LE(num_edge, 4u);
    ASSERT_EQ(created.replaced_points.size() + num_edge, created.count);
    EXPECT_EQ(created.removed_points.size(), created.replaced_points.size());
    EXPECT_TRUE(created.removed_observations.empty());  // the losers' were all new
    EXPECT_EQ(mapper_.num_map_points(), initial.count + num_edge);
    for (const events::ReplacedPoint& replaced : created.replaced_points) {
        EXPECT_GE(replaced.loser, created.start_id);
        EXPECT_LT(replaced.winner, created.start_id);
        // The union: Frame 0's and Frame 1's observation, through the same
        // feature.
        const std::vector<map::PointObservation>& observations =
            mapper_.covisibility().Observations(replaced.winner);
        ASSERT_EQ(observations.size(), 2u) << "winner " << replaced.winner.value;
        EXPECT_EQ(observations[0].feature, observations[1].feature);
    }
    // Every other pair is one point: Frame 1 observes Frame 0's point on
    // every feature but the edge ones (the rest found by the backward pass).
    std::vector<int64_t> in_first(static_cast<size_t>(first.descriptors.cols()), -1);
    for (size_t o = 0; o < first.observed_points.size(); ++o) {
        in_first[first.observed_features[o]] = first.observed_points[o].value;
    }
    const PublishedFrame& frame = created.frame();
    ASSERT_EQ(frame.observed_points.size(), first.observed_points.size());
    for (size_t o = 0; o < frame.observed_points.size(); ++o) {
        if (frame.observed_points[o] < created.start_id) {
            EXPECT_EQ(frame.observed_points[o].value, in_first[frame.observed_features[o]])
                << "observation " << o;
        }
    }
}

TEST_F(MapperFusionTest, ChainedMergesPublishFinalWinner) {
    // One view, four stereo Frames. Frame 0 maps points A; Frame 1 tracks
    // them all. Frame 2 tracks nothing (no covisible Frame, no fusion) and
    // maps duplicates B. Frame 3 tracks 30 B and 10 A points; its close
    // points N duplicate both. Fusion into Frame 2 (the most covisible)
    // merges N into B (tie, lower id); fusion into Frame 0 then merges B
    // (Frames 2 and 3) into A (Frames 0 and 1; tie, lower id): N -> B -> A.
    const Published initial = InitializeStereoMap();
    const PublishedFrame& first = initial.frame();
    PublishStereoFrameTracked(1.0, first.pose, PairsOf(first), first.frame_id, 20);
    Drain();
    ASSERT_EQ(TakeCaptured().size(), 2u);
    PublishStereoFrameTracked(2.0, first.pose, {}, first.frame_id, 20);
    Drain();
    const auto duplicates = TakeCaptured();
    ASSERT_EQ(duplicates.size(), 2u);
    const PublishedFrame& b_frame = duplicates[0].frame();
    ASSERT_TRUE(duplicates[0].replaced_points.empty());
    ASSERT_GE(b_frame.observed_points.size(), 100u);

    std::vector<Pair> pairs;
    std::vector<bool> used(static_cast<size_t>(first.descriptors.cols()), false);
    for (size_t o = 0; o < 30; ++o) {
        pairs.push_back({b_frame.observed_features[o], b_frame.observed_points[o]});
        used[b_frame.observed_features[o]] = true;
    }
    for (size_t o = 0, a = 0; o < first.observed_points.size() && a < 10; ++o) {
        if (!used[first.observed_features[o]]) {
            pairs.push_back({first.observed_features[o], first.observed_points[o]});
            ++a;
        }
    }
    PublishStereoFrameTracked(3.0, first.pose, pairs, first.frame_id, 20);
    Drain();
    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    const Published& created = published[0];
    ExpectReplicaConsistent();

    // Every record names the final winner, an A point; no winner is a loser.
    size_t chained = 0;
    ASSERT_FALSE(created.replaced_points.empty());
    for (const events::ReplacedPoint& replaced : created.replaced_points) {
        EXPECT_LT(replaced.winner.value, initial.count) << "loser " << replaced.loser.value;
        for (const events::ReplacedPoint& other : created.replaced_points) {
            EXPECT_NE(other.loser, replaced.winner) << "winner " << replaced.winner.value;
        }
        chained += replaced.loser >= created.start_id;  // an N point, merged through B
    }
    EXPECT_GT(chained, 0u);
    // B's own observations in Frame 2 moved to A.
    EXPECT_FALSE(created.removed_observations.empty());
    for (const events::RemovedObservation& removed : created.removed_observations) {
        EXPECT_EQ(removed.frame, b_frame.frame_id);
        EXPECT_FALSE(replica().IsAlive(removed.point));
    }
}

TEST_F(MapperFusionTest, FusionRaisesObservationCounts) {
    SetUpMonoMap();
    const size_t second_index = next_ - 1;
    // Four Frames tracking only initial points: each triangulates new points
    // with 2 observations (the new Frame and one neighbour); only fusion adds
    // more.
    for (size_t k = 1; k <= 4; ++k) {
        const Published created = CreateMonoFrame(second_index + 3 * k);
        ASSERT_EQ(created.frames.size(), 1u);
        ExpectReplicaConsistent();
    }
    // Every point created after the initialization; a culled one counts as
    // under 3.
    const auto total = static_cast<uint32_t>(replica().num_points() - num_points_);
    uint32_t observed_3 = 0;
    for (uint32_t p = static_cast<uint32_t>(num_points_); p < replica().num_points(); ++p) {
        observed_3 += replica().IsAlive(map::MapPointId{p}) &&
                      mapper_.covisibility().NumObservations(map::MapPointId{p}) >= 3;
    }
    ASSERT_GE(total, 10u);
    EXPECT_GE(2 * observed_3, total) << observed_3 << " of " << total;
}

TEST_F(MapperFusionTest, ReplicaStaysConsistent) {
    SetUpMonoMap();
    ExpectReplicaConsistent();
    const size_t second_index = next_ - 1;
    size_t num_removed = 0;
    size_t num_culled = 0;
    for (size_t k = 1; k <= 6; ++k) {
        const size_t index = second_index + 2 * k;
        const std::vector<Published> published = CreateMonoFrame(index, AllTrackedPairs(index));
        ASSERT_EQ(published[0].frames.size(), 1u);
        ExpectReplicaConsistent();
        num_removed += published[0].removed_points.size() + published[1].removed_points.size();
        num_culled += published[1].removed_frames.size();
    }
    EXPECT_EQ(mapper_.num_keyframes(), 8u - num_culled);
    // Something was removed: points, or Frames with their observations.
    EXPECT_GT(num_removed + num_culled, 0u);
}

TEST_F(MapperCullingTest, CountersIncludeSkippedFrameTracked) {
    SetUpMonoMap();
    const std::vector<Pair> pairs = TrackedPairs(next_, num_points_ / 2);
    const map::MapPointId found = pairs[0].point;
    const map::MapPointId unmatched = TrackedPairs(next_, num_points_ / 2 + 1).back().point;
    const uint32_t visible_before = mapper_.NumVisible(found);
    const uint32_t found_before = mapper_.NumFound(found);
    const uint32_t unmatched_found_before = mapper_.NumFound(unmatched);
    const size_t evaluations = mapper_.num_frame_evaluations();

    // Three strong events (no Frame) in one batch: one evaluated, two skipped.
    for (size_t k = 0; k < 3; ++k) {
        PublishFrameTracked(1, next_, pairs, map::FrameId{1}, num_points_, std::nullopt,
                            {found, unmatched});
    }
    ASSERT_TRUE(mapper_.ProcessPending());
    ASSERT_EQ(mapper_.num_frame_evaluations(), evaluations + 1);

    EXPECT_EQ(mapper_.NumVisible(found), visible_before + 3);
    EXPECT_EQ(mapper_.NumFound(found), found_before + 3);
    EXPECT_EQ(mapper_.NumVisible(unmatched), visible_before + 3);
    EXPECT_EQ(mapper_.NumFound(unmatched), unmatched_found_before);
}

TEST_F(MapperCullingTest, RemovesPoorRecentPoint) {
    SetUpMonoMap();
    const size_t index = next_ - 1 + 5;
    const Published created = CreateMonoFrame(index);
    // A point Frame 2 created and kept.
    std::optional<map::MapPointId> poor;
    for (const map::MapPointId point : created.frame().observed_points) {
        if (point >= created.start_id && replica().IsAlive(point)) {
            poor = point;
            break;
        }
    }
    ASSERT_TRUE(poor.has_value());
    ASSERT_EQ(mapper_.NumVisible(*poor), 1u);
    ASSERT_EQ(mapper_.NumFound(*poor), 1u);

    // Four strong events see it but do not match it: found / visible = 1/5.
    for (size_t k = 1; k <= 4; ++k) {
        PublishFrameTracked(1, index + k, AllTrackedPairs(index + k), map::FrameId{1},
                            mapper_.num_map_points(), MapPose(index + k), {*poor});
        Drain();
    }
    ASSERT_TRUE(TakeCaptured().empty());
    ASSERT_EQ(mapper_.NumVisible(*poor), 5u);
    ASSERT_EQ(mapper_.NumFound(*poor), 1u);

    const Published next = CreateMonoFrame(index + 5);
    EXPECT_NE(std::find(next.removed_points.begin(), next.removed_points.end(), *poor),
              next.removed_points.end());
    EXPECT_FALSE(replica().IsAlive(*poor));
    EXPECT_FALSE(mapper_.Reference(*poor).has_value());
    ExpectReplicaConsistent();
}

TEST_F(MapperFusionTest, StereoFusionKeepsPointIdentity) {
    // A stereo sequence moving 0.05 per frame. Frames every 3 frames at their
    // true poses each track only 30 points, so their close points and
    // triangulated points re-map scene points the map already holds: fusion
    // must merge exactly those, never two scene points.
    test::SyntheticStereoSequenceOptions options;
    options.trajectory.num_points = 600;
    const test::SyntheticStereoSequence stereo = test::MakeSyntheticStereoSequence(options);
    const auto matched = [&](size_t index) {
        TBinocularMeasurement measurement = stereo.frames[index].measurement;
        measurement.SetStereoMatches(
            features::MatchStereo<Extractor>(measurement, features::StereoMatcherOptions{}));
        return measurement;
    };
    PublishInitializationMeasurement(matched(0));
    Drain();
    const auto initial = TakeCaptured();
    ASSERT_EQ(initial.size(), 1u);
    // The sequence frame of each Frame; a map point per scene point.
    std::vector<std::pair<map::FrameId, size_t>> frames = {{initial[0].frame().frame_id, 0}};
    std::vector<int64_t> point_of_scene(static_cast<size_t>(stereo.points_w.cols()), -1);
    const auto scene_of = [&](const map::PointObservation& observation) {
        const auto frame = std::find_if(frames.begin(), frames.end(), [&](const auto& f) {
            return f.first == observation.frame;
        });
        return stereo.frames[frame->second].point_of_feature[observation.feature];
    };
    // Every alive point's observations see one scene point; records the
    // points of the scene points.
    const auto expect_one_scene_per_point = [&](size_t index) {
        for (uint32_t p = 0; p < replica().num_points(); ++p) {
            const map::MapPointId point{p};
            if (!replica().IsAlive(point)) continue;
            const std::vector<map::PointObservation>& observations =
                mapper_.covisibility().Observations(point);
            const Eigen::Index scene = scene_of(observations.front());
            for (const map::PointObservation& observation : observations) {
                EXPECT_EQ(scene_of(observation), scene) << "point " << p << ", frame " << index;
            }
            if (scene >= 0) point_of_scene[static_cast<size_t>(scene)] = p;
        }
    };
    expect_one_scene_per_point(0);

    size_t num_merged = 0;
    for (size_t index = 3; index < 24; index += 3) {
        const TBinocularMeasurement measurement = matched(index);
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = map::MapId{1};
        event->timestamp = measurement.timestamp();
        event->pose = stereo.frames[index].T_cw;
        event->measurement = test::Share(measurement);
        const auto& scene_of_feature = stereo.frames[index].point_of_feature;
        for (size_t f = 0; f < scene_of_feature.size() && event->observed_points.size() < 30; ++f) {
            const Eigen::Index scene = scene_of_feature[f];
            if (scene >= 0 && point_of_scene[static_cast<size_t>(scene)] >= 0) {
                event->observed_features.push_back(static_cast<uint32_t>(f));
                event->observed_points.push_back(
                    map::MapPointId{static_cast<uint32_t>(point_of_scene[static_cast<size_t>(scene)])});
            }
        }
        event->reference_frame_id = frames.back().first;
        event->num_matches = event->num_inliers = 20;
        tracker_.Notify(std::move(event));
        Drain();
        const auto published = TakeCaptured();
        ASSERT_EQ(published.size(), 2u) << "frame " << index;
        frames.emplace_back(published[0].frame().frame_id, index);
        num_merged += published[0].replaced_points.size();
        ExpectReplicaConsistent();
        expect_one_scene_per_point(index);
    }
    EXPECT_GT(num_merged, 0u);
}

TEST_F(MapperFusionTest, LoserObservationDroppedWhereWinnerObserved) {
    // One stereo view. Frame 0 maps points A. Frame 1 tracks nothing (no
    // fusion) and maps duplicates B. Frame 2 tracks every A point and, through
    // an unmapped feature m, one B point: it observes both A_s and B_s of
    // scene point s. Fusion into Frame 0 merges B_s into A_s (more weight);
    // B_s's observation in Frame 1 moves to A_s, its one in Frame 2 is
    // dropped since Frame 2 already observes A_s.
    const Published initial = InitializeStereoMap();
    const PublishedFrame& first = initial.frame();
    PublishStereoFrameTracked(1.0, first.pose, {}, first.frame_id, 20);
    Drain();
    const auto duplicates = TakeCaptured();
    ASSERT_EQ(duplicates.size(), 2u);
    const PublishedFrame& b_frame = duplicates[0].frame();
    ASSERT_FALSE(b_frame.observed_points.empty());
    const uint32_t feature_s = b_frame.observed_features[0];
    const map::MapPointId b_s = b_frame.observed_points[0];
    std::optional<map::MapPointId> a_s;
    std::vector<bool> mapped(static_cast<size_t>(first.descriptors.cols()), false);
    for (size_t o = 0; o < first.observed_points.size(); ++o) {
        mapped[first.observed_features[o]] = true;
        if (first.observed_features[o] == feature_s) a_s = first.observed_points[o];
    }
    ASSERT_TRUE(a_s.has_value());
    const auto unmapped = std::find(mapped.begin(), mapped.end(), false);
    ASSERT_NE(unmapped, mapped.end());
    const auto feature_m = static_cast<uint32_t>(unmapped - mapped.begin());

    std::vector<Pair> pairs = PairsOf(first);
    pairs.push_back({feature_m, b_s});
    PublishStereoFrameTracked(2.0, first.pose, pairs, first.frame_id, 20);
    Drain();
    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    const Published& created = published[0];
    ExpectReplicaConsistent();
    const map::FrameId frame_2 = created.frame().frame_id;

    // B_s lost to A_s.
    const auto replaced = std::find_if(created.replaced_points.begin(), created.replaced_points.end(),
                                       [&](const events::ReplacedPoint& r) { return r.loser == b_s; });
    ASSERT_NE(replaced, created.replaced_points.end());
    EXPECT_EQ(replaced->winner, *a_s);
    // Frame 2 observes A_s once, through feature s; B_s and feature m are gone.
    size_t a_s_records = 0;
    for (size_t o = 0; o < created.frame().observed_points.size(); ++o) {
        EXPECT_NE(created.frame().observed_points[o], b_s);
        EXPECT_NE(created.frame().observed_features[o], feature_m);
        if (created.frame().observed_points[o] == *a_s) {
            ++a_s_records;
            EXPECT_EQ(created.frame().observed_features[o], feature_s);
        }
    }
    EXPECT_EQ(a_s_records, 1u);
    // Frame 1's observation moved: removed (Frame 1, B_s), added (Frame 1, s, A_s).
    EXPECT_NE(std::find_if(created.removed_observations.begin(), created.removed_observations.end(),
                           [&](const events::RemovedObservation& r) {
                               return r.frame == b_frame.frame_id && r.point == b_s;
                           }),
              created.removed_observations.end());
    size_t added_a_s = 0;
    for (const events::TransactionObservation& added : created.added_observations) {
        EXPECT_NE(added.frame, frame_2);
        if (added.point == *a_s) {
            ++added_a_s;
            EXPECT_EQ(added.frame, b_frame.frame_id);
            EXPECT_EQ(added.feature, feature_s);
        }
    }
    EXPECT_EQ(added_a_s, 1u);
    // A_s: Frames 0, 1 and 2, each once.
    std::vector<uint32_t> frames;
    for (const map::PointObservation& o : mapper_.covisibility().Observations(*a_s)) {
        frames.push_back(o.frame.value);
        EXPECT_EQ(o.feature, feature_s);
    }
    std::sort(frames.begin(), frames.end());
    EXPECT_EQ(frames, (std::vector<uint32_t>{first.frame_id.value, b_frame.frame_id.value,
                                            frame_2.value}));
}

TEST_F(MapperCullingTest, CountersResetOnMapClear) {
    SetUpMonoMap();
    const map::MapPointId point = TrackedPairs(next_, 1)[0].point;
    PublishFrameTracked(1, next_, TrackedPairs(next_, num_points_ / 2), map::FrameId{1},
                        num_points_, std::nullopt, {point});
    Drain();
    ASSERT_EQ(mapper_.NumVisible(point), 2u);

    PublishTrackingLost(1, sequence_.frames[next_].measurement.timestamp());
    Drain();
    EXPECT_EQ(mapper_.NumVisible(point), 0u);
    EXPECT_EQ(mapper_.NumFound(point), 0u);
    EXPECT_FALSE(mapper_.Reference(point).has_value());

    // Map 2 issues the id again, fresh.
    ASSERT_FALSE(InitializeMono(17).empty());
    ASSERT_GT(mapper_.num_map_points(), point.value);
    EXPECT_EQ(mapper_.NumVisible(point), 1u);
    EXPECT_EQ(mapper_.NumFound(point), 1u);
}

TEST_F(MapperCullingWithoutFusionTest, TrackedObservationCountsBeforeCulling) {
    SetUpMonoMap();
    const size_t second_index = next_ - 1;
    // Frame 2 triangulates new points, 2 observations each (no fusion).
    const size_t index_2 = second_index + 5;
    const Published created = CreateMonoFrame(index_2);
    std::optional<std::pair<map::MapPointId, Eigen::Index>> recent;  // point, scene point
    for (size_t o = 0; o < created.frame().observed_points.size(); ++o) {
        const map::MapPointId point = created.frame().observed_points[o];
        const Eigen::Index scene =
            sequence_.frames[index_2].point_of_feature[created.frame().observed_features[o]];
        if (point >= created.start_id && scene >= 0 && replica().IsAlive(point)) {
            recent.emplace(point, scene);
            break;
        }
    }
    ASSERT_TRUE(recent.has_value());
    // Frame 3 does not track it: still 2 observations.
    CreateMonoFrame(second_index + 7);
    ASSERT_TRUE(replica().IsAlive(recent->first));
    ASSERT_EQ(mapper_.covisibility().NumObservations(recent->first), 2u);

    // Frame 4, 2 Frames after its creation, tracks it: its third observation
    // counts before the culling rule (weighted observations <= 2) is applied.
    const size_t index_4 = second_index + 9;
    std::vector<Pair> pairs = AllTrackedPairs(index_4);
    const auto& scene_of_feature = sequence_.frames[index_4].point_of_feature;
    const auto feature = std::find(scene_of_feature.begin(), scene_of_feature.end(), recent->second);
    ASSERT_NE(feature, scene_of_feature.end());
    pairs.push_back({static_cast<uint32_t>(feature - scene_of_feature.begin()), recent->first});
    PublishFrameTracked(1, index_4, pairs, map::FrameId{1}, 20, MapPose(index_4));
    Drain();
    // Checked after transaction 1: transaction 2 may cull one of its Frames.
    bool alive = false;
    size_t num_observations = 0;
    const auto published = TakeCaptured([&](const Published& p) {
        if (p.frames.empty()) return;
        alive = replica().IsAlive(recent->first);
        num_observations = replica().covisibility().NumObservations(recent->first);
    });
    ASSERT_EQ(published.size(), 2u);
    EXPECT_EQ(std::find(published[0].removed_points.begin(), published[0].removed_points.end(),
                        recent->first),
              published[0].removed_points.end());
    EXPECT_TRUE(alive);
    EXPECT_EQ(num_observations, 3u);
    ExpectReplicaConsistent();
}

// Transaction 2: local BA and Frame culling (spec sections 1, 4.2, 5).

TEST_F(MapperQualityTest, PublishesTwoTransactionsPerFrame) {
    SetUpMonoMap();
    const size_t second_index = next_ - 1;
    for (size_t k = 1; k <= 3; ++k) {
        const std::vector<Published> published =
            CreateMonoFrame(second_index + 3 * k, AllTrackedPairs(second_index + 3 * k));
        ASSERT_EQ(published.size(), 2u);
        const Published& created = published[0];
        const Published& refined = published[1];
        ASSERT_EQ(created.frames.size(), 1u);
        const map::FrameId frame = created.frame().frame_id;
        // Transaction 2 adds nothing: it moves Frames and points and removes.
        EXPECT_EQ(refined.type, events::EventType::MapTransaction);
        EXPECT_EQ(refined.map_id, map::MapId{1});
        EXPECT_FALSE(refined.new_map);
        EXPECT_TRUE(refined.frames.empty());
        EXPECT_EQ(refined.count, 0u);
        EXPECT_EQ(refined.num_added_observations, 0u);
        EXPECT_TRUE(refined.replaced_points.empty());
        // The new Frame is optimized, the first Frame never is.
        ASSERT_TRUE(mapper_.last_local_ba().ran);
        EXPECT_FALSE(mapper_.last_local_ba().interrupted);
        EXPECT_EQ(mapper_.last_local_ba().stage_reached, 2);
        std::vector<map::FrameId> posed;
        for (const auto& [id, pose] : refined.updated_poses) posed.push_back(id);
        EXPECT_NE(std::find(posed.begin(), posed.end(), frame), posed.end());
        EXPECT_EQ(std::find(posed.begin(), posed.end(), first_.frame_id), posed.end());
        EXPECT_FALSE(refined.updated_ids.empty());
        ExpectReplicaConsistent();
        // The replica holds the optimized poses.
        for (const auto& [id, pose] : refined.updated_poses) {
            ASSERT_NE(replica().keyframe(id), nullptr);
            EXPECT_TRUE(replica().keyframe(id)->T_cw.ToMatrix34().isApprox(pose.ToMatrix34()));
        }
    }
}

TEST_F(MapperQualityTest, LocalBaReducesReprojectionError) {
    // The default sequence has 0.5 px noise. The new Frame is posed off its
    // true pose, as a drifting Tracker would pose it.
    SetUpMonoMap();
    const size_t index = next_ + 2;
    const geometry::RigidTransformation truth = MapPose(index);
    const geometry::RigidTransformation pose(
        truth.rotation(), truth.translation() + map_scale_ * geometry::TVector3DF(0.08, -0.05, 0.06));
    std::vector<map::MapPointId> local;
    double before = 0.0;
    const std::vector<Published> published =
        CreateMonoFrame(index, AllTrackedPairs(index), pose, map::FrameId{1}, [&](const Published& p) {
            if (p.frames.empty()) return;
            // After transaction 1: the points of the new Frame.
            local = p.frame().observed_points;
            before = MeanReprojectionError(local);
        });
    ASSERT_FALSE(local.empty());
    ASSERT_TRUE(mapper_.last_local_ba().ran);
    const double after = MeanReprojectionError(local);
    EXPECT_LT(after, before);
    EXPECT_LT(after, 1.0) << "before " << before;
    ExpectReplicaConsistent();
}

TEST_F(MapperQualityTest, RemovesBaOutliers) {
    // The new Frame tracks a map point through an unmatched feature: a gross
    // outlier that local BA removes.
    SetUpMonoMap();
    const size_t index = next_ + 2;
    std::vector<Pair> pairs = AllTrackedPairs(index);
    const map::MapPointId outlier = pairs[0].point;
    pairs[0].feature = ExtraFeatures(index).at(0);
    const std::vector<Published> published = CreateMonoFrame(index, pairs);
    const map::FrameId frame = published[0].frame().frame_id;
    ASSERT_TRUE(mapper_.last_local_ba().ran);
    EXPECT_EQ(mapper_.last_local_ba().stage_reached, 2);
    EXPECT_GE(mapper_.last_local_ba().num_outliers, 1u);
    const auto& removed = published[1].removed_observations;
    EXPECT_NE(std::find_if(removed.begin(), removed.end(),
                           [&](const events::RemovedObservation& r) {
                               return r.frame == frame && r.point == outlier;
                           }),
              removed.end());
    EXPECT_FALSE(mapper_.covisibility().Contains(frame, outlier));
    EXPECT_TRUE(replica().IsAlive(outlier));
    ExpectReplicaConsistent();
}

TEST_F(MapperQualityTest, UrgentEventStopsBa) {
    // The same outlier, but an urgent FrameTracked arrives once transaction 1
    // is out: BA stops before its first iteration and removes nothing.
    subscriptions_.push_back(urgent_.ConnectAsync(mapper_.inbox()));
    SetUpMonoMap();
    subscriptions_.push_back(mapper_.AddSyncObserver(&MapperQualityTest::PublishUrgentOnFrameCreated, this));
    const size_t index = next_ + 2;
    std::vector<Pair> pairs = AllTrackedPairs(index);
    const map::MapPointId outlier = pairs[0].point;
    pairs[0].feature = ExtraFeatures(index).at(0);
    const std::vector<Published> published = CreateMonoFrame(index, pairs);
    ASSERT_TRUE(urgent_sent_);
    const map::FrameId frame = published[0].frame().frame_id;
    const LocalBaSummary& ba = mapper_.last_local_ba();
    ASSERT_TRUE(ba.ran);
    EXPECT_TRUE(ba.interrupted);
    EXPECT_EQ(ba.stage_reached, 0);
    EXPECT_EQ(ba.num_outliers, 0u);
    EXPECT_TRUE(published[1].removed_observations.empty());
    EXPECT_TRUE(mapper_.covisibility().Contains(frame, outlier));
    // Nothing moved: the new Frame keeps its tracked pose.
    for (const auto& [id, pose] : published[1].updated_poses) {
        if (id == frame) EXPECT_TRUE(pose.ToMatrix34().isApprox(MapPose(index).ToMatrix34()));
    }
    EXPECT_EQ(mapper_.inbox().pending_priority(), 0u);  // drained
    ExpectReplicaConsistent();
}

TEST_F(MapperQualityTest, CullsRedundantFrame) {
    // Frames repeating Frame 1's view, each tracking every initial point: once
    // three other Frames observe its points, a repeated Frame is redundant.
    SetUpMonoMap();
    const size_t second_index = next_ - 1;
    std::vector<map::FrameId> removed;
    for (size_t k = 0; k < 4; ++k) {
        const std::vector<Published> published =
            CreateMonoFrame(second_index, AllTrackedPairs(second_index));
        const map::FrameId frame = published[0].frame().frame_id;
        for (const map::FrameId culled : published[1].removed_frames) {
            EXPECT_NE(culled, frame);
            EXPECT_NE(culled, first_.frame_id);
            EXPECT_EQ(replica().keyframe(culled), nullptr);
            removed.push_back(culled);
        }
        ExpectReplicaConsistent();
    }
    EXPECT_FALSE(removed.empty());
    EXPECT_EQ(mapper_.num_keyframes(), 2 + 4 - removed.size());
    EXPECT_NE(replica().keyframe(first_.frame_id), nullptr);
}

TEST_F(MapperQualityTest, CullsFrameHoldingReferences) {
    // Frame F triangulates points, which take F as their reference. Frames
    // repeating F's view track all of F's points until F is redundant; its
    // points' references move to their observation in the smallest remaining
    // Frame, on both sides.
    SetUpMonoMap();
    const size_t index = next_ + 5;
    const Published created = CreateMonoFrame(index);
    const map::FrameId f = created.frame().frame_id;
    std::vector<map::MapPointId> referenced;
    for (const map::MapPointId point : created.frame().observed_points) {
        const auto reference = mapper_.Reference(point);
        if (reference && reference->keyframe == f) referenced.push_back(point);
    }
    ASSERT_FALSE(referenced.empty());
    const std::vector<Pair> pairs = PairsOf(created.frame());

    bool culled = false;
    for (size_t k = 0; k < 4 && !culled; ++k) {
        const std::vector<Published> published = CreateMonoFrame(index, pairs, std::nullopt, f);
        const auto& frames = published[1].removed_frames;
        culled = std::find(frames.begin(), frames.end(), f) != frames.end();
        ExpectReplicaConsistent();
    }
    ASSERT_TRUE(culled);
    EXPECT_EQ(replica().keyframe(f), nullptr);
    size_t num_alive = 0;
    for (const map::MapPointId point : referenced) {
        if (!replica().IsAlive(point)) continue;
        ++num_alive;
        const auto reference = mapper_.Reference(point);
        ASSERT_TRUE(reference.has_value());
        EXPECT_NE(reference->keyframe, f);
        map::FrameId smallest = reference->keyframe;
        for (const map::PointObservation& o : mapper_.covisibility().Observations(point)) {
            smallest = std::min(smallest, o.frame);
        }
        EXPECT_EQ(reference->keyframe, smallest) << "point " << point.value;
    }
    EXPECT_GT(num_alive, 0u);
}

TEST_F(MapperQualityTest, SkipsBaWithoutFreeFrames) {
    // A Frame created without any observation: nothing to optimize.
    SetUpMonoMap();
    const size_t index = next_ + 2;
    const std::vector<Published> published = CreateMonoFrame(index, {});
    EXPECT_TRUE(published[0].frame().observed_points.empty());
    EXPECT_FALSE(mapper_.last_local_ba().ran);
    // Transaction 2 is still published, empty.
    const Published& refined = published[1];
    EXPECT_EQ(refined.type, events::EventType::MapTransaction);
    EXPECT_TRUE(refined.frames.empty());
    EXPECT_EQ(refined.num_other_parts, 0u);
    EXPECT_EQ(refined.num_updated, 0u);
    EXPECT_EQ(mapper_.num_keyframes(), 3u);
    ExpectReplicaConsistent();
}

// The Mapper's Atlas (map-merging spec section 1): a final loss keeps a map
// of at least atlas.min_frames_to_keep (10) Frames inactive and discards a
// smaller one (ORB-SLAM3 Tracking: CreateMapInAtlas vs ResetActiveMap).
// Frame culling is off, so a map has exactly the Frames created.
class MapperAtlasTest : public MapperQualityTest {
protected:
    MapperAtlasTest()
        : MapperQualityTest(WithoutFrameCulling()) {}

    static MapperOptions WithoutFrameCulling() {
        MapperOptions options;
        options.map_quality.redundancy_ratio = 1.1;  // no Frame is ever redundant
        return options;
    }

    // A monocular map 1 of `num_frames` Frames: the initial two, then one per
    // sequence frame from next_. next_ is left after the last one used.
    void GrowMonoMap(size_t num_frames) {
        SetUpMonoMap();
        ASSERT_FALSE(HasFatalFailure());
        while (mapper_.num_keyframes() < num_frames) {
            ASSERT_LT(next_, sequence_.frames.size());
            CreateMonoFrame(next_++);
            ASSERT_FALSE(HasFailure());
        }
        ASSERT_EQ(mapper_.num_keyframes(), num_frames);
        ASSERT_EQ(mapper_.num_keyframes(map::MapId{1}), num_frames);
    }

    // A final TrackingLost of the active map `map`, which the Mapper keeps
    // inactive: one transaction of that map with no active map and nothing
    // else.
    void DeactivateMap(uint32_t map, double timestamp) {
        PublishTrackingLost(map, timestamp);
        Drain();
        const std::vector<Published> published = TakeCaptured();
        ASSERT_EQ(published.size(), 1u);
        const Published& actual = published[0];
        EXPECT_EQ(actual.type, events::EventType::MapTransaction);
        EXPECT_EQ(actual.map_id, map::MapId{map});
        EXPECT_TRUE(actual.removed_maps.empty());
        EXPECT_FALSE(actual.active_map.has_value());
        EXPECT_FALSE(actual.new_map);
        EXPECT_TRUE(actual.frames.empty());
        EXPECT_EQ(actual.count, 0u);
        EXPECT_EQ(actual.num_updated, 0u);
        EXPECT_EQ(actual.num_added_observations, 0u);
        EXPECT_EQ(actual.num_other_parts, 0u);
        EXPECT_TRUE(actual.parent_changes.empty());
        EXPECT_TRUE(actual.added_loop_edges.empty());
    }

    // The replica fed every transaction mirrors the Mapper's Atlas: the same
    // live maps, each with the same Frames, and the same active map.
    void ExpectReplicaMirrorsAtlas() const {
        EXPECT_EQ(atlas_.map_ids(), mapper_.map_ids());
        EXPECT_EQ(atlas_.active_map(), mapper_.active_map());
        for (const map::MapId id : mapper_.map_ids()) {
            ASSERT_NE(atlas_.map(id), nullptr) << "map " << id.value;
            EXPECT_EQ(atlas_.map(id)->num_keyframes(), mapper_.num_keyframes(id)) << "map " << id.value;
        }
    }

    void PublishLoop(uint32_t map, map::FrameId query, map::FrameId match) {
        auto event = std::make_shared<events::LoopDetected>();
        event->map_id = event->query_map_id = event->match_map_id = map::MapId{map};
        event->query_frame = query;
        event->match_frame = match;
        event->S_query_match = geometry::Sim3{};
        tracker_.Notify(std::move(event));
    }
    // An identity global-BA result of map `map` at epoch 0 for `frames`.
    void PublishGlobalBa(uint32_t map, const std::vector<PublishedFrame>& frames) {
        auto event = std::make_shared<events::GlobalBAFinished>();
        event->map_id = map::MapId{map};
        event->map_epoch = 0;
        for (const PublishedFrame& frame : frames) event->frame_poses.emplace_back(frame.frame_id, frame.pose);
        event->priority = true;
        tracker_.Notify(std::move(event));
    }

};

// Review Focus 5 (with DiscardsSmallMap): 10 Frames are enough to keep.
TEST_F(MapperAtlasTest, KeepsMapsFromTenFrames) {
    GrowMonoMap(10);
    ASSERT_FALSE(HasFatalFailure());
    const size_t num_points = mapper_.num_map_points();

    DeactivateMap(1, sequence_.frames[next_].measurement.timestamp());

    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_EQ(mapper_.map_ids(), std::vector<map::MapId>{map::MapId{1}});
    EXPECT_FALSE(mapper_.active_map().has_value());
    EXPECT_EQ(mapper_.num_keyframes(map::MapId{1}), 10u);
    EXPECT_EQ(MapperTestAccess::NumPoints(mapper_, map::MapId{1}), num_points);
    // The active-map counters: none active.
    EXPECT_EQ(mapper_.num_keyframes(), 0u);
    EXPECT_EQ(mapper_.num_map_points(), 0u);
}

// Review Focus 5: 9 Frames, one short of min_frames_to_keep, are discarded.
TEST_F(MapperAtlasTest, DiscardsSmallMap) {
    GrowMonoMap(9);
    ASSERT_FALSE(HasFatalFailure());

    LoseMap(1, sequence_.frames[next_].measurement.timestamp());

    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_TRUE(mapper_.map_ids().empty());
    EXPECT_FALSE(mapper_.active_map().has_value());
    EXPECT_EQ(mapper_.num_keyframes(map::MapId{1}), 0u);
}

TEST_F(MapperAtlasTest, NewMapGetsNextId) {
    GrowMonoMap(10);
    ASSERT_FALSE(HasFatalFailure());
    DeactivateMap(1, sequence_.frames[next_].measurement.timestamp());

    PublishInitializationMeasurement(StereoMeasurement(100.0));
    Drain();
    const std::vector<Published> published = TakeCaptured();

    ASSERT_EQ(published.size(), 1u);
    EXPECT_TRUE(published[0].new_map);
    EXPECT_EQ(published[0].map_id, map::MapId{2});
    EXPECT_EQ(published[0].active_map, map::MapId{2});
    EXPECT_EQ(published[0].start_id, map::MapPointId{0});
    // Frame ids go on across maps: after map 1's (Frames 0..9).
    ASSERT_EQ(published[0].frames.size(), 1u);
    EXPECT_EQ(published[0].frame().frame_id, map::FrameId{10});
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{2});
    EXPECT_EQ(mapper_.active_map(), map::MapId{2});
    EXPECT_EQ(mapper_.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{2}}));
    EXPECT_EQ(mapper_.num_keyframes(map::MapId{1}), 10u);
    EXPECT_EQ(mapper_.num_keyframes(map::MapId{2}), 1u);
    EXPECT_EQ(mapper_.num_keyframes(), 1u);
}

// Inactive maps are frozen: FrameTracked events (weak: one would create a
// Frame; the first, superseded, only counts points), a LoopDetected and a
// GlobalBAFinished of map 1 change nothing, with no map active and with map 2
// active, and no transaction names map 1 after its deactivation.
TEST_F(MapperAtlasTest, InactiveMapIsFrozen) {
    GrowMonoMap(10);
    ASSERT_FALSE(HasFatalFailure());
    DeactivateMap(1, sequence_.frames[next_].measurement.timestamp());
    const size_t num_points = MapperTestAccess::NumPoints(mapper_, map::MapId{1});
    const std::vector<uint32_t> num_visible = MapperTestAccess::NumVisible(mapper_, map::MapId{1});
    const std::vector<uint32_t> num_found = MapperTestAccess::NumFound(mapper_, map::MapId{1});
    const size_t index = next_;
    const auto send_map_1_events = [&] {
        const std::vector<Pair> pairs = AllTrackedPairs(index);
        std::vector<map::MapPointId> visible;
        for (const Pair& pair : pairs) visible.push_back(pair.point);
        PublishFrameTracked(1, index, pairs, map::FrameId{1}, 20, MapPose(index), visible);
        PublishFrameTracked(1, index, pairs, map::FrameId{1}, 20, MapPose(index), visible);
        PublishLoop(1, map::FrameId{9}, first_.frame_id);
        PublishGlobalBa(1, {first_, second_});
        Drain();
    };

    std::vector<Published> published;
    const size_t rejected_before = mapper_.num_global_ba_rejected();
    send_map_1_events();  // no map active
    for (Published& p : TakeCaptured()) published.push_back(std::move(p));
    PublishInitializationMeasurement(StereoMeasurement(100.0));
    Drain();
    for (Published& p : TakeCaptured()) published.push_back(std::move(p));
    ASSERT_EQ(mapper_.active_map(), map::MapId{2});
    const size_t map_2_points = mapper_.num_map_points();
    const std::vector<uint32_t> map_2_visible = MapperTestAccess::NumVisible(mapper_, map::MapId{2});
    const std::vector<uint32_t> map_2_found = MapperTestAccess::NumFound(mapper_, map::MapId{2});
    send_map_1_events();  // map 2 active
    for (Published& p : TakeCaptured()) published.push_back(std::move(p));

    // Only map 2's new map was published.
    ASSERT_EQ(published.size(), 1u);
    EXPECT_EQ(published[0].map_id, map::MapId{2});
    EXPECT_TRUE(published[0].new_map);
    EXPECT_EQ(mapper_.num_global_ba_rejected(), rejected_before + 2);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 0u);
    EXPECT_EQ(mapper_.num_keyframes(map::MapId{1}), 10u);
    EXPECT_EQ(MapperTestAccess::NumPoints(mapper_, map::MapId{1}), num_points);
    EXPECT_EQ(MapperTestAccess::NumVisible(mapper_, map::MapId{1}), num_visible);
    EXPECT_EQ(MapperTestAccess::NumFound(mapper_, map::MapId{1}), num_found);
    // Map 2 is untouched too.
    EXPECT_EQ(mapper_.num_keyframes(map::MapId{2}), 1u);
    EXPECT_EQ(mapper_.num_map_points(), map_2_points);
    EXPECT_EQ(MapperTestAccess::NumVisible(mapper_, map::MapId{2}), map_2_visible);
    EXPECT_EQ(MapperTestAccess::NumFound(mapper_, map::MapId{2}), map_2_found);
    EXPECT_EQ(mapper_.backlog_skipped(), 0u);
}

// An AtlasReplica fed the Mapper's transactions holds its live maps and its
// active map at every step: a final loss no longer clears replicas.
TEST_F(MapperAtlasTest, ReplicasMirrorAtlas) {
    GrowMonoMap(10);
    ASSERT_FALSE(HasFatalFailure());
    ExpectReplicaMirrorsAtlas();

    DeactivateMap(1, sequence_.frames[next_].measurement.timestamp());  // kept
    ExpectReplicaMirrorsAtlas();
    ASSERT_NE(atlas_.map(map::MapId{1}), nullptr);
    EXPECT_EQ(atlas_.map(map::MapId{1})->num_keyframes(), 10u);

    InitializeStereo(100.0);
    ExpectReplicaMirrorsAtlas();
    LoseMap(2, 101.0);  // 1 Frame: discarded
    ExpectReplicaMirrorsAtlas();
    EXPECT_EQ(atlas_.map(map::MapId{2}), nullptr);

    InitializeStereo(102.0);
    ExpectReplicaMirrorsAtlas();
    EXPECT_EQ(mapper_.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{3}}));
    EXPECT_EQ(atlas_.active_map(), map::MapId{3});
}

// The map graph (loop-closing spec section 1): each Frame's parent in the
// spanning tree and its loop edges, published to the replica. Without fusion
// by default: fusion makes Frames 0 and 1 observe nearly every point, so
// every parent would be Frame 0.
class MapperGraphTest : public MapperQualityTest {
protected:
    explicit MapperGraphTest(MapperOptions options = WithoutFusion())
        : MapperQualityTest(options) {}

    static MapperOptions WithoutFusion() {
        MapperOptions options;
        options.map_quality.fuse_neighbours_mono = 0;
        options.map_quality.fuse_neighbours_stereo = 0;
        return options;
    }

    // The replica's spanning tree: the map's first Frame is its root, every
    // other Frame has an alive parent that lists it as a child, and every
    // Frame reaches the root.
    void ExpectSpanningTree() const {
        const std::vector<map::FrameId> frames = replica().keyframe_ids();
        for (const map::FrameId frame : frames) {
            const std::optional<map::FrameId> parent = replica().parent(frame);
            if (frame == first_.frame_id) {
                EXPECT_FALSE(parent.has_value());
                continue;
            }
            ASSERT_TRUE(parent.has_value()) << "orphan Frame " << frame.value;
            EXPECT_NE(replica().keyframe(*parent), nullptr) << "Frame " << frame.value;
            EXPECT_TRUE(replica().children(*parent).contains(frame)) << "Frame " << frame.value;
            map::FrameId ancestor = frame;
            for (size_t step = 0; step < frames.size() && ancestor != first_.frame_id; ++step) {
                ancestor = replica().parent(ancestor).value_or(ancestor);
            }
            EXPECT_EQ(ancestor, first_.frame_id) << "Frame " << frame.value;
        }
    }

    // Initializes a stereo map from StereoMeasurement(0.0); its Frame is first_.
    void SetUpStereoMap() { first_ = InitializeStereoMap().frame(); }

    // A Frame of StereoMeasurement(timestamp) at first_'s pose (the same
    // view), tracking `pairs` against `reference`, weak at 20 inliers; its own
    // features without a point give it close points no other Frame has.
    // Returns its two transactions.
    std::vector<Published> CreateStereoFrame(
        double timestamp, const std::vector<Pair>& pairs, map::FrameId reference,
        const std::function<void(const Published&)>& after_each = {}) {
        PublishStereoFrameTracked(timestamp, first_.pose, pairs, reference, 20);
        Drain();
        std::vector<Published> published = TakeCaptured(after_each);
        EXPECT_EQ(published.size(), 2u) << "t=" << timestamp;
        return published.size() == 2 ? published : std::vector<Published>(2);
    }

    // After a transaction that adds a Frame: its parent is the Frame sharing
    // the most points with it (the lower id on a tie), in the transaction and
    // in the replica. Returns that parent.
    map::FrameId ExpectMostCovisibleParent(const Published& created) const {
        const map::FrameId frame = created.frame().frame_id;
        const auto covisible = replica().covisibility().Covisible(frame, 1);
        EXPECT_EQ(covisible.size(), 1u) << "Frame " << frame.value;
        if (covisible.empty()) return frame;
        const map::FrameId expected = covisible[0].first;
        EXPECT_EQ(created.frame().parent, expected) << "Frame " << frame.value;
        EXPECT_EQ(replica().parent(frame), expected) << "Frame " << frame.value;
        EXPECT_TRUE(replica().children(expected).contains(frame)) << "Frame " << frame.value;
        return expected;
    }
};

// With fusion: the parent counts the points fused into the new Frame.
class MapperGraphFusionTest : public MapperGraphTest {
protected:
    MapperGraphFusionTest()
        : MapperGraphTest(MapperOptions{}) {}
};

TEST_F(MapperGraphFusionTest, ParentIsMostCovisibleAfterFusion) {
    SetUpMonoMap();
    const size_t index = next_ + 5;
    size_t num_checked = 0;
    const auto check = [&](const Published& published) {
        if (published.frames.empty()) return;
        ExpectMostCovisibleParent(published);
        ++num_checked;
    };
    const std::vector<Published> created =
        CreateMonoFrame(index, AllTrackedPairs(index), std::nullopt, map::FrameId{1}, check);
    CreateMonoFrame(index, PairsOf(created[0].frame()), std::nullopt,
                    created[0].frame().frame_id, check);
    EXPECT_EQ(num_checked, 2u);
    ExpectReplicaConsistent();
    ExpectSpanningTree();
}

TEST_F(MapperGraphTest, NewMapFramesAreConnected) {
    // Mono: the second Frame's parent is the first, the root.
    SetUpMonoMap();
    EXPECT_FALSE(first_.parent.has_value());
    EXPECT_EQ(second_.parent, first_.frame_id);
    EXPECT_EQ(initial_.map_epoch, 0u);
    ExpectSpanningTree();
}

TEST_F(MapperGraphTest, ParentIsMostCovisibleAtCreation) {
    // Frame A tracks half of Frame 0's points and adds its close points;
    // Frames B and C track all of A's points: A shares the most with both
    // (B also adds close points of its own, which C does not track).
    SetUpStereoMap();
    EXPECT_FALSE(first_.parent.has_value());
    std::vector<map::FrameId> parents;
    const auto check = [&](const Published& published) {
        EXPECT_EQ(published.map_epoch, 0u);
        if (!published.frames.empty()) parents.push_back(ExpectMostCovisibleParent(published));
    };
    std::vector<Pair> pairs = PairsOf(first_);
    pairs.resize(pairs.size() / 2);
    const PublishedFrame a_part = CreateStereoFrame(1.0, pairs, first_.frame_id, check)[0].frame();
    const map::FrameId a = a_part.frame_id;
    const std::vector<Pair> pairs_a = PairsOf(a_part);
    CreateStereoFrame(2.0, pairs_a, a, check);
    CreateStereoFrame(3.0, pairs_a, a, check);
    EXPECT_EQ(parents, (std::vector<map::FrameId>{first_.frame_id, a, a}));
    ExpectReplicaConsistent();
    ExpectSpanningTree();
}

TEST_F(MapperGraphTest, ParentWithoutSharedPointsIsNewestFrame) {
    // ORB-SLAM3 leaves such a Frame without a parent; ours takes the newest
    // Frame, so the tree stays connected.
    SetUpMonoMap();
    const std::vector<Published> published = CreateMonoFrame(next_ + 2, {});
    ASSERT_TRUE(published[0].frame().observed_points.empty());
    EXPECT_EQ(published[0].frame().parent, second_.frame_id);
    ExpectReplicaConsistent();
    ExpectSpanningTree();
}

TEST_F(MapperGraphTest, CulledFrameChildrenAreReparented) {
    // Frame A tracks half of Frame 0's points and adds its close points;
    // Frames B, C, D track all of A's points, so they are A's children, until
    // A is redundant. Its children are re-parented to A's parent or to an
    // already re-parented sibling, the pair sharing the most points first
    // (ORB-SLAM3 KeyFrame::SetBadFlag).
    SetUpStereoMap();
    std::vector<Pair> pairs = PairsOf(first_);
    pairs.resize(pairs.size() / 2);
    const PublishedFrame a_part = CreateStereoFrame(1.0, pairs, first_.frame_id)[0].frame();
    const map::FrameId a = a_part.frame_id;
    const std::vector<Pair> pairs_a = PairsOf(a_part);

    // The replica's parent and children per Frame before each transaction.
    std::map<map::FrameId, std::optional<map::FrameId>> parent_before;
    std::map<map::FrameId, std::set<map::FrameId>> children_before;
    const auto snapshot = [&] {
        parent_before.clear();
        children_before.clear();
        for (const map::FrameId frame : replica().keyframe_ids()) {
            parent_before[frame] = replica().parent(frame);
            children_before[frame] = replica().children(frame);
        }
    };
    snapshot();
    size_t num_reparented = 0;
    std::vector<std::pair<map::FrameId, map::FrameId>> changes;
    const auto check = [&](const Published& published) {
        ExpectSpanningTree();
        for (const map::FrameId culled : published.removed_frames) {
            for (const map::FrameId child : children_before[culled]) {
                if (replica().keyframe(child) == nullptr) continue;  // culled too
                EXPECT_TRUE(std::any_of(published.parent_changes.begin(),
                                        published.parent_changes.end(),
                                        [&](const auto& change) { return change.first == child; }))
                    << "child " << child.value << " of culled Frame " << culled.value;
                ++num_reparented;
            }
        }
        // One culled Frame: each new parent is its parent or a sibling
        // re-parented before.
        if (published.removed_frames.size() == 1) {
            const map::FrameId culled = published.removed_frames[0];
            std::set<map::FrameId> candidates = {*parent_before[culled]};
            for (const auto& [child, parent] : published.parent_changes) {
                EXPECT_TRUE(children_before[culled].contains(child)) << "Frame " << child.value;
                EXPECT_TRUE(candidates.contains(parent)) << "Frame " << child.value;
                candidates.insert(child);
            }
        }
        changes.insert(changes.end(), published.parent_changes.begin(),
                       published.parent_changes.end());
        snapshot();
    };
    std::vector<map::FrameId> children;
    bool culled = false;
    for (size_t k = 0; k < 6 && !culled; ++k) {
        const std::vector<Published> published =
            CreateStereoFrame(2.0 + static_cast<double>(k), pairs_a, a, check);
        children.push_back(published[0].frame().frame_id);
        EXPECT_EQ(published[0].frame().parent, a);
        const auto& frames = published[1].removed_frames;
        culled = std::find(frames.begin(), frames.end(), a) != frames.end();
        ExpectReplicaConsistent();
    }
    ASSERT_TRUE(culled);
    EXPECT_EQ(num_reparented, children.size());
    // All children share as much with Frame 0, so the lowest id goes there
    // first; each later one shares more with that sibling than with Frame 0,
    // the lowest sibling id on a tie.
    ASSERT_EQ(children.size(), 3u);
    EXPECT_EQ(changes, (std::vector<std::pair<map::FrameId, map::FrameId>>{
                           {children[0], first_.frame_id},
                           {children[1], children[0]},
                           {children[2], children[0]}}));
}

TEST_F(MapperGraphTest, LoopEdgeFramesAreNotCulled) {
    // As CullsRedundantFrame, with a loop edge between Frame 1 and the first
    // repeating Frame: neither is ever culled, while other Frames are.
    SetUpMonoMap();
    const size_t second_index = next_ - 1;
    const map::FrameId looped =
        CreateMonoFrame(second_index, AllTrackedPairs(second_index))[0].frame().frame_id;
    MapperTestAccess::PublishLoopEdge(mapper_, second_.frame_id, looped);
    const std::vector<Published> edge = TakeCaptured();
    ASSERT_EQ(edge.size(), 1u);
    EXPECT_EQ(edge[0].added_loop_edges,
              (std::vector<std::pair<map::FrameId, map::FrameId>>{{second_.frame_id, looped}}));
    EXPECT_EQ(edge[0].map_epoch, 0u);
    EXPECT_EQ(replica().loop_edges(second_.frame_id), std::set<map::FrameId>{looped});
    EXPECT_EQ(replica().loop_edges(looped), std::set<map::FrameId>{second_.frame_id});
    ExpectReplicaConsistent();

    std::vector<map::FrameId> removed;
    for (size_t k = 0; k < 6; ++k) {
        const std::vector<Published> published =
            CreateMonoFrame(second_index, AllTrackedPairs(second_index));
        for (const map::FrameId culled : published[1].removed_frames) {
            EXPECT_NE(culled, second_.frame_id);
            EXPECT_NE(culled, looped);
            removed.push_back(culled);
        }
        ExpectReplicaConsistent();
    }
    EXPECT_FALSE(removed.empty());
    EXPECT_NE(replica().keyframe(second_.frame_id), nullptr);
    EXPECT_NE(replica().keyframe(looped), nullptr);
    EXPECT_EQ(replica().loop_edges(looped), std::set<map::FrameId>{second_.frame_id});
}

// Bag of words. The Mapper's Frames get their BowVector and FeatureVector
// from the vocabulary, and triangulation takes its candidates from shared
// FeatureVector nodes (spec sections 3, 4).

// k = 2, L = 5, four words. The root's children split descriptors by bit 0
// of byte 0: node 1 is all zeros and node 2 has only that bit, so a
// descriptor descends to node 2 iff it has the bit (one Hamming unit closer,
// never a tie). Below each, a chain of single children down to depth 4 ends
// in two leaves of idf weight 1 with random descriptors. With levels_up = 4
// the FeatureVector files every feature under node 1 or node 2.
std::shared_ptr<const TVocabulary> MakeSplitVocabulary() {
    constexpr Eigen::Index kNumNodes = 3 + 2 * (3 + 2);
    std::vector<bow::NodeId> parents{0, 0, 0};
    TDescriptors descriptors = TDescriptors::Zero(Extractor::DescriptorSize, kNumNodes);
    descriptors(0, 2) = 1;
    std::vector<double> weights(kNumNodes, 0.0);
    std::mt19937 rng(7);
    bow::NodeId next = 3;
    for (const bow::NodeId branch : {bow::NodeId{1}, bow::NodeId{2}}) {
        bow::NodeId parent = branch;
        for (int depth = 2; depth <= 4; ++depth) {
            parents.push_back(parent);
            descriptors.col(next) = descriptors.col(branch);
            parent = next++;
        }
        for (int leaf = 0; leaf < 2; ++leaf) {
            parents.push_back(parent);
            descriptors.col(next) = test::detail::RandomDescriptor(rng);
            weights[next++] = 1.0;
        }
    }
    return std::make_shared<const TVocabulary>(
        TVocabulary::FromTree(2, 5, std::move(parents), std::move(descriptors), std::move(weights)));
}

// `measurement` with bit 0 of byte 0 of every descriptor set to `bit`: all
// its features under node 1 (false) or node 2 (true) of
// MakeSplitVocabulary.
TMonocularMeasurement WithSplitBit(const TMonocularMeasurement& measurement, bool bit) {
    TDescriptors descriptors = measurement.descriptors();
    for (Eigen::Index j = 0; j < descriptors.cols(); ++j) {
        descriptors(0, j) = static_cast<uint8_t>(bit ? descriptors(0, j) | 1 : descriptors(0, j) & ~1);
    }
    return TMonocularMeasurement(
        test::TestImageFeatures(geometry::TPoints2DF(measurement.distorted_points()),
                                geometry::TRowVectorF(measurement.strengths()),
                                geometry::TRowVectorF(measurement.angles()), std::move(descriptors),
                                features::TLevels(measurement.levels())),
        measurement.K(), nullptr, measurement.timestamp());
}

// One Mapper run, so that a test can compare runs of differently constructed
// Mappers: a MapperQualityTest driven by hand, outside gtest's fixture
// lifecycle (TestBody is never run).
class MapperBowRun : public MapperQualityTest {
public:
    // Without the vocabulary argument.
    explicit MapperBowRun(MapperOptions options = {})
        : MapperQualityTest(options) {}
    MapperBowRun(MapperOptions options, std::shared_ptr<const TVocabulary> vocabulary)
        : MapperQualityTest(options, std::move(vocabulary)) {}

    void TestBody() override {}

    using MapperFrameCreationTest::CreationOnlyOptions;

    [[nodiscard]] const TMapper& mapper() const { return mapper_; }

    // The default (noisy) mono sequence: the new map, then three Frames each
    // tracking half of the initial points it sees, so that they triangulate,
    // fuse, cull and run local BA. Returns every published transaction.
    std::vector<Published> RunMono() {
        SetUpMonoMap();
        std::vector<Published> published{initial_};
        for (size_t k = 0; k < 3; ++k) {
            const size_t index = next_ + 2 + 3 * k;
            std::vector<Pair> pairs = AllTrackedPairs(index);
            pairs.resize(pairs.size() / 2);
            for (Published& p : CreateMonoFrame(index, pairs)) published.push_back(std::move(p));
        }
        return published;
    }

    // MapperFrameCreationTest.MonoTriangulatesAgainstNeighbours' scene (run
    // with CreationOnlyOptions): returns the number of points the third Frame
    // triangulates against Frame 2. With `split`, every sequence frame's
    // features are under node 1 of MakeSplitVocabulary and the third frame's
    // under node 2: no node is shared, but a descriptor moves by one bit.
    size_t RunTriangulation(bool split) {
        UseNoiseFreeSequence();
        if (split) {
            for (test::SyntheticFrame& frame : sequence_.frames) {
                frame.measurement = WithSplitBit(frame.measurement, false);
            }
        }
        SetUpMonoMap();
        const size_t second_index = next_ - 1;
        const size_t frame_2 = second_index + 5;
        const size_t third = second_index + 10;
        EXPECT_LT(third, sequence_.frames.size());
        if (split) {
            sequence_.frames[third].measurement = WithSplitBit(sequence_.frames[third].measurement, true);
        }
        PublishFrameTracked(1, frame_2, TrackedPairs(frame_2, num_points_ / 2), map::FrameId{1},
                            std::nullopt, MapPose(frame_2));
        Drain();
        const auto before = TakeCaptured();
        EXPECT_EQ(before.size(), 2u);
        if (before.size() != 2) return 0;
        size_t num_visible = 0;
        for (const Eigen::Index scene : sequence_.frames[third].point_of_feature) {
            num_visible += scene >= 0 && point_of_scene_[static_cast<size_t>(scene)] >= 0;
        }
        std::vector<Pair> pairs = TrackedPairs(third, num_visible);
        pairs.erase(pairs.begin(), pairs.begin() + static_cast<std::ptrdiff_t>(num_visible / 2));
        PublishFrameTracked(1, third, pairs, before[0].frame().frame_id, 20, MapPose(third));
        Drain();
        const auto published = TakeCaptured();
        EXPECT_EQ(published.size(), 2u);
        return published.empty() ? 0 : published[0].count;
    }
};

// Same size and coefficients.
template <typename TMatrix>
bool SameMatrix(const TMatrix& a, const TMatrix& b) {
    return a.rows() == b.rows() && a.cols() == b.cols() && a == b;
}

// `a` and `b` are the same transactions, bit for bit.
void ExpectSamePublished(const std::vector<Published>& a, const std::vector<Published>& b) {
    ASSERT_EQ(a.size(), b.size());
    const auto observation = [](const events::TransactionObservation& o) {
        return std::tuple(o.frame, o.feature, o.point, o.depth, o.right_index);
    };
    for (size_t i = 0; i < a.size(); ++i) {
        SCOPED_TRACE("transaction " + std::to_string(i));
        const Published& x = a[i];
        const Published& y = b[i];
        EXPECT_EQ(x.type, y.type);
        EXPECT_EQ(x.map_id, y.map_id);
        EXPECT_EQ(x.new_map, y.new_map);
        EXPECT_EQ(x.start_frame, y.start_frame);
        ASSERT_EQ(x.frames.size(), y.frames.size());
        for (size_t k = 0; k < x.frames.size(); ++k) {
            const PublishedFrame& f = x.frames[k];
            const PublishedFrame& g = y.frames[k];
            EXPECT_EQ(f.frame_id, g.frame_id);
            EXPECT_EQ(f.timestamp, g.timestamp);
            EXPECT_TRUE(SameMatrix(f.pose.ToMatrix34(), g.pose.ToMatrix34()));
            EXPECT_EQ(f.observed_features, g.observed_features);
            EXPECT_EQ(f.observed_points, g.observed_points);
            EXPECT_TRUE(SameMatrix(f.descriptors, g.descriptors));
            EXPECT_TRUE(SameMatrix(f.depths, g.depths));
        }
        EXPECT_EQ(x.start_id, y.start_id);
        EXPECT_EQ(x.count, y.count);
        EXPECT_TRUE(SameMatrix(x.positions, y.positions));
        EXPECT_TRUE(SameMatrix(x.point_descriptors, y.point_descriptors));
        EXPECT_TRUE(SameMatrix(x.normals, y.normals));
        ASSERT_EQ(x.added_observations.size(), y.added_observations.size());
        for (size_t k = 0; k < x.added_observations.size(); ++k) {
            EXPECT_EQ(observation(x.added_observations[k]), observation(y.added_observations[k]));
        }
        EXPECT_EQ(x.updated_ids, y.updated_ids);
        EXPECT_TRUE(SameMatrix(x.updated_positions, y.updated_positions));
        EXPECT_TRUE(SameMatrix(x.updated_descriptors, y.updated_descriptors));
        EXPECT_TRUE(SameMatrix(x.updated_normals, y.updated_normals));
        ASSERT_EQ(x.removed_observations.size(), y.removed_observations.size());
        for (size_t k = 0; k < x.removed_observations.size(); ++k) {
            EXPECT_EQ(x.removed_observations[k].frame, y.removed_observations[k].frame);
            EXPECT_EQ(x.removed_observations[k].point, y.removed_observations[k].point);
        }
        ASSERT_EQ(x.replaced_points.size(), y.replaced_points.size());
        for (size_t k = 0; k < x.replaced_points.size(); ++k) {
            EXPECT_EQ(x.replaced_points[k].loser, y.replaced_points[k].loser);
            EXPECT_EQ(x.replaced_points[k].winner, y.replaced_points[k].winner);
        }
        EXPECT_EQ(x.removed_points, y.removed_points);
        ASSERT_EQ(x.updated_poses.size(), y.updated_poses.size());
        for (size_t k = 0; k < x.updated_poses.size(); ++k) {
            EXPECT_EQ(x.updated_poses[k].first, y.updated_poses[k].first);
            EXPECT_TRUE(SameMatrix(x.updated_poses[k].second.ToMatrix34(),
                                   y.updated_poses[k].second.ToMatrix34()));
        }
        EXPECT_EQ(x.removed_frames, y.removed_frames);
    }
}

TEST(MapperBowTest, FramesGetBowWithVocabulary) {
    const auto vocabulary = MakeSplitVocabulary();
    MapperBowRun run(MapperOptions{}, vocabulary);
    const std::vector<Published> published = run.RunMono();
    ASSERT_FALSE(published.empty());
    // Every Frame ever added, the initial ones included; each still in the
    // map has the BowVector of its descriptors.
    size_t num_checked = 0;
    for (const Published& p : published) {
        for (const PublishedFrame& frame : p.frames) {
            const bow::BowVector* bow = run.mapper().BowVectorOf(frame.frame_id);
            if (bow == nullptr) continue;  // culled
            ++num_checked;
            ASSERT_FALSE(bow->entries.empty()) << "Frame " << frame.frame_id.value;
            const double sum = std::accumulate(
                bow->entries.begin(), bow->entries.end(), 0.0,
                [](double total, const auto& entry) { return total + entry.second; });
            EXPECT_NEAR(sum, 1.0, 1e-12) << "Frame " << frame.frame_id.value;
            bow::BowVector expected;
            bow::FeatureVector features;
            vocabulary->Transform(frame.descriptors, TMapper::kBowLevelsUp, expected, features);
            EXPECT_EQ(bow->entries, expected.entries) << "Frame " << frame.frame_id.value;
        }
    }
    EXPECT_EQ(num_checked, run.mapper().num_keyframes());
    // The initial Frames included (Frame 0 is never culled).
    EXPECT_NE(run.mapper().BowVectorOf(map::FrameId{0}), nullptr);
}

TEST(MapperBowTest, TransactionsCarryBow) {
    const auto vocabulary = MakeSplitVocabulary();
    MapperBowRun run(MapperOptions{}, vocabulary);
    const std::vector<Published> published = run.RunMono();
    size_t num_frames = 0;
    for (const Published& p : published) {
        for (const PublishedFrame& frame : p.frames) {
            ++num_frames;
            SCOPED_TRACE("Frame " + std::to_string(frame.frame_id.value));
            EXPECT_FALSE(frame.bow.entries.empty());
            EXPECT_FALSE(frame.features.nodes.empty());
            bow::BowVector expected;
            bow::FeatureVector features;
            vocabulary->Transform(frame.descriptors, TMapper::kBowLevelsUp, expected, features);
            EXPECT_EQ(frame.bow.entries, expected.entries);
            EXPECT_EQ(frame.features.nodes, features.nodes);
            EXPECT_EQ(frame.features.offsets, features.offsets);
            EXPECT_EQ(frame.features.features, features.features);
            // Equal to the Mapper's own, where the Frame is still in the map.
            if (const bow::BowVector* own = run.mapper().BowVectorOf(frame.frame_id)) {
                EXPECT_EQ(frame.bow.entries, own->entries);
            }
        }
    }
    EXPECT_GT(num_frames, 0u);
}

TEST(MapperBowTest, TransactionsHaveNoBowWithoutVocabulary) {
    MapperBowRun run;
    for (const Published& p : run.RunMono()) {
        for (const PublishedFrame& frame : p.frames) {
            EXPECT_TRUE(frame.bow.entries.empty());
            EXPECT_TRUE(frame.features.nodes.empty());
        }
    }
}

TEST(MapperBowTest, NoBowWithoutVocabulary) {
    MapperBowRun run;
    run.RunMono();
    EXPECT_EQ(run.mapper().BowVectorOf(map::FrameId{0}), nullptr);
}

TEST(MapperBowTest, TriangulationUsesBowPath) {
    // Signal: matches restricted to shared FeatureVector nodes. In the split
    // scene the third Frame shares no node with its neighbours, so with the
    // vocabulary it has no candidate pair and triangulates nothing; without
    // it, brute force triangulates as usual (the split bit costs a distance
    // of 1). Unsplit, the vocabulary still finds the matches.
    const auto vocabulary = MakeSplitVocabulary();
    const MapperOptions options = MapperBowRun::CreationOnlyOptions();
    const size_t brute_force_split = MapperBowRun(options).RunTriangulation(true);
    const size_t bow_split = MapperBowRun(options, vocabulary).RunTriangulation(true);
    const size_t bow_unsplit = MapperBowRun(options, vocabulary).RunTriangulation(false);
    EXPECT_GT(brute_force_split, 0u);
    EXPECT_EQ(bow_split, 0u);
    EXPECT_GT(bow_unsplit, 0u);
}

// k = 1, L = 1: one word under the root. With levels_up = 4 >= L every
// feature is filed under the root, the one node, so SearchByBoW's candidates
// are brute force's.
std::shared_ptr<const TVocabulary> MakeOneNodeVocabulary() {
    std::mt19937 rng(7);
    TDescriptors descriptors = TDescriptors::Zero(Extractor::DescriptorSize, 2);
    descriptors.col(1) = test::detail::RandomDescriptor(rng);
    return std::make_shared<const TVocabulary>(
        TVocabulary::FromTree(1, 1, {0, 0}, std::move(descriptors), {0.0, 1.0}));
}

TEST(MapperBowTest, OneNodeVocabularyMatchesNull) {
    // A vocabulary changes only triangulation's candidate pairs (its bags of
    // words are not published): with one node, the transactions are those of
    // a run without one.
    MapperBowRun without;
    MapperBowRun one_node(MapperOptions{}, MakeOneNodeVocabulary());
    const std::vector<Published> expected = without.RunMono();
    const std::vector<Published> actual = one_node.RunMono();
    ASSERT_NE(one_node.mapper().BowVectorOf(map::FrameId{0}), nullptr);  // the BoW path ran
    // The scene triangulates (and exercises every other step).
    size_t num_new_points = 0;
    for (const Published& p : expected) {
        if (!p.new_map) num_new_points += p.count;
    }
    EXPECT_GT(num_new_points, 0u);
    ExpectSamePublished(expected, actual);
}


// Loop correction (loop-closing spec section 3; ORB-SLAM3 CorrectLoop). A
// stereo scene built through the Mapper's own inbox: Frames E0..E3 map region
// A from the start (E0 initializes the map, at the origin), B0..B3 turn around
// and map region B behind the start, posed with half the drift (about the
// origin), and L0..L3 come back to region A
// through new points, posed with the full drift, so their side of the map is
// drift(true world). A LoopDetected from L3 to E1 with the true similarity
// closes the loop.

constexpr Eigen::Index kLoopRegionPoints = 500;
constexpr size_t kLoopSideFrames = 4;
constexpr size_t kLoopMatch = 1;  // E1

enum class LoopSide : uint8_t { Start, Middle, Return };

geometry::RigidTransformation LoopPose(double x, double y, double z, double yaw_deg) {
    const geometry::TMatrix33F R =
        Eigen::AngleAxis<double>(yaw_deg * std::numbers::pi / 180.0, geometry::TVector3DF::UnitY())
            .toRotationMatrix();
    return {R, -R * geometry::TVector3DF(x, y, z)};
}

geometry::RigidTransformation StartPose(size_t k) {
    return LoopPose(0.1 * static_cast<double>(k), 0.0, 0.0, 0.5 * static_cast<double>(k));
}
geometry::RigidTransformation MiddlePose(size_t k) {
    return LoopPose(0.5 + 0.1 * static_cast<double>(k), 0.0, -0.5, 180.0 + 0.5 * static_cast<double>(k));
}
geometry::RigidTransformation ReturnPose(size_t k) {
    return LoopPose(0.3 + 0.1 * static_cast<double>(k), 0.05, 0.2, 2.0 + 0.5 * static_cast<double>(k));
}

// `fraction` of the drift: 4 deg of rotation and 0.5 m of translation in
// full; scale 1 (stereo).
geometry::Sim3 LoopDrift(double fraction) {
    const geometry::TVector3DF axis = geometry::TVector3DF(0.2, 1.0, -0.1).normalized();
    return {1.0,
            Eigen::AngleAxis<double>(fraction * 4.0 * std::numbers::pi / 180.0, axis).toRotationMatrix(),
            fraction * geometry::TVector3DF(0.4, -0.1, 0.3)};
}

// The map pose of a camera at true pose `T_true` in a map whose world is
// drift(true world): T_true ∘ drift⁻¹.
geometry::RigidTransformation Drifted(const geometry::RigidTransformation& T_true, const geometry::Sim3& drift) {
    const geometry::Sim3 inverse = geometry::Inverse(drift);
    return {T_true.rotation() * inverse.R, T_true.rotation() * inverse.t + T_true.translation()};
}

geometry::Sim3 ToSim3(const geometry::RigidTransformation& T) { return {1.0, T.rotation(), T.translation()}; }

double RotationDeg(const geometry::TMatrix33F& a, const geometry::TMatrix33F& b) {
    return Eigen::AngleAxis<double>(a.transpose() * b).angle() * 180.0 / std::numbers::pi;
}

class MapperLoopTest : public MapperQualityTest {
protected:
    explicit MapperLoopTest(MapperOptions options = SceneOptions(/*cull_frames=*/false))
        : MapperQualityTest(options) {}

    // Every tracked measurement becomes a Frame (need_close as soon as one
    // close point is untracked); no recent point is culled; Frames are culled
    // only if asked.
    static MapperOptions SceneOptions(bool cull_frames) {
        MapperOptions options;
        options.keyframe.close_tracked_max = std::numeric_limits<size_t>::max();
        options.keyframe.close_untracked_min = 0;
        options.map_quality.min_found_ratio = 0.0;
        options.map_quality.min_obs_stereo = 0;
        if (!cull_frames) options.map_quality.redundancy_ratio = 2.0;
        return options;
    }

    struct SceneFrame {
        map::FrameId id;
        LoopSide side;
        geometry::RigidTransformation T_true;
        std::vector<Eigen::Index> scene_of_feature;  // per left feature, -1 for extras
    };

    // Regions A (points 0..499, 2..4 m in front of the origin) and B (points
    // 500..999, in front of MiddlePose(0)); E0 sees more than the 500
    // features a stereo initialization needs.
    void MakeRegions() {
        const geometry::TMatrix33F K_inverse = test::SyntheticStereoIntrinsics().inverse();
        std::uniform_real_distribution<double> pixel(150.0, 874.0);
        std::uniform_real_distribution<double> depth(2.0, 4.0);
        std::uniform_real_distribution<double> angle(0.0, 2.0 * std::numbers::pi);
        points_w_.resize(3, 2 * kLoopRegionPoints);
        for (Eigen::Index i = 0; i < 2 * kLoopRegionPoints; ++i) {
            const geometry::RigidTransformation T_wc = (i < kLoopRegionPoints ? StartPose(0) : MiddlePose(0)).Inverse();
            const geometry::TVector3DF ray(pixel(rng_), pixel(rng_), 1.0);
            points_w_.col(i) = T_wc.Transform(geometry::TPoint3DF(depth(rng_) * (K_inverse * ray)));
            descriptors_.push_back(test::detail::RandomDescriptor(rng_));
            angles_.push_back(angle(rng_));
        }
    }

    // The stereo-matched view of every scene point in front of a camera at
    // `T_true`, plus random extras; scene_of_feature per left feature.
    TBinocularMeasurement View(const geometry::RigidTransformation& T_true, double timestamp,
                               std::vector<Eigen::Index>& out_scene_of_feature) {
        constexpr double kImageSize = 1024.0;
        constexpr size_t kExtras = 20;
        const features::StereoRig rig = test::SyntheticStereoRig();
        const geometry::TMatrix33F K = test::SyntheticStereoIntrinsics();
        std::normal_distribution<double> noise(0.0, 0.5);
        std::uniform_real_distribution<double> any_pixel(0.0, kImageSize);
        std::uniform_real_distribution<double> any_angle(0.0, 2.0 * std::numbers::pi);
        struct Record {
            geometry::TPoint2DF pixel;
            test::detail::SyntheticDescriptor descriptor;
            double angle;
            Eigen::Index scene;
        };
        const auto in_image = [&](const geometry::TPoint2DF& p) {
            return p.x() >= 0.0 && p.x() < kImageSize && p.y() >= 0.0 && p.y() < kImageSize;
        };
        std::vector<Record> left, right;
        for (Eigen::Index i = 0; i < points_w_.cols(); ++i) {
            const geometry::TPoint3DF X = T_true.Transform(geometry::TPoint3DF(points_w_.col(i)));
            const geometry::TPoint3DF X_r = rig.T_rl.Transform(X);
            if (X.z() <= 0.0 || X_r.z() <= 0.0) continue;
            const geometry::TPoint2DF uv = (K * X).hnormalized();
            const geometry::TPoint2DF uv_r = (K * X_r).hnormalized();
            if (!in_image(uv) || !in_image(uv_r)) continue;
            const auto s = static_cast<size_t>(i);
            left.push_back({uv + geometry::TPoint2DF(noise(rng_), noise(rng_)),
                            test::detail::FlipBits(descriptors_[s], 3, rng_), angles_[s], i});
            right.push_back({uv_r + geometry::TPoint2DF(noise(rng_), noise(rng_)),
                             test::detail::FlipBits(descriptors_[s], 3, rng_), angles_[s], i});
        }
        for (std::vector<Record>* image : {&left, &right}) {
            for (size_t e = 0; e < kExtras; ++e) {
                image->push_back({geometry::TPoint2DF(any_pixel(rng_), any_pixel(rng_)),
                                  test::detail::RandomDescriptor(rng_), any_angle(rng_), -1});
            }
        }
        const auto build = [&](std::vector<Record>& records) {
            std::shuffle(records.begin(), records.end(), rng_);
            const auto n = static_cast<Eigen::Index>(records.size());
            geometry::TPoints2DF points(2, n);
            geometry::TRowVectorF feature_angles(n);
            TDescriptors feature_descriptors(Extractor::DescriptorSize, n);
            for (Eigen::Index j = 0; j < n; ++j) {
                const Record& record = records[static_cast<size_t>(j)];
                points.col(j) = record.pixel;
                feature_angles[j] = record.angle;
                feature_descriptors.col(j) = record.descriptor;
            }
            return test::TestImageFeatures(std::move(points), geometry::TRowVectorF::Ones(n),
                                           std::move(feature_angles), std::move(feature_descriptors),
                                           features::TLevels::Zero(n));
        };
        test::TestImageFeatures left_features = build(left);
        test::TestImageFeatures right_features = build(right);
        out_scene_of_feature.clear();
        for (const Record& record : left) out_scene_of_feature.push_back(record.scene);
        TBinocularMeasurement measurement(std::move(left_features), std::move(right_features), rig, timestamp);
        measurement.SetStereoMatches(features::MatchStereo<Extractor>(measurement, features::StereoMatcherOptions{}));
        return measurement;
    }

    // Region A's scene index for a region-A view, B's for a region-B view.
    static LoopSide SideOf(size_t frame_index) {
        return frame_index < kLoopSideFrames ? LoopSide::Start
               : frame_index < 2 * kLoopSideFrames ? LoopSide::Middle
                                                   : LoopSide::Return;
    }

    // The alive point of `side` per scene point, from the Frames of that side
    // in `map` (the replica's active map by default).
    std::vector<std::optional<map::MapPointId>> SidePoints(LoopSide side) const { return SidePoints(side, replica()); }
    std::vector<std::optional<map::MapPointId>> SidePoints(LoopSide side,
                                                           const tracking::MapReplica<Extractor>& map) const {
        std::vector<std::optional<map::MapPointId>> points(static_cast<size_t>(points_w_.cols()));
        for (const SceneFrame& frame : frames_) {
            const tracking::ReplicaKeyFrame<Extractor>* keyframe = map.keyframe(frame.id);
            if (frame.side != side || keyframe == nullptr) continue;
            for (const auto& [feature, point] : keyframe->observations) {
                const Eigen::Index scene = frame.scene_of_feature[feature];
                if (scene >= 0 && map.IsAlive(point)) points[static_cast<size_t>(scene)] = point;
            }
        }
        return points;
    }

    // Builds the scene: E0 initializes the map, every other view becomes a
    // Frame tracking every point its side has mapped and it sees, except one
    // scene point in five (rotating), which it maps anew. Views up to `end`;
    // a later call goes on from there.
    void BuildScene(size_t end = 3 * kLoopSideFrames) {
        if (frames_.empty()) MakeRegions();
        for (size_t k = frames_.size(); k < end; ++k) {
            const LoopSide side = SideOf(k);
            const size_t j = k % kLoopSideFrames;
            const geometry::RigidTransformation T_true =
                side == LoopSide::Start ? StartPose(j) : side == LoopSide::Middle ? MiddlePose(j) : ReturnPose(j);
            const geometry::RigidTransformation T_map =
                side == LoopSide::Start ? T_true : Drifted(T_true, LoopDrift(side == LoopSide::Middle ? 0.5 : 1.0));
            AddSceneView(side, T_true, T_map);
            ASSERT_FALSE(HasFatalFailure()) << "view " << k;
        }
    }

    // Adds view k = frames_.size() of `side`, truly at `T_true`: while the
    // Mapper is Initializing it initializes a map; otherwise it becomes a
    // Frame of scene_map_ at `T_map`, tracking every point its side has
    // mapped (in the active map) and it sees, except one scene point in five
    // (rotating with k), which it maps anew.
    void AddSceneView(LoopSide side, const geometry::RigidTransformation& T_true,
                      const geometry::RigidTransformation& T_map) {
        const size_t k = frames_.size();
        SceneFrame frame{{}, side, T_true, {}};
        const double timestamp = 0.05 * static_cast<double>(k);
        TBinocularMeasurement measurement = View(T_true, timestamp, frame.scene_of_feature);
        if (mapper_.state() == MapperState::Initializing) {
            PublishInitializationMeasurement(std::move(measurement));
            Drain();
            const std::vector<Published> initial = TakeCaptured();
            ASSERT_EQ(initial.size(), 1u);
            frame.id = initial[0].frame().frame_id;
            frames_.push_back(std::move(frame));
            return;
        }
        const std::vector<std::optional<map::MapPointId>> mapped = SidePoints(side);
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = scene_map_;
        event->timestamp = timestamp;
        event->pose = T_map;
        for (size_t f = 0; f < frame.scene_of_feature.size(); ++f) {
            const Eigen::Index scene = frame.scene_of_feature[f];
            if (scene < 0 || static_cast<size_t>(scene) % 5 == k % 5) continue;
            if (const auto& point = mapped[static_cast<size_t>(scene)]) {
                event->observed_features.push_back(static_cast<uint32_t>(f));
                event->observed_points.push_back(*point);
            }
        }
        event->measurement = test::Share(std::move(measurement));
        event->reference_frame_id = frames_.back().id;
        event->num_matches = static_cast<uint32_t>(event->observed_points.size());
        event->num_inliers = 20;
        tracker_.Notify(std::move(event));
        Drain();
        const std::vector<Published> created = TakeCaptured(on_scene_published_);
        ASSERT_EQ(created.size(), 2u) << "view " << k;
        frame.id = created[0].frame().frame_id;
        frames_.push_back(std::move(frame));
    }

    [[nodiscard]] const SceneFrame& Query() const { return frames_.back(); }
    [[nodiscard]] const SceneFrame& Match() const { return frames_[kLoopMatch]; }

    // The query's points paired with the start side's points of the same
    // scene point (as the LoopCloser would match them).
    std::vector<std::pair<map::MapPointId, map::MapPointId>> MatchedPoints() const {
        const std::vector<std::optional<map::MapPointId>> start = SidePoints(LoopSide::Start);
        std::vector<std::pair<map::MapPointId, map::MapPointId>> pairs;
        for (const auto& [feature, point] : replica().keyframe(Query().id)->observations) {
            const Eigen::Index scene = Query().scene_of_feature[feature];
            if (scene >= 0 && start[static_cast<size_t>(scene)]) {
                pairs.emplace_back(point, *start[static_cast<size_t>(scene)]);
            }
        }
        return pairs;
    }

    // The true similarity from the match camera to the query camera.
    [[nodiscard]] geometry::Sim3 TrueS() const { return ToSim3(Query().T_true * Match().T_true.Inverse()); }

    void PublishLoop(map::FrameId query, map::FrameId match, const geometry::Sim3& S,
                     const std::vector<std::pair<map::MapPointId, map::MapPointId>>& matched_points,
                     uint32_t query_map = 1, uint32_t match_map = 1) {
        auto event = std::make_shared<events::LoopDetected>();
        event->map_id = map::MapId{query_map};
        event->query_map_id = map::MapId{query_map};
        event->match_map_id = map::MapId{match_map};
        event->query_frame = query;
        event->match_frame = match;
        event->S_query_match = S;
        event->matched_points = matched_points;
        tracker_.Notify(std::move(event));
    }

    // The loop of the scene: L3 with E1, the true similarity, all pairs.
    void PublishSceneLoop() { PublishLoop(Query().id, Match().id, TrueS(), MatchedPoints()); }

    struct SideError {
        double centre = 0.0;    // mean camera-centre error, m
        double rotation = 0.0;  // mean rotation error, deg
    };
    // The replica's mean pose error of the alive Frames of `side`.
    [[nodiscard]] SideError PoseError(LoopSide side) const {
        SideError error;
        size_t count = 0;
        for (const SceneFrame& frame : frames_) {
            const tracking::ReplicaKeyFrame<Extractor>* keyframe = replica().keyframe(frame.id);
            if (frame.side != side || keyframe == nullptr) continue;
            error.centre += (keyframe->T_cw.Inverse().translation() - frame.T_true.Inverse().translation()).norm();
            error.rotation += RotationDeg(keyframe->T_cw.rotation(), frame.T_true.rotation());
            ++count;
        }
        EXPECT_GT(count, 0u);
        error.centre /= static_cast<double>(std::max<size_t>(count, 1));
        error.rotation /= static_cast<double>(std::max<size_t>(count, 1));
        return error;
    }
    // The replica's mean position error of the points `frame` observes.
    [[nodiscard]] double PointError(const SceneFrame& frame) const {
        double sum = 0.0;
        size_t count = 0;
        for (const auto& [feature, point] : replica().keyframe(frame.id)->observations) {
            const Eigen::Index scene = frame.scene_of_feature[feature];
            if (scene < 0) continue;
            sum += (replica().Position(point) - geometry::TPoint3DF(points_w_.col(scene))).norm();
            ++count;
        }
        EXPECT_GT(count, 0u);
        return sum / static_cast<double>(std::max<size_t>(count, 1));
    }

    // Publishes the scene's loop and returns its one transaction.
    Published CorrectSceneLoop() {
        PublishSceneLoop();
        Drain();
        std::vector<Published> published = TakeCaptured();
        EXPECT_EQ(published.size(), 1u);
        return published.empty() ? Published{} : published[0];
    }

    std::mt19937 rng_{11};
    map::MapId scene_map_{1};  // the map BuildScene's FrameTracked name
    // Runs on each transaction of a view after the first (may be empty).
    std::function<void(const Published&)> on_scene_published_;
    geometry::TPoints3DF points_w_;
    std::vector<test::detail::SyntheticDescriptor> descriptors_;
    std::vector<double> angles_;
    std::vector<SceneFrame> frames_;
};

class MapperLoopCullingTest : public MapperLoopTest {
protected:
    MapperLoopCullingTest()
        : MapperLoopTest(SceneOptions(/*cull_frames=*/true)) {}
};

TEST_F(MapperLoopTest, CorrectsWindowPointsAndGraph) {
    BuildScene();
    ASSERT_EQ(frames_.size(), 3 * kLoopSideFrames);
    ASSERT_EQ(replica().num_keyframes(), 3 * kLoopSideFrames);
    const SideError start_before = PoseError(LoopSide::Start);
    const SideError middle_before = PoseError(LoopSide::Middle);
    const SideError return_before = PoseError(LoopSide::Return);
    const double points_before = PointError(Query());
    // The stereo error of the start side's points (0.5 px at 2..4 m).
    const double match_points = PointError(Match());
    const geometry::RigidTransformation match_before = replica().keyframe(Match().id)->T_cw;

    const auto start = std::chrono::steady_clock::now();
    const Published corrected = CorrectSceneLoop();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    ExpectReplicaConsistent();

    const SideError start_after = PoseError(LoopSide::Start);
    const SideError middle_after = PoseError(LoopSide::Middle);
    const SideError return_after = PoseError(LoopSide::Return);
    const double points_after = PointError(Query());
    const auto change = [](double before, double after) {
        return std::to_string(before) + " -> " + std::to_string(after);
    };
    RecordProperty("return_centre_m", change(return_before.centre, return_after.centre));
    RecordProperty("return_rotation_deg", change(return_before.rotation, return_after.rotation));
    RecordProperty("middle_centre_m", change(middle_before.centre, middle_after.centre));
    RecordProperty("middle_rotation_deg", change(middle_before.rotation, middle_after.rotation));
    RecordProperty("start_centre_m", change(start_before.centre, start_after.centre));
    RecordProperty("query_points_m",
                   change(points_before, points_after) + " (match " + std::to_string(match_points) + ")");
    RecordProperty("correction_ms", std::to_string(ms));

    // The drift before: about 0.5 m and 4 deg at the loop, half in between.
    EXPECT_GT(return_before.centre, 0.3);
    EXPECT_GT(return_before.rotation, 3.0);
    EXPECT_GT(points_before, 0.3);
    EXPECT_GT(middle_before.centre, 0.15);
    // After: the loop is closed (the query observes start points, as exact as
    // their stereo depths), and the middle's centres follow. Not its
    // rotations: the identity-weighted Sim3 edges of ORB-SLAM3's essential
    // graph trade rotation for translation across the two tree edges that
    // alone hold the middle (2 -> 3.2 deg here; a pure translation drift
    // rotates it too).
    EXPECT_LT(return_after.centre, 0.03);
    EXPECT_LT(return_after.rotation, 0.3);
    EXPECT_LT(points_after, 1.2 * match_points);
    EXPECT_LT(middle_after.centre, 0.5 * middle_before.centre);
    EXPECT_LT(start_after.centre, 0.01);
    EXPECT_LT(start_after.rotation, 0.1);
    // The match Frame is fixed.
    const geometry::RigidTransformation& match_after = replica().keyframe(Match().id)->T_cw;
    EXPECT_LT((match_after.translation() - match_before.translation()).norm(), 1e-9);
    EXPECT_LT(RotationDeg(match_after.rotation(), match_before.rotation()), 1e-6);
    EXPECT_EQ(corrected.added_loop_edges,
              (std::vector<std::pair<map::FrameId, map::FrameId>>{{Query().id, Match().id}}));
    EXPECT_EQ(replica().loop_edges(Query().id), std::set<map::FrameId>{Match().id});
    EXPECT_EQ(replica().loop_edges(Match().id), std::set<map::FrameId>{Query().id});
}

TEST_F(MapperLoopTest, FusesLoopPoints) {
    BuildScene();
    const auto pairs = MatchedPoints();
    ASSERT_GE(pairs.size(), 100u);
    const Published corrected = CorrectSceneLoop();
    ExpectReplicaConsistent();
    // Each pair's query point is replaced by its match point, or by the
    // point SearchAndFuse later merged that one into (a start-side duplicate
    // of the same scene point).
    std::map<uint32_t, map::MapPointId> winner_of;
    for (const events::ReplacedPoint& replaced : corrected.replaced_points) {
        winner_of[replaced.loser.value] = replaced.winner;
    }
    for (const auto& [query_point, match_point] : pairs) {
        EXPECT_FALSE(replica().IsAlive(query_point)) << "query point " << query_point.value;
        ASSERT_TRUE(winner_of.contains(query_point.value)) << "query point " << query_point.value;
        map::MapPointId final_point = winner_of[query_point.value];
        EXPECT_TRUE(final_point == match_point || winner_of.contains(match_point.value))
            << "query point " << query_point.value;
        for (size_t step = 0; step < winner_of.size() && winner_of.contains(final_point.value); ++step) {
            final_point = winner_of[final_point.value];
        }
        EXPECT_TRUE(replica().IsAlive(final_point)) << "query point " << query_point.value;
    }
    // SearchAndFuse: the return Frames observe region A through the start
    // side's points, almost no return-side point is left.
    const std::vector<std::optional<map::MapPointId>> start = SidePoints(LoopSide::Start);
    size_t num_start = 0, num_return = 0;
    for (const SceneFrame& frame : frames_) {
        if (frame.side != LoopSide::Return) continue;
        for (const auto& [feature, point] : replica().keyframe(frame.id)->observations) {
            const Eigen::Index scene = frame.scene_of_feature[feature];
            if (scene < 0) continue;
            const bool from_start = std::any_of(frames_.begin(), frames_.end(), [&](const SceneFrame& other) {
                return other.side == LoopSide::Start && replica().keyframe(other.id) != nullptr &&
                       replica().covisibility().Contains(other.id, point);
            });
            ++(from_start ? num_start : num_return);
        }
    }
    RecordProperty("return_observations_start_return", std::to_string(num_start) + " / " + std::to_string(num_return));
    EXPECT_GE(num_start, 9 * num_return);
}

TEST_F(MapperLoopTest, PublishesOneTransactionWithEpoch) {
    BuildScene();
    const Published corrected = CorrectSceneLoop();
    EXPECT_EQ(corrected.type, events::EventType::MapTransaction);
    EXPECT_EQ(corrected.map_id, map::MapId{1});
    EXPECT_FALSE(corrected.new_map);
    EXPECT_TRUE(corrected.frames.empty());
    EXPECT_EQ(corrected.count, 0u);
    EXPECT_EQ(corrected.map_epoch, 1u);
    EXPECT_EQ(replica().map_epoch(), 1u);
    // Every Frame's pose and every alive point.
    EXPECT_EQ(corrected.updated_poses.size(), replica().num_keyframes());
    size_t num_alive = 0;
    for (uint32_t p = 0; p < replica().num_points(); ++p) num_alive += replica().IsAlive(map::MapPointId{p});
    EXPECT_EQ(corrected.updated_ids.size(), num_alive);
    EXPECT_FALSE(corrected.replaced_points.empty());
    EXPECT_FALSE(corrected.added_observations.empty());
    ExpectReplicaConsistent();

    // Later transactions carry the new epoch.
    std::vector<Eigen::Index> scene_of_feature;
    auto event = std::make_shared<TFrameTracked>();
    event->map_id = map::MapId{1};
    event->timestamp = 10.0;
    event->pose = replica().keyframe(Query().id)->T_cw;
    event->measurement = test::Share(View(Query().T_true, 10.0, scene_of_feature));
    event->reference_frame_id = Query().id;
    event->num_inliers = 20;
    event->map_epoch = 1;
    tracker_.Notify(std::move(event));
    Drain();
    const std::vector<Published> created = TakeCaptured();
    ASSERT_EQ(created.size(), 2u);
    EXPECT_EQ(created[0].map_epoch, 1u);
    EXPECT_EQ(created[1].map_epoch, 1u);
}

TEST_F(MapperLoopTest, StaleEpochCreatesNoFrame) {
    BuildScene();
    CorrectSceneLoop();
    const size_t num_keyframes = mapper_.num_keyframes();
    // A FrameTracked of the replica before the correction: weak, but no
    // Frame; its points are still counted.
    const std::vector<map::MapPointId> points = mapper_.covisibility().Points(Query().id);
    ASSERT_FALSE(points.empty());
    const uint32_t visible = mapper_.NumVisible(points[0]);
    const uint32_t found = mapper_.NumFound(points[0]);
    for (const uint32_t epoch : {0u, 1u}) {
        std::vector<Eigen::Index> scene_of_feature;
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = map::MapId{1};
        event->timestamp = 10.0 + epoch;
        event->pose = replica().keyframe(Query().id)->T_cw;
        event->measurement = test::Share(View(Query().T_true, 10.0 + epoch, scene_of_feature));
        event->visible_points = {points[0]};
        event->observed_points = {points[0]};
        event->observed_features = {0};
        event->reference_frame_id = Query().id;
        event->num_inliers = 20;
        event->map_epoch = epoch;
        tracker_.Notify(std::move(event));
        Drain();
        const std::vector<Published> created = TakeCaptured();
        if (epoch == 0) {
            EXPECT_TRUE(created.empty());
            EXPECT_EQ(mapper_.num_keyframes(), num_keyframes);
            EXPECT_EQ(mapper_.NumVisible(points[0]), visible + 1);
            EXPECT_EQ(mapper_.NumFound(points[0]), found + 1);
        } else {
            EXPECT_EQ(created.size(), 2u);  // the current epoch creates it
            EXPECT_EQ(mapper_.num_keyframes(), num_keyframes + 1);
        }
        EXPECT_EQ(mapper_.stale_epoch_skipped(), 1u);  // the epoch-0 one
    }
}

TEST_F(MapperLoopCullingTest, RejectsLoopWithCulledFrames) {
    // Review Focus 1. Frames E1..E3 repeat E0's view: with Frame culling some
    // of them go. A loop naming a culled Frame, as the query or as the match,
    // is ignored; a loop of alive Frames is corrected, and its transaction
    // names no culled Frame.
    BuildScene();
    std::vector<map::FrameId> culled;
    for (const SceneFrame& frame : frames_) {
        if (replica().keyframe(frame.id) == nullptr) culled.push_back(frame.id);
    }
    RecordProperty("culled", std::to_string(culled.size()));
    ASSERT_FALSE(culled.empty());
    std::optional<map::FrameId> alive_match;
    for (size_t k = 0; k < kLoopSideFrames; ++k) {
        if (replica().keyframe(frames_[k].id) != nullptr) alive_match = frames_[k].id;
    }
    ASSERT_TRUE(alive_match.has_value());
    for (const map::FrameId frame : culled) {
        PublishLoop(Query().id, frame, TrueS(), {});
        PublishLoop(frame, *alive_match, TrueS(), {});
    }
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(replica().map_epoch(), 0u);

    // Alive: the query and, as the match, the first alive start Frame.
    const SceneFrame* match = nullptr;
    for (size_t k = 0; k < kLoopSideFrames && match == nullptr; ++k) {
        if (replica().keyframe(frames_[k].id) != nullptr) match = &frames_[k];
    }
    ASSERT_NE(match, nullptr);
    ASSERT_NE(replica().keyframe(Query().id), nullptr);
    PublishLoop(Query().id, match->id, ToSim3(Query().T_true * match->T_true.Inverse()), MatchedPoints());
    Drain();
    const std::vector<Published> corrected = TakeCaptured();
    ASSERT_EQ(corrected.size(), 1u);
    EXPECT_EQ(corrected[0].map_epoch, 1u);
    for (const map::FrameId frame : culled) {
        for (const auto& [posed, pose] : corrected[0].updated_poses) EXPECT_NE(posed, frame);
        for (const auto& added : corrected[0].added_observations) EXPECT_NE(added.frame, frame);
        for (const auto& removed : corrected[0].removed_observations) EXPECT_NE(removed.frame, frame);
    }
    ExpectReplicaConsistent();
}

TEST_F(MapperLoopTest, IgnoresDuplicateLoop) {
    // Review Focus 2: the LoopCloser keeps finding the place until the
    // correction reaches its replica. The same loop again, and the query's
    // neighbour with the match's neighbour, are ignored.
    BuildScene();
    const auto pairs = MatchedPoints();
    CorrectSceneLoop();
    PublishLoop(Query().id, Match().id, TrueS(), pairs);
    const SceneFrame& neighbour = frames_[frames_.size() - 2];
    const SceneFrame& match_neighbour = frames_[kLoopMatch + 1];
    PublishLoop(neighbour.id, match_neighbour.id, ToSim3(neighbour.T_true * match_neighbour.T_true.Inverse()), {});
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(replica().map_epoch(), 1u);
}

TEST_F(MapperLoopTest, MonoWindowFoldsScale) {
    // Mono: a window moved by a similarity of scale 1.25. The query's pose is
    // [R t/s], and in every window camera each point the window observes
    // keeps its direction with its depth divided by the scale (X' = S_new^-1
    // S_old X, the scale folded into the translation). Other Frames and
    // points stay.
    SetUpMonoMap();
    ASSERT_LT(next_ + 16, sequence_.frames.size());
    const map::FrameId other = CreateMonoFrame(next_ + 8).frame().frame_id;
    const map::FrameId query = CreateMonoFrame(next_ + 16).frame().frame_id;
    const std::vector<map::FrameId> window = {query, other};
    const map::CovisibilityIndex& covisibility = mapper_.covisibility();
    ASSERT_FALSE(covisibility.Points(query).empty());

    const geometry::TVector3DF axis = geometry::TVector3DF(0.3, 1.0, 0.2).normalized();
    const geometry::Sim3 S{1.25, Eigen::AngleAxis<double>(0.1, axis).toRotationMatrix(),
                           geometry::TVector3DF(0.2, -0.1, 0.3)};
    std::map<uint32_t, geometry::TPoint3DF> before;
    for (uint32_t p = 0; p < replica().num_points(); ++p) {
        if (replica().IsAlive(map::MapPointId{p})) before[p] = MapperTestAccess::Position(mapper_, {p});
    }
    std::map<uint32_t, geometry::RigidTransformation> poses;
    for (const map::FrameId frame : window) poses[frame.value] = MapperTestAccess::Pose(mapper_, frame);
    const geometry::RigidTransformation first_pose = MapperTestAccess::Pose(mapper_, first_.frame_id);
    const geometry::RigidTransformation second_pose = MapperTestAccess::Pose(mapper_, second_.frame_id);
    MapperTestAccess::CorrectWindow(mapper_, window, S);
    EXPECT_EQ(MapperTestAccess::Pose(mapper_, first_.frame_id).translation(), first_pose.translation());
    EXPECT_EQ(MapperTestAccess::Pose(mapper_, second_.frame_id).translation(), second_pose.translation());
    EXPECT_EQ(MapperTestAccess::Pose(mapper_, second_.frame_id).rotation(), second_pose.rotation());

    const geometry::RigidTransformation query_pose = MapperTestAccess::Pose(mapper_, query);
    EXPECT_LT(RotationDeg(query_pose.rotation(), S.R), 1e-9);
    EXPECT_LT((query_pose.translation() - S.t / S.scale).norm(), 1e-12);
    std::set<uint32_t> moved;
    for (const map::FrameId frame : window) {
        const geometry::RigidTransformation pose = MapperTestAccess::Pose(mapper_, frame);
        for (const map::MapPointId point : covisibility.Points(frame)) {
            const geometry::TPoint3DF old_camera = poses[frame.value].Transform(before[point.value]);
            const geometry::TPoint3DF new_camera = pose.Transform(MapperTestAccess::Position(mapper_, point));
            EXPECT_LT((new_camera - old_camera / S.scale).norm(), 1e-9 * old_camera.norm())
                << "Frame " << frame.value << " point " << point.value;
            moved.insert(point.value);
        }
    }
    size_t num_still = 0;
    for (const auto& [p, X] : before) {
        if (moved.contains(p)) continue;
        EXPECT_EQ(MapperTestAccess::Position(mapper_, {p}), X) << "point " << p;
        ++num_still;
    }
    EXPECT_FALSE(moved.empty());
    RecordProperty("moved_still", std::to_string(moved.size()) + " / " + std::to_string(num_still));
}

TEST_F(MapperLoopTest, IgnoresLoopClosedByLoopEdgeOnly) {
    // Review Focus 2, the loop edge alone: a similarity 20 m off fuses
    // nothing, so after its correction the two sides share no point, yet
    // the query and the match have a loop edge. The same loop again, and the
    // query's neighbour with the match's neighbour, are ignored.
    BuildScene();
    geometry::Sim3 far = TrueS();
    far.t += geometry::TVector3DF(20.0, 0.0, 0.0);
    PublishLoop(Query().id, Match().id, far, {});
    Drain();
    ASSERT_EQ(TakeCaptured().size(), 1u);
    ASSERT_EQ(replica().loop_edges(Query().id), std::set<map::FrameId>{Match().id});
    for (const SceneFrame& frame : frames_) {
        if (frame.side != LoopSide::Return) continue;
        for (const auto& [other, shared] : replica().covisibility().Covisible(frame.id, frames_.size())) {
            EXPECT_EQ(std::find_if(frames_.begin(), frames_.end(),
                                   [&](const SceneFrame& f) { return f.id == other && f.side == LoopSide::Start; }),
                      frames_.end())
                << "Frame " << frame.id.value << " covisible with start Frame " << other.value;
        }
    }

    PublishLoop(Query().id, Match().id, TrueS(), MatchedPoints());
    const SceneFrame& neighbour = frames_[frames_.size() - 2];
    const SceneFrame& match_neighbour = frames_[kLoopMatch + 1];
    PublishLoop(neighbour.id, match_neighbour.id, ToSim3(neighbour.T_true * match_neighbour.T_true.Inverse()), {});
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(replica().map_epoch(), 1u);
}

TEST_F(MapperLoopTest, WeakQueryMatchLinkIsALoopConnection) {
    // ORB-SLAM3 takes the loop connections among GetConnectedKeyFrames, at any
    // weight: fused through only 10 pairs, the query and the match share 10
    // points, fewer than a connection's 15, and are still a loop connection;
    // every other loop connection is strong.
    BuildScene();
    std::vector<std::pair<map::MapPointId, map::MapPointId>> pairs;
    for (const auto& pair : MatchedPoints()) {
        if (pairs.size() < 10 && mapper_.covisibility().Contains(Match().id, pair.second)) pairs.push_back(pair);
    }
    ASSERT_EQ(pairs.size(), 10u);
    const auto connections = MapperTestAccess::LoopConnections(mapper_, Query().id, Match().id, TrueS(), pairs);
    const auto shared = [&](map::FrameId a, map::FrameId b) {
        for (const auto& [other, count] : mapper_.covisibility().Covisible(a, frames_.size())) {
            if (other == b) return count;
        }
        return 0u;
    };
    const uint32_t query_match = shared(Query().id, Match().id);
    EXPECT_GT(query_match, 0u);
    EXPECT_LT(query_match, 15u);
    const auto query_match_pair = std::make_pair(Query().id, Match().id);
    EXPECT_NE(std::find(connections.begin(), connections.end(), query_match_pair), connections.end());
    for (const auto& [frame, other] : connections) {
        if (std::make_pair(frame, other) != query_match_pair) EXPECT_GE(shared(frame, other), 100u);
    }
}

TEST_F(MapperLoopTest, IgnoresLoopOfOldMap) {
    // Review Focus 3: map 1 is gone and the scene is map 2. Loops naming its
    // live Frames but map 1 as the query's or the match's map (or both) are
    // ignored; the same loop of map 2 is corrected.
    InitializeStereo(0.0);
    LoseMap(1, 0.5);
    scene_map_ = map::MapId{2};
    BuildScene();
    ASSERT_EQ(mapper_.current_map_id(), map::MapId{2});
    ASSERT_EQ(replica().num_keyframes(), 3 * kLoopSideFrames);
    const auto pairs = MatchedPoints();
    PublishLoop(Query().id, Match().id, TrueS(), pairs, 1, 1);
    PublishLoop(Query().id, Match().id, TrueS(), pairs, 2, 1);
    PublishLoop(Query().id, Match().id, TrueS(), pairs, 1, 2);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(replica().map_epoch(), 0u);

    PublishLoop(Query().id, Match().id, TrueS(), pairs, 2, 2);
    Drain();
    const std::vector<Published> corrected = TakeCaptured();
    ASSERT_EQ(corrected.size(), 1u);
    EXPECT_EQ(corrected[0].map_id, map::MapId{2});
    EXPECT_EQ(corrected[0].map_epoch, 1u);
}

// Global BA merge (global-BA spec section 3; ORB-SLAM3's post-GBA block in
// LoopClosing::RunGlobalBundleAdjustment). The scene is the loop scene; a
// result is made by hand from the replica at a snapshot (the GlobalBA thread's
// replica at epoch N): each Frame's pose moved by its own small motion, each
// alive point shifted by its own offset.
class MapperGlobalBaTest : public MapperLoopTest {
protected:
    explicit MapperGlobalBaTest(MapperOptions options = SceneOptions(/*cull_frames=*/false))
        : MapperLoopTest(options) {}

    // The scene views made before the snapshot: the start and middle sides;
    // the return side's 4 views follow it. Without Frame culling, return
    // Frames 8 <- 10 <- 11 are a chain of 3 new Frames under middle Frame 7.
    static constexpr size_t kSnapshotViews = 2 * kLoopSideFrames;

    struct Result {
        map::MapId map_id;
        uint32_t map_epoch = 0;
        std::vector<std::pair<map::FrameId, geometry::RigidTransformation>> frame_poses;
        std::vector<std::pair<map::MapPointId, geometry::TPoint3DF>> point_positions;
    };

    // Frame `frame`'s motion: a rotation of 0.2 (id + 1) mrad and a
    // translation of 1 (id + 1) cm, distinct per Frame.
    static geometry::RigidTransformation Motion(map::FrameId frame) {
        const double k = 1.0 + static_cast<double>(frame.value);
        const geometry::TVector3DF axis = geometry::TVector3DF(0.3, 1.0, -0.2).normalized();
        return {Eigen::AngleAxis<double>(2e-4 * k, axis).toRotationMatrix(),
                0.01 * k * geometry::TVector3DF(1.0, -0.5, 0.3)};
    }
    // Point `point`'s offset: 1..7 mm.
    static geometry::TPoint3DF Offset(map::MapPointId point) {
        return 1e-3 * static_cast<double>(1 + point.value % 7) * geometry::TPoint3DF(1.0, 0.5, -1.0);
    }

    // The result of the replica's map as it stands: every Frame at
    // Motion * T_cw, every alive point at X + Offset.
    [[nodiscard]] Result Snapshot() const {
        Result result{replica().map_id(), replica().map_epoch(), {}, {}};
        for (const map::FrameId frame : replica().keyframe_ids()) {
            result.frame_poses.emplace_back(frame, Motion(frame) * replica().keyframe(frame)->T_cw);
        }
        for (uint32_t p = 0; p < replica().num_points(); ++p) {
            const map::MapPointId point{p};
            if (replica().IsAlive(point)) {
                result.point_positions.emplace_back(point, replica().Position(point) + Offset(point));
            }
        }
        return result;
    }

    void PublishGlobalBa(const Result& result) {
        auto event = std::make_shared<events::GlobalBAFinished>();
        event->map_id = result.map_id;
        event->map_epoch = result.map_epoch;
        event->frame_poses = result.frame_poses;
        event->point_positions = result.point_positions;
        event->priority = true;  // as the GlobalBA thread publishes it
        tracker_.Notify(std::move(event));
    }

    // Publishes `result` and returns what the Mapper published.
    std::vector<Published> Merge(const Result& result) {
        PublishGlobalBa(result);
        Drain();
        return TakeCaptured();
    }

    // The Mapper's current Frames and alive points.
    [[nodiscard]] std::map<uint32_t, geometry::RigidTransformation> MapperPoses() const {
        std::map<uint32_t, geometry::RigidTransformation> poses;
        for (const map::FrameId frame : replica().keyframe_ids()) {
            poses.emplace(frame.value, MapperTestAccess::Pose(mapper_, frame));
        }
        return poses;
    }
    [[nodiscard]] std::map<uint32_t, geometry::TPoint3DF> MapperPositions() const {
        std::map<uint32_t, geometry::TPoint3DF> positions;
        for (uint32_t p = 0; p < replica().num_points(); ++p) {
            if (mapper_.Reference(map::MapPointId{p})) {
                positions.emplace(p, MapperTestAccess::Position(mapper_, map::MapPointId{p}));
            }
        }
        return positions;
    }

    // The largest entry of the difference of two poses' [R | t].
    static double PoseDifference(const geometry::RigidTransformation& a, const geometry::RigidTransformation& b) {
        return (a.ToMatrix34() - b.ToMatrix34()).cwiseAbs().maxCoeff();
    }

    // Sync observer of the Mapper: on the first transaction 1 of a new
    // Frame, publishes `pending_result_` into the Mapper's inbox, as the
    // GlobalBA thread would while the Mapper is about to run local BA.
    static void PublishResultOnFrameCreated(void* ctx, const events::Event& event) {
        auto* self = static_cast<MapperGlobalBaTest*>(ctx);
        if (!self->pending_result_ || event.type != events::EventType::MapTransaction) return;
        const auto& transaction = static_cast<const TTransaction&>(event);
        if (transaction.new_map || transaction.added_frames.empty()) return;
        auto finished = std::make_shared<events::GlobalBAFinished>();
        finished->map_id = self->pending_result_->map_id;
        finished->map_epoch = self->pending_result_->map_epoch;
        finished->frame_poses = self->pending_result_->frame_poses;
        finished->point_positions = self->pending_result_->point_positions;
        finished->priority = true;
        self->pending_result_.reset();
        self->global_ba_.Notify(std::move(finished));
    }

    events::Observable global_ba_;  // stands in for the GlobalBA thread
    std::optional<Result> pending_result_;
};

class MapperGlobalBaCullingTest : public MapperGlobalBaTest {
protected:
    MapperGlobalBaCullingTest()
        : MapperGlobalBaTest(SceneOptions(/*cull_frames=*/true)) {}
};

TEST_F(MapperGlobalBaTest, AppliesOptimizedPosesAndPoints) {
    BuildScene();
    const Result result = Snapshot();
    const auto start = std::chrono::steady_clock::now();
    const std::vector<Published> merged = Merge(result);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    RecordProperty("merge_ms", std::to_string(ms));
    RecordProperty("frames_points", std::to_string(result.frame_poses.size()) + " / " +
                                        std::to_string(result.point_positions.size()));
    ASSERT_EQ(merged.size(), 1u);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 1u);
    EXPECT_EQ(mapper_.num_global_ba_rejected(), 0u);
    // Every Frame and every point, in the Mapper and in the replica.
    EXPECT_EQ(merged[0].updated_poses.size(), result.frame_poses.size());
    EXPECT_EQ(merged[0].updated_ids.size(), result.point_positions.size());
    for (const auto& [frame, T_cw] : result.frame_poses) {
        EXPECT_LT(PoseDifference(MapperTestAccess::Pose(mapper_, frame), T_cw), 1e-12) << "Frame " << frame.value;
        EXPECT_LT(PoseDifference(replica().keyframe(frame)->T_cw, T_cw), 1e-12) << "Frame " << frame.value;
    }
    for (const auto& [point, X] : result.point_positions) {
        EXPECT_LT((MapperTestAccess::Position(mapper_, point) - X).norm(), 1e-12) << "point " << point.value;
        EXPECT_LT((replica().Position(point) - X).norm(), 1e-12) << "point " << point.value;
    }
    ExpectReplicaConsistent();
}

TEST_F(MapperGlobalBaTest, PropagatesToNewFramesThroughTree) {
    // Frames created after the snapshot are not in the result: each keeps its
    // pose relative to its parent, which was corrected first.
    BuildScene(kSnapshotViews);
    const Result result = Snapshot();
    BuildScene();
    std::vector<map::FrameId> added;
    for (size_t k = kSnapshotViews; k < frames_.size(); ++k) added.push_back(frames_[k].id);
    ASSERT_EQ(added.size(), kLoopSideFrames);
    const auto is_added = [&](map::FrameId frame) {
        return std::find(added.begin(), added.end(), frame) != added.end();
    };
    // The longest chain of new Frames under a Frame of the result.
    size_t chain = 0;
    for (const map::FrameId frame : added) {
        size_t depth = 0;
        for (std::optional<map::FrameId> f = frame; f && is_added(*f); f = mapper_.Parent(*f)) ++depth;
        chain = std::max(chain, depth);
    }
    EXPECT_EQ(chain, 3u);
    const std::map<uint32_t, geometry::RigidTransformation> before = MapperPoses();

    ASSERT_EQ(Merge(result).size(), 1u);
    for (const map::FrameId frame : added) {
        const map::FrameId parent = *mapper_.Parent(frame);
        const geometry::RigidTransformation relative_before =
            before.at(frame.value) * before.at(parent.value).Inverse();
        const geometry::RigidTransformation relative_after =
            MapperTestAccess::Pose(mapper_, frame) * MapperTestAccess::Pose(mapper_, parent).Inverse();
        EXPECT_LT(PoseDifference(relative_after, relative_before), 1e-9) << "Frame " << frame.value;
        EXPECT_GT(PoseDifference(MapperTestAccess::Pose(mapper_, frame), before.at(frame.value)), 1e-3)
            << "Frame " << frame.value;
        EXPECT_LT(PoseDifference(replica().keyframe(frame)->T_cw, MapperTestAccess::Pose(mapper_, frame)), 1e-12);
    }
    for (const auto& [frame, T_cw] : result.frame_poses) {
        EXPECT_LT(PoseDifference(MapperTestAccess::Pose(mapper_, frame), T_cw), 1e-12) << "Frame " << frame.value;
    }
    ExpectReplicaConsistent();
}

TEST_F(MapperGlobalBaCullingTest, FollowsReparentedChain) {
    // Review Focus 1: after the snapshot, return Frame 10 is culled and its
    // child re-parented. The child, not in the result, follows its new parent.
    BuildScene(kSnapshotViews);
    const Result result = Snapshot();
    std::map<uint32_t, map::FrameId> reparented;  // child -> new parent, after the snapshot
    std::vector<map::FrameId> culled;
    on_scene_published_ = [&](const Published& published) {
        for (const auto& [child, parent] : published.parent_changes) reparented[child.value] = parent;
        culled.insert(culled.end(), published.removed_frames.begin(), published.removed_frames.end());
    };
    BuildScene();
    const auto in_result = [&](uint32_t frame) {
        return std::any_of(result.frame_poses.begin(), result.frame_poses.end(),
                           [&](const auto& posed) { return posed.first.value == frame; });
    };
    std::vector<map::FrameId> followers;  // re-parented, alive, not in the result
    for (const auto& [child, parent] : reparented) {
        if (replica().keyframe(map::FrameId{child}) != nullptr && !in_result(child)) {
            followers.push_back(map::FrameId{child});
        }
    }
    RecordProperty("culled_followers", std::to_string(culled.size()) + " / " + std::to_string(followers.size()));
    ASSERT_FALSE(followers.empty());
    const std::map<uint32_t, geometry::RigidTransformation> before = MapperPoses();

    ASSERT_EQ(Merge(result).size(), 1u);
    for (const map::FrameId frame : followers) {
        const map::FrameId parent = *mapper_.Parent(frame);
        EXPECT_EQ(parent, reparented.at(frame.value));
        const geometry::RigidTransformation relative_before =
            before.at(frame.value) * before.at(parent.value).Inverse();
        const geometry::RigidTransformation relative_after =
            MapperTestAccess::Pose(mapper_, frame) * MapperTestAccess::Pose(mapper_, parent).Inverse();
        EXPECT_LT(PoseDifference(relative_after, relative_before), 1e-9) << "Frame " << frame.value;
        EXPECT_GT(PoseDifference(MapperTestAccess::Pose(mapper_, frame), before.at(frame.value)), 1e-3)
            << "Frame " << frame.value;
    }
    ExpectReplicaConsistent();
}

TEST_F(MapperGlobalBaTest, NewPointFollowsNewReferenceFrame) {
    // Review Focus 2: a point created after the snapshot moves with its
    // reference Frame, itself created after the snapshot and corrected
    // through the tree: in that Frame's camera it stays where it was.
    BuildScene(kSnapshotViews);
    const Result result = Snapshot();
    const uint32_t first_new = replica().num_points();
    BuildScene();
    const std::map<uint32_t, geometry::RigidTransformation> poses_before = MapperPoses();
    const std::map<uint32_t, geometry::TPoint3DF> positions_before = MapperPositions();

    ASSERT_EQ(Merge(result).size(), 1u);
    size_t num_new = 0;
    for (const auto& [p, X_before] : positions_before) {
        if (p < first_new) continue;
        const map::MapPointId point{p};
        const map::FrameId reference = mapper_.Reference(point)->keyframe;
        ASSERT_GE(reference.value, frames_[kSnapshotViews].id.value) << "point " << p;
        const geometry::TPoint3DF X = MapperTestAccess::Position(mapper_, point);
        const geometry::TPoint3DF camera_before = poses_before.at(reference.value).Transform(X_before);
        const geometry::TPoint3DF camera_after = MapperTestAccess::Pose(mapper_, reference).Transform(X);
        EXPECT_LT((camera_after - camera_before).norm(), 1e-9) << "point " << p;
        EXPECT_GT((X - X_before).norm(), 1e-3) << "point " << p;
        EXPECT_LT((replica().Position(point) - X).norm(), 1e-12) << "point " << p;
        ++num_new;
    }
    RecordProperty("new_points", std::to_string(num_new));
    EXPECT_GE(num_new, 100u);
    ExpectReplicaConsistent();
}

TEST_F(MapperGlobalBaCullingTest, SkipsCulledAndFused) {
    // After the snapshot (the start side and 3 middle views), middle Frame 5
    // is culled and points are culled or merged away: the result names them,
    // the merge transaction does not.
    BuildScene(kSnapshotViews - 1);
    const Result result = Snapshot();
    BuildScene();
    std::set<uint32_t> culled, removed;
    for (const auto& [frame, T_cw] : result.frame_poses) {
        if (replica().keyframe(frame) == nullptr) culled.insert(frame.value);
    }
    for (const auto& [point, X] : result.point_positions) {
        if (!replica().IsAlive(point)) removed.insert(point.value);
    }
    RecordProperty("culled_removed", std::to_string(culled.size()) + " / " + std::to_string(removed.size()));
    ASSERT_FALSE(culled.empty());
    ASSERT_FALSE(removed.empty());

    const std::vector<Published> merged = Merge(result);
    ASSERT_EQ(merged.size(), 1u);
    EXPECT_FALSE(merged[0].updated_poses.empty());
    EXPECT_FALSE(merged[0].updated_ids.empty());
    for (const auto& [frame, T_cw] : merged[0].updated_poses) {
        EXPECT_FALSE(culled.contains(frame.value)) << "Frame " << frame.value;
    }
    for (const map::MapPointId point : merged[0].updated_ids) {
        EXPECT_FALSE(removed.contains(point.value)) << "point " << point.value;
    }
    for (const uint32_t p : removed) EXPECT_FALSE(replica().IsAlive(map::MapPointId{p})) << "point " << p;
    ExpectReplicaConsistent();
}

TEST_F(MapperGlobalBaTest, PublishesOneTransactionAtNextEpoch) {
    // After a loop correction (epoch 1), a result of epoch 1 is merged at
    // epoch 2: exactly one transaction, without loop edges, so the GlobalBA
    // thread starts no new run on it.
    BuildScene();
    CorrectSceneLoop();
    ASSERT_EQ(replica().map_epoch(), 1u);
    const Result result = Snapshot();
    ASSERT_EQ(result.map_epoch, 1u);
    const std::vector<Published> merged = Merge(result);
    ASSERT_EQ(merged.size(), 1u);
    const Published& transaction = merged[0];
    EXPECT_EQ(transaction.type, events::EventType::MapTransaction);
    EXPECT_EQ(transaction.map_id, map::MapId{1});
    EXPECT_EQ(transaction.map_epoch, 2u);
    EXPECT_TRUE(transaction.added_loop_edges.empty());
    EXPECT_FALSE(transaction.new_map);
    EXPECT_TRUE(transaction.frames.empty());
    EXPECT_EQ(transaction.count, 0u);
    EXPECT_TRUE(transaction.removed_points.empty());
    EXPECT_TRUE(transaction.removed_frames.empty());
    EXPECT_TRUE(transaction.parent_changes.empty());
    EXPECT_TRUE(transaction.replaced_points.empty());
    EXPECT_EQ(replica().map_epoch(), 2u);
    // The replica applied it consistently (TakeCaptured checks CheckConsistency,
    // the graph included), with the Mapper's poses.
    ExpectReplicaConsistent();
    for (const map::FrameId frame : replica().keyframe_ids()) {
        EXPECT_EQ(replica().keyframe(frame)->T_cw.ToMatrix34(), MapperTestAccess::Pose(mapper_, frame).ToMatrix34())
            << "Frame " << frame.value;
    }
    // The Mapper is at epoch 2: a FrameTracked of epoch 1 creates no Frame.
    std::vector<Eigen::Index> scene_of_feature;
    auto event = std::make_shared<TFrameTracked>();
    event->map_id = map::MapId{1};
    event->timestamp = 10.0;
    event->pose = replica().keyframe(Query().id)->T_cw;
    event->measurement = test::Share(View(Query().T_true, 10.0, scene_of_feature));
    event->reference_frame_id = Query().id;
    event->num_inliers = 20;
    event->map_epoch = 1;
    tracker_.Notify(std::move(event));
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.stale_epoch_skipped(), 1u);
}

TEST_F(MapperGlobalBaTest, ResultInterruptsLocalBa) {
    // GlobalBAFinished is urgent, like LoopDetected: arriving once a new
    // Frame's transaction 1 is out, it stops that Frame's local BA, then is
    // merged, the new Frame following its parent.
    BuildScene(kSnapshotViews);
    pending_result_ = Snapshot();
    subscriptions_.push_back(global_ba_.ConnectAsync(mapper_.inbox()));
    subscriptions_.push_back(mapper_.AddSyncObserver(&MapperGlobalBaTest::PublishResultOnFrameCreated, this));
    std::vector<Eigen::Index> scene_of_feature;
    auto event = std::make_shared<TFrameTracked>();
    event->map_id = map::MapId{1};
    event->timestamp = 10.0;
    // Start Frame E1's view again, tracking the start side's points: local
    // BA has work, with E0 fixed.
    event->pose = replica().keyframe(Match().id)->T_cw;
    event->measurement = test::Share(View(Match().T_true, 10.0, scene_of_feature));
    const std::vector<std::optional<map::MapPointId>> mapped = SidePoints(LoopSide::Start);
    for (size_t f = 0; f < scene_of_feature.size(); ++f) {
        const Eigen::Index scene = scene_of_feature[f];
        if (scene >= 0 && mapped[static_cast<size_t>(scene)]) {
            event->observed_features.push_back(static_cast<uint32_t>(f));
            event->observed_points.push_back(*mapped[static_cast<size_t>(scene)]);
        }
    }
    event->reference_frame_id = Match().id;
    event->num_matches = static_cast<uint32_t>(event->observed_points.size());
    event->num_inliers = 20;
    tracker_.Notify(std::move(event));
    Drain();
    ASSERT_FALSE(pending_result_.has_value());
    const LocalBaSummary& ba = mapper_.last_local_ba();
    ASSERT_TRUE(ba.ran);
    EXPECT_TRUE(ba.interrupted);
    EXPECT_EQ(mapper_.inbox().pending_priority(), 0u);
    const std::vector<Published> published = TakeCaptured();
    ASSERT_EQ(published.size(), 3u);  // the new Frame's two transactions, the merge
    const map::FrameId added = published[0].frame().frame_id;
    EXPECT_EQ(published[2].map_epoch, 1u);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 1u);
    EXPECT_NE(std::find_if(published[2].updated_poses.begin(), published[2].updated_poses.end(),
                           [&](const auto& posed) { return posed.first == added; }),
              published[2].updated_poses.end());
    ExpectReplicaConsistent();
}

TEST_F(MapperGlobalBaTest, RejectsStaleEpoch) {
    // A result of epoch 0 arrives after a loop correction (epoch 1): ignored.
    // The same map's result of epoch 1 is merged.
    BuildScene();
    const Result stale = Snapshot();
    CorrectSceneLoop();
    const std::map<uint32_t, geometry::RigidTransformation> poses = MapperPoses();
    const std::map<uint32_t, geometry::TPoint3DF> positions = MapperPositions();
    EXPECT_TRUE(Merge(stale).empty());
    EXPECT_EQ(mapper_.num_global_ba_rejected(), 1u);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 0u);
    EXPECT_EQ(MapperPoses().size(), poses.size());
    for (const auto& [frame, T_cw] : MapperPoses()) {
        EXPECT_EQ(T_cw.ToMatrix34(), poses.at(frame).ToMatrix34()) << "Frame " << frame;
    }
    EXPECT_EQ(MapperPositions(), positions);
    EXPECT_EQ(replica().map_epoch(), 1u);

    EXPECT_EQ(Merge(Snapshot()).size(), 1u);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 1u);
}

TEST_F(MapperGlobalBaTest, RejectsOldMap) {
    // Review Focus 3: a result of map 1 arrives once map 1 is gone, first
    // with no map, then with map 2 at the same epoch 0 (its point ids
    // overlap map 1's): ignored. Map 2's result is merged.
    InitializeStereo(0.0);
    const Result old_map = Snapshot();
    ASSERT_EQ(old_map.map_id, map::MapId{1});
    LoseMap(1, 0.5);
    ASSERT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_TRUE(Merge(old_map).empty());
    EXPECT_EQ(mapper_.num_global_ba_rejected(), 1u);

    scene_map_ = map::MapId{2};
    BuildScene();
    ASSERT_EQ(mapper_.current_map_id(), map::MapId{2});
    const std::map<uint32_t, geometry::RigidTransformation> poses = MapperPoses();
    const std::map<uint32_t, geometry::TPoint3DF> positions = MapperPositions();
    EXPECT_TRUE(Merge(old_map).empty());
    EXPECT_EQ(mapper_.num_global_ba_rejected(), 2u);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 0u);
    for (const auto& [frame, T_cw] : MapperPoses()) {
        EXPECT_EQ(T_cw.ToMatrix34(), poses.at(frame).ToMatrix34()) << "Frame " << frame;
    }
    EXPECT_EQ(MapperPositions(), positions);

    const std::vector<Published> merged = Merge(Snapshot());
    ASSERT_EQ(merged.size(), 1u);
    EXPECT_EQ(merged[0].map_id, map::MapId{2});
    EXPECT_EQ(mapper_.num_global_ba_merges(), 1u);
}

// Map merging (map-merging spec section 3; ORB-SLAM3 LoopClosing::MergeLocal).
// The loop scene's two regions in two maps, built through the Mapper's own
// inbox: map 1 maps region A from the start (S0..S9 at StartPose, in the true
// world: S0 initializes it at the origin) and is lost with 10 Frames, so it is
// kept inactive; map 2 initializes at MiddlePose(0) (its world is that
// camera), maps region B (M0..M3) and comes back to region A through new
// points (R0..R3 at ReturnPose), all exact in its own world. A merge
// LoopDetected from R3 to S1 carries the true similarity disturbed by
// MergeError(): the window it places is off by about 4 cm and 1 deg, which the
// weld and then the essential graph remove.
constexpr size_t kMapOneFrames = 10;

class MapperMergeTest : public MapperLoopTest {
protected:
    explicit MapperMergeTest(MapperOptions options = SceneOptions(/*cull_frames=*/false))
        : MapperLoopTest(options) {}

    void BuildMergeScene() {
        MakeRegions();
        for (size_t j = 0; j < kMapOneFrames; ++j) {
            AddSceneView(LoopSide::Start, StartPose(j), StartPose(j));
            ASSERT_FALSE(HasFatalFailure()) << "S" << j;
        }
        if (before_map_one_lost_) before_map_one_lost_();
        PublishTrackingLost(1, 0.05 * static_cast<double>(kMapOneFrames) - 0.01);
        Drain();
        const std::vector<Published> lost = TakeCaptured();
        ASSERT_EQ(lost.size(), 1u);
        ASSERT_TRUE(lost[0].removed_maps.empty());  // kept inactive
        scene_map_ = map::MapId{2};
        const geometry::RigidTransformation T_world2_world = MiddlePose(0);
        for (size_t j = 0; j < kLoopSideFrames; ++j) {
            AddSceneView(LoopSide::Middle, MiddlePose(j), MiddlePose(j) * T_world2_world.Inverse());
            ASSERT_FALSE(HasFatalFailure()) << "M" << j;
        }
        for (size_t j = 0; j < kLoopSideFrames; ++j) {
            AddSceneView(LoopSide::Return, ReturnPose(j), ReturnPose(j) * T_world2_world.Inverse());
            ASSERT_FALSE(HasFatalFailure()) << "R" << j;
        }
        ASSERT_EQ(mapper_.active_map(), map::MapId{2});
        ASSERT_EQ(atlas_.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{2}}));
        ASSERT_EQ(atlas_.map(map::MapId{1})->num_keyframes(), kMapOneFrames);
        ASSERT_EQ(atlas_.map(map::MapId{2})->num_keyframes(), 2 * kLoopSideFrames);
    }

    [[nodiscard]] const tracking::MapReplica<Extractor>& MapOne() const { return *atlas_.map(map::MapId{1}); }
    [[nodiscard]] const tracking::MapReplica<Extractor>& MapTwo() const { return *atlas_.map(map::MapId{2}); }

    // The ids of the scene's Frames of `side`.
    [[nodiscard]] std::set<map::FrameId> SideFrames(LoopSide side) const {
        std::set<map::FrameId> ids;
        for (const SceneFrame& frame : frames_) {
            if (frame.side == side) ids.insert(frame.id);
        }
        return ids;
    }

    // About 1 deg and 4 cm, applied in the query camera.
    static geometry::Sim3 MergeError() {
        return {1.0,
                Eigen::AngleAxis<double>(1.0 * std::numbers::pi / 180.0, geometry::TVector3DF(0.3, 1.0, 0.2).normalized())
                    .toRotationMatrix(),
                geometry::TVector3DF(0.03, -0.01, 0.025)};
    }
    [[nodiscard]] geometry::Sim3 MergeS() const { return geometry::Compose(MergeError(), TrueS()); }

    // The query's map-2 points paired with map 1's start-side points of the
    // same scene point (as the LoopCloser would match them).
    [[nodiscard]] std::vector<std::pair<map::MapPointId, map::MapPointId>> MergePairs() const {
        const std::vector<std::optional<map::MapPointId>> start = SidePoints(LoopSide::Start, MapOne());
        std::vector<std::pair<map::MapPointId, map::MapPointId>> pairs;
        for (const auto& [feature, point] : MapTwo().keyframe(Query().id)->observations) {
            const Eigen::Index scene = Query().scene_of_feature[feature];
            if (scene >= 0 && start[static_cast<size_t>(scene)]) {
                pairs.emplace_back(point, *start[static_cast<size_t>(scene)]);
            }
        }
        return pairs;
    }

    void PublishSceneMerge() { PublishLoop(Query().id, Match().id, MergeS(), MergePairs(), 2, 1); }

    // `query`'s points in `query_map` paired with `match_map`'s points of
    // `side` for the same scene point.
    [[nodiscard]] std::vector<std::pair<map::MapPointId, map::MapPointId>> PairsBetween(
        const SceneFrame& query, const tracking::MapReplica<Extractor>& query_map, LoopSide side,
        const tracking::MapReplica<Extractor>& match_map) const {
        const std::vector<std::optional<map::MapPointId>> match_points = SidePoints(side, match_map);
        std::vector<std::pair<map::MapPointId, map::MapPointId>> pairs;
        for (const auto& [feature, point] : query_map.keyframe(query.id)->observations) {
            const Eigen::Index scene = query.scene_of_feature[feature];
            if (scene >= 0 && match_points[static_cast<size_t>(scene)]) {
                pairs.emplace_back(point, *match_points[static_cast<size_t>(scene)]);
            }
        }
        return pairs;
    }

    // A final TrackingLost of `map` that keeps it inactive.
    void KeepMap(uint32_t map) {
        PublishTrackingLost(map, 0.05 * static_cast<double>(frames_.size()) - 0.01);
        Drain();
        const std::vector<Published> lost = TakeCaptured();
        ASSERT_EQ(lost.size(), 1u);
        ASSERT_TRUE(lost[0].removed_maps.empty());
    }

    // Three maps: map 1 (S0..S9, region A, the true world) and map 2 (ten
    // views of region B from MiddlePose(0..9), its world that first camera)
    // are kept inactive; map 3 is the merge scene's map 2 (M0..M3 in region
    // B, R0..R3 back in region A), active. Frame ids: map 1 0..9, map 2
    // 10..19, map 3 20..27.
    void BuildThreeMapScene() {
        MakeRegions();
        const geometry::RigidTransformation T_world_b = MiddlePose(0);
        for (size_t j = 0; j < kMapOneFrames; ++j) {
            AddSceneView(LoopSide::Start, StartPose(j), StartPose(j));
            ASSERT_FALSE(HasFatalFailure());
        }
        KeepMap(1);
        ASSERT_FALSE(HasFatalFailure());
        scene_map_ = map::MapId{2};
        for (size_t j = 0; j < kMapOneFrames; ++j) {
            AddSceneView(LoopSide::Middle, MiddlePose(j), MiddlePose(j) * T_world_b.Inverse());
            ASSERT_FALSE(HasFatalFailure());
        }
        KeepMap(2);
        ASSERT_FALSE(HasFatalFailure());
        scene_map_ = map::MapId{3};
        for (size_t j = 0; j < kLoopSideFrames; ++j) {
            AddSceneView(LoopSide::Middle, MiddlePose(j), MiddlePose(j) * T_world_b.Inverse());
            ASSERT_FALSE(HasFatalFailure());
        }
        for (size_t j = 0; j < kLoopSideFrames; ++j) {
            AddSceneView(LoopSide::Return, ReturnPose(j), ReturnPose(j) * T_world_b.Inverse());
            ASSERT_FALSE(HasFatalFailure());
        }
        ASSERT_EQ(mapper_.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{2}, map::MapId{3}}));
        ASSERT_EQ(mapper_.active_map(), map::MapId{3});
    }

    // Publishes the scene's merge and returns its one transaction.
    Published MergeScene() {
        PublishSceneMerge();
        Drain();
        std::vector<Published> published = TakeCaptured();
        EXPECT_EQ(published.size(), 1u);
        return published.empty() ? Published{} : published[0];
    }

    // The mean centre error of the Frames of `side` placed only by the merge
    // similarity (what CorrectWindow gives every absorbed Frame), from map 2
    // before the merge.
    [[nodiscard]] double SimilarityOnlyCentreError(LoopSide side) const {
        const geometry::RigidTransformation T_world2_query = MapTwo().keyframe(Query().id)->T_cw.Inverse();
        const geometry::Sim3 S_query_world = geometry::Compose(MergeS(), ToSim3(MapOne().keyframe(Match().id)->T_cw));
        double sum = 0.0;
        size_t count = 0;
        for (const SceneFrame& frame : frames_) {
            if (frame.side != side) continue;
            const geometry::Sim3 S = geometry::Compose(ToSim3(MapTwo().keyframe(frame.id)->T_cw * T_world2_query),
                                                       S_query_world);
            const geometry::RigidTransformation T{S.R, S.t / S.scale};
            sum += (T.Inverse().translation() - frame.T_true.Inverse().translation()).norm();
            ++count;
        }
        return sum / static_cast<double>(count);
    }

    // A weak FrameTracked of map `map` at `epoch`: a new view at ReturnPose(3)
    // tracking `pairs`.
    void PublishReturnFrameTracked(uint32_t map, uint32_t epoch,
                                   const std::vector<std::pair<uint32_t, map::MapPointId>>& pairs) {
        std::vector<Eigen::Index> scene_of_feature;
        auto event = std::make_shared<TFrameTracked>();
        event->map_id = map::MapId{map};
        event->map_epoch = epoch;
        event->timestamp = 10.0 + static_cast<double>(++extra_views_);
        event->pose = ReturnPose(3);  // the merged map's world is the true one
        event->measurement = test::Share(View(ReturnPose(3), event->timestamp, scene_of_feature));
        for (const auto& [feature, point] : pairs) {
            event->observed_features.push_back(feature);
            event->observed_points.push_back(point);
        }
        event->reference_frame_id = Query().id;
        event->num_matches = event->num_inliers = 20;
        tracker_.Notify(std::move(event));
    }

    void PublishGlobalBa(uint32_t map, uint32_t epoch) {
        auto event = std::make_shared<events::GlobalBAFinished>();
        event->map_id = map::MapId{map};
        event->map_epoch = epoch;
        event->frame_poses.emplace_back(Match().id, replica().keyframe(Match().id)->T_cw);
        event->priority = true;
        tracker_.Notify(std::move(event));
    }

    size_t extra_views_ = 0;
    std::function<void()> before_map_one_lost_;  // runs while map 1 is still active
};

// merge_window 3: each window is its Frame and its 3 best connected Frames.
class MapperSmallMergeWindowTest : public MapperMergeTest {
protected:
    MapperSmallMergeWindowTest()
        : MapperMergeTest(SmallWindow()) {}
    static MapperOptions SmallWindow() {
        MapperOptions options = SceneOptions(/*cull_frames=*/false);
        options.atlas.merge_window = 3;
        return options;
    }
};

// Step 1: the current window is the query's side (the return Frames; the
// middle shares no point with them), the merge window all of map 1 (every
// start Frame is connected to S1), each with its Frame first.
TEST_F(MapperMergeTest, BuildsWindows) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const std::vector<map::FrameId> current = MapperTestAccess::MergeWindowOf(mapper_, map::MapId{2}, Query().id);
    const std::vector<map::FrameId> merge = MapperTestAccess::MergeWindowOf(mapper_, map::MapId{1}, Match().id);
    ASSERT_FALSE(current.empty());
    ASSERT_FALSE(merge.empty());
    EXPECT_EQ(current.front(), Query().id);
    EXPECT_EQ(merge.front(), Match().id);
    EXPECT_EQ(current.size(), kLoopSideFrames);
    EXPECT_EQ(std::set<map::FrameId>(current.begin(), current.end()), SideFrames(LoopSide::Return));
    EXPECT_EQ(merge.size(), kMapOneFrames);
    EXPECT_EQ(std::set<map::FrameId>(merge.begin(), merge.end()), SideFrames(LoopSide::Start));
}

// merge_window caps the first ring: S1 and its 3 most covisible Frames.
TEST_F(MapperSmallMergeWindowTest, BuildsWindows) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const std::vector<map::FrameId> merge = MapperTestAccess::MergeWindowOf(mapper_, map::MapId{1}, Match().id);
    std::vector<map::FrameId> expected = {Match().id};
    for (const auto& [frame, shared] : MapOne().covisibility().Covisible(Match().id, 3)) {
        ASSERT_GE(shared, 15u);
        expected.push_back(frame);
    }
    EXPECT_EQ(merge, expected);
}

// Step 3: one tree rooted at map 1's root: R3 hangs from S1 and map 2's old
// chain to M0 is inverted.
TEST_F(MapperMergeTest, ReRootsTreeIntoSingleTree) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const Published merged = MergeScene();
    ASSERT_EQ(replica().num_keyframes(), kMapOneFrames + 2 * kLoopSideFrames);
    std::vector<map::FrameId> roots;
    for (const map::FrameId frame : replica().keyframe_ids()) {
        if (!replica().parent(frame)) roots.push_back(frame);
    }
    EXPECT_EQ(roots, std::vector<map::FrameId>{frames_.front().id});
    // Acyclic: every Frame reaches the root within as many steps as Frames.
    for (const map::FrameId frame : replica().keyframe_ids()) {
        map::FrameId at = frame;
        size_t steps = 0;
        while (const std::optional<map::FrameId> parent = replica().parent(at)) {
            at = *parent;
            ASSERT_LE(++steps, replica().num_keyframes()) << "Frame " << frame.value;
        }
        EXPECT_EQ(at, frames_.front().id) << "Frame " << frame.value;
    }
    EXPECT_EQ(replica().parent(Query().id), Match().id);
    EXPECT_TRUE(replica().parent(frames_[kMapOneFrames].id).has_value());  // M0, map 2's old root
    EXPECT_FALSE(merged.parent_changes.empty());
    EXPECT_EQ(merged.parent_changes.front(), std::make_pair(Query().id, Match().id));
}

// Step 7's id rule: map 2's alive points, by old id, take map 1's next ids; a
// Frame the merge did not touch (the middle) observes them through the same
// features, and the replica's TranslatePoint agrees.
TEST_F(MapperMergeTest, RenumbersPointsContiguously) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const uint32_t first_new = static_cast<uint32_t>(MapOne().num_points());
    std::vector<uint32_t> rank(MapTwo().num_points(), 0);
    uint32_t num_alive = 0;
    std::optional<map::MapPointId> dead;
    for (uint32_t p = 0; p < MapTwo().num_points(); ++p) {
        if (MapTwo().IsAlive({p})) {
            rank[p] = num_alive++;
        } else if (!dead) {
            dead = map::MapPointId{p};
        }
    }
    ASSERT_GT(num_alive, 500u);
    std::map<uint32_t, std::vector<std::pair<uint32_t, map::MapPointId>>> middle;
    for (const map::FrameId frame : SideFrames(LoopSide::Middle)) {
        middle[frame.value] = MapTwo().keyframe(frame)->observations;
    }

    MergeScene();
    EXPECT_EQ(replica().num_points(), first_new + num_alive);
    size_t num_replica_alive = 0;
    for (uint32_t p = 0; p < replica().num_points(); ++p) num_replica_alive += replica().IsAlive({p});
    EXPECT_EQ(MapperTestAccess::NumPoints(mapper_, map::MapId{1}), num_replica_alive);
    size_t checked = 0;
    for (const auto& [frame, observations] : middle) {
        std::vector<std::pair<uint32_t, map::MapPointId>> expected;
        for (const auto& [feature, old] : observations) {
            const map::MapPointId renumbered{first_new + rank[old.value]};
            expected.emplace_back(feature, renumbered);
            EXPECT_EQ(atlas_.TranslatePoint(map::MapId{2}, old), renumbered) << "point " << old.value;
            ++checked;
        }
        std::vector<std::pair<uint32_t, map::MapPointId>> actual = replica().keyframe(map::FrameId{frame})->observations;
        std::sort(expected.begin(), expected.end());
        std::sort(actual.begin(), actual.end());
        EXPECT_EQ(actual, expected) << "Frame " << frame;
    }
    EXPECT_GT(checked, 500u);
    if (dead) EXPECT_EQ(atlas_.TranslatePoint(map::MapId{2}, *dead), std::nullopt);
    EXPECT_EQ(atlas_.TranslatePoint(map::MapId{1}, map::MapPointId{0}), std::nullopt);  // never absorbed
}

// Step 4: the query observes map 1's points where map 2 had duplicates of
// them; TranslatePoint follows the fusions.
TEST_F(MapperMergeTest, FusesSeamDuplicates) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const uint32_t first_new = static_cast<uint32_t>(MapOne().num_points());
    const auto pairs = MergePairs();
    ASSERT_GE(pairs.size(), 100u);
    std::map<uint32_t, uint32_t> feature_of;  // old query point -> feature
    for (const auto& [feature, point] : MapTwo().keyframe(Query().id)->observations) feature_of[point.value] = feature;

    const Published merged = MergeScene();
    EXPECT_GE(merged.replaced_points.size(), pairs.size());
    std::map<uint32_t, map::MapPointId> query_points;  // feature -> point after the merge
    for (const auto& [feature, point] : replica().keyframe(Query().id)->observations) query_points[feature] = point;
    size_t fused = 0;
    for (const auto& [query_point, match_point] : pairs) {
        const auto observed = query_points.find(feature_of.at(query_point.value));
        if (observed == query_points.end() || observed->second.value >= first_new) continue;
        ++fused;  // a map-1 point
        EXPECT_EQ(atlas_.TranslatePoint(map::MapId{2}, query_point), observed->second);
    }
    RecordProperty("fused_pairs", std::to_string(fused) + " / " + std::to_string(pairs.size()));
    EXPECT_GE(fused, 9 * pairs.size() / 10);
}

// Step 5: map 1's Frames stay exactly where they were; the window, placed by
// the disturbed similarity, is welded onto map 1's points.
TEST_F(MapperMergeTest, WeldMovesOnlyCurrentWindow) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    std::map<uint32_t, geometry::RigidTransformation> map_one;
    for (const map::FrameId frame : MapOne().keyframe_ids()) map_one.emplace(frame.value, MapOne().keyframe(frame)->T_cw);
    const double similarity_only = SimilarityOnlyCentreError(LoopSide::Return);

    MergeScene();
    for (const auto& [frame, T_cw] : map_one) {
        const geometry::RigidTransformation& after = replica().keyframe(map::FrameId{frame})->T_cw;
        EXPECT_EQ(after.translation(), T_cw.translation()) << "Frame " << frame;
        EXPECT_EQ(after.rotation(), T_cw.rotation()) << "Frame " << frame;
    }
    const SideError window = PoseError(LoopSide::Return);
    RecordProperty("window_centre_m", std::to_string(similarity_only) + " -> " + std::to_string(window.centre));
    RecordProperty("window_rotation_deg", std::to_string(window.rotation));
    EXPECT_GT(similarity_only, 0.03);
    EXPECT_LT(window.centre, 0.25 * similarity_only);
    EXPECT_LT(window.rotation, 0.3);
    EXPECT_LT(PointError(Query()), 0.1);
}

// Step 6: the middle, outside the window, takes the similarity and then
// follows the welded window through the essential graph, its points with it.
TEST_F(MapperMergeTest, MovesRestOfAbsorbedMap) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const double similarity_only = SimilarityOnlyCentreError(LoopSide::Middle);

    const Published merged = MergeScene();
    const SideError middle = PoseError(LoopSide::Middle);
    const double points = PointError(frames_[kMapOneFrames + 1]);  // M1
    RecordProperty("middle_centre_m", std::to_string(similarity_only) + " -> " + std::to_string(middle.centre));
    RecordProperty("middle_rotation_deg", std::to_string(middle.rotation));
    RecordProperty("middle_points_m", std::to_string(points));
    EXPECT_GT(similarity_only, 0.03);
    EXPECT_LT(middle.centre, 0.25 * similarity_only);
    EXPECT_LT(middle.rotation, 0.3);
    EXPECT_LT(points, 0.1);
    // Every absorbed Frame's pose is published.
    std::set<map::FrameId> posed;
    for (const auto& [frame, pose] : merged.updated_poses) posed.insert(frame);
    std::set<map::FrameId> absorbed = SideFrames(LoopSide::Middle);
    absorbed.merge(SideFrames(LoopSide::Return));
    EXPECT_EQ(posed, absorbed);
}

// Step 8: one transaction of map 1 retires map 2; the AtlasReplica applying
// it holds one consistent map equal to the Mapper's.
TEST_F(MapperMergeTest, PublishesOneMergeTransaction) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const auto start = std::chrono::steady_clock::now();
    PublishSceneMerge();
    Drain();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const std::vector<Published> published = TakeCaptured();
    RecordProperty("merge_ms", std::to_string(ms));
    ASSERT_EQ(published.size(), 1u);
    const Published& merged = published[0];
    EXPECT_EQ(merged.map_id, map::MapId{1});
    EXPECT_EQ(merged.absorbed_map, map::MapId{2});
    EXPECT_EQ(merged.removed_maps, std::vector<map::MapId>{map::MapId{2}});
    EXPECT_EQ(merged.active_map, map::MapId{1});
    EXPECT_EQ(merged.map_epoch, 1u);
    EXPECT_FALSE(merged.new_map);
    EXPECT_TRUE(merged.frames.empty());
    EXPECT_EQ(merged.count, 0u);
    EXPECT_EQ(merged.added_loop_edges,
              (std::vector<std::pair<map::FrameId, map::FrameId>>{{Query().id, Match().id}}));

    EXPECT_EQ(mapper_.active_map(), map::MapId{1});
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
    EXPECT_EQ(mapper_.map_ids(), std::vector<map::MapId>{map::MapId{1}});
    EXPECT_EQ(mapper_.num_keyframes(), kMapOneFrames + 2 * kLoopSideFrames);
    EXPECT_EQ(mapper_.num_merges(), 1u);
    EXPECT_EQ(atlas_.map_ids(), std::vector<map::MapId>{map::MapId{1}});
    EXPECT_EQ(atlas_.active_map(), map::MapId{1});
    EXPECT_EQ(atlas_.map(map::MapId{2}), nullptr);
    EXPECT_EQ(replica().map_epoch(), 1u);
    ASSERT_TRUE(replica().CheckConsistency());
    ExpectReplicaConsistent();
    for (const map::FrameId frame : replica().keyframe_ids()) {
        const geometry::RigidTransformation expected = MapperTestAccess::Pose(mapper_, frame);
        EXPECT_LT((replica().keyframe(frame)->T_cw.translation() - expected.translation()).norm(), 1e-12);
        EXPECT_LT((replica().keyframe(frame)->T_cw.rotation() - expected.rotation()).norm(), 1e-12);
    }
    for (uint32_t p = 0; p < replica().num_points(); ++p) {
        if (!replica().IsAlive({p})) continue;
        EXPECT_LT((replica().Position({p}) - MapperTestAccess::Position(mapper_, map::MapPointId{p})).norm(), 1e-12)
            << "point " << p;
    }
}

// Rejection: the query map must be the active one and the match map a live
// inactive one, unknown or removed maps included.
TEST_F(MapperMergeTest, RejectsActiveOrRemovedTarget) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    PublishLoop(Match().id, Query().id, MergeS(), {}, /*query_map=*/1, /*match_map=*/2);  // query map inactive
    PublishLoop(Query().id, Match().id, MergeS(), MergePairs(), 2, 9);  // unknown match map
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    MergeScene();
    PublishLoop(Match().id, Query().id, MergeS(), {}, 1, 2);  // removed match map
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_merges(), 1u);
    EXPECT_EQ(mapper_.map_ids(), std::vector<map::MapId>{map::MapId{1}});
}

// Rejection: the query or the match Frame was culled (through the Mapper's
// Frame culling, RemoveFrame) after the LoopCloser saw it: S2 while map 1 was
// active (map 1 gets an eleventh view first, so it is still kept), R2 in map
// 2. A Frame of the other map, or an unknown id, is rejected the same way.
TEST_F(MapperMergeTest, RejectsCulledFrames) {
    before_map_one_lost_ = [&] {
        // An eleventh view, so map 1 keeps 10 Frames after the cull.
        AddSceneView(LoopSide::Start, StartPose(kMapOneFrames), StartPose(kMapOneFrames));
        MapperTestAccess::CullFrame(mapper_, frames_[2].id);
        EXPECT_EQ(TakeCaptured().size(), 1u);
    };
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const SceneFrame& culled_query = frames_[frames_.size() - 2];  // R2
    const auto culled_query_pairs = PairsBetween(culled_query, MapTwo(), LoopSide::Start, MapOne());
    ASSERT_GE(culled_query_pairs.size(), 100u);
    MapperTestAccess::CullFrame(mapper_, culled_query.id);
    ASSERT_EQ(TakeCaptured().size(), 1u);
    ASSERT_EQ(MapOne().keyframe(frames_[2].id), nullptr);
    ASSERT_EQ(MapTwo().keyframe(culled_query.id), nullptr);
    const geometry::Sim3 S = ToSim3(culled_query.T_true * frames_[2].T_true.Inverse());
    PublishLoop(culled_query.id, Match().id, MergeS(), culled_query_pairs, 2, 1);  // query culled
    PublishLoop(Query().id, frames_[2].id, S, MergePairs(), 2, 1);                  // match culled
    PublishLoop(frames_[3].id, Match().id, MergeS(), MergePairs(), 2, 1);          // query of map 1
    PublishLoop(Query().id, map::FrameId{1000}, MergeS(), MergePairs(), 2, 1);    // unknown
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.active_map(), map::MapId{2});
    EXPECT_EQ(mapper_.num_merges(), 0u);
    MergeScene();  // nothing was changed
    EXPECT_EQ(mapper_.num_merges(), 1u);
}

// Rejection while a merge is under way: the LoopCloser may publish the merge
// again, or a loop of the absorbed map, before the first merge reaches its
// replica. Queued behind the merge they name a map the merge removed.
TEST_F(MapperMergeTest, RejectsWhileCorrecting) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    PublishSceneMerge();
    PublishSceneMerge();
    PublishLoop(Query().id, frames_[kMapOneFrames].id, geometry::Sim3{}, {}, 2, 2);
    Drain();
    const std::vector<Published> published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    EXPECT_EQ(published[0].absorbed_map, map::MapId{2});
    EXPECT_EQ(mapper_.num_merges(), 1u);
    EXPECT_EQ(replica().map_epoch(), 1u);
}

// Review Focus 1: a FrameTracked of the absorbed map, or of the target before
// the merge's epoch, creates no Frame and counts nothing; one of the merged
// map at its epoch does.
TEST_F(MapperMergeTest, IgnoresFrameTrackedOfAbsorbedMap) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const std::vector<std::pair<uint32_t, map::MapPointId>> old_pairs = MapTwo().keyframe(Query().id)->observations;
    MergeScene();
    const std::vector<uint32_t> visible = MapperTestAccess::NumVisible(mapper_, map::MapId{1});
    const std::vector<uint32_t> found = MapperTestAccess::NumFound(mapper_, map::MapId{1});
    const size_t num_frames = mapper_.num_keyframes();

    PublishReturnFrameTracked(2, 0, old_pairs);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(MapperTestAccess::NumVisible(mapper_, map::MapId{1}), visible);
    EXPECT_EQ(MapperTestAccess::NumFound(mapper_, map::MapId{1}), found);
    EXPECT_EQ(mapper_.num_keyframes(), num_frames);

    const std::vector<std::pair<uint32_t, map::MapPointId>> pairs = replica().keyframe(Query().id)->observations;
    const size_t stale_before = mapper_.stale_epoch_skipped();
    PublishReturnFrameTracked(1, 0, pairs);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.stale_epoch_skipped(), stale_before + 1);

    PublishReturnFrameTracked(1, 1, pairs);
    Drain();
    EXPECT_EQ(TakeCaptured().size(), 2u);
    EXPECT_EQ(mapper_.num_keyframes(), num_frames + 1);
}

// A final TrackingLost of the absorbed map that the merge overtook is stale;
// the Tracker announces it again for the target (tracker.h, map merging), and
// that one is a final loss as any other: the target is kept inactive and the
// next initialization builds a new map.
TEST_F(MapperMergeTest, ReannouncedLossAfterMergeDeactivatesTarget) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    MergeScene();
    ASSERT_EQ(mapper_.active_map(), map::MapId{1});

    PublishTrackingLost(2, 20.0);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.active_map(), map::MapId{1});

    PublishTrackingLost(1, 20.0);
    Drain();
    const std::vector<Published> lost = TakeCaptured();
    ASSERT_EQ(lost.size(), 1u);
    EXPECT_FALSE(lost[0].active_map.has_value());
    EXPECT_TRUE(lost[0].removed_maps.empty());  // kept inactive
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_EQ(mapper_.map_ids(), (std::vector<map::MapId>{map::MapId{1}}));

    InitializeStereo(21.0);
    ASSERT_FALSE(HasFatalFailure());
    EXPECT_EQ(mapper_.active_map(), map::MapId{3});
    EXPECT_EQ(mapper_.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{3}}));
}

// Review Focus 2: a global BA of the absorbed map, or of the target before
// the merge, is not merged.
TEST_F(MapperMergeTest, RejectsGlobalBaOfAbsorbedMap) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    MergeScene();
    const size_t rejected = mapper_.num_global_ba_rejected();
    PublishGlobalBa(2, 0);
    PublishGlobalBa(1, 0);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_global_ba_rejected(), rejected + 2);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 0u);
    PublishGlobalBa(1, 1);  // the merged map's own
    Drain();
    EXPECT_EQ(TakeCaptured().size(), 1u);
    EXPECT_EQ(mapper_.num_global_ba_merges(), 1u);
}

// Review Focus 3: once applied, the merge found again names a removed map,
// and as a loop of the merged map it is already closed.
TEST_F(MapperMergeTest, IgnoresRepeatOfAppliedMerge) {
    BuildMergeScene();
    ASSERT_FALSE(HasFatalFailure());
    const auto pairs = MergePairs();
    MergeScene();
    PublishLoop(Query().id, Match().id, MergeS(), pairs, 2, 1);
    PublishLoop(Query().id, Match().id, TrueS(), {}, 1, 1);
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_merges(), 1u);
    EXPECT_EQ(replica().map_epoch(), 1u);
}

// Critical 1 (review): merges chain. Map 3 merges into map 1, which becomes
// active holding Frames 0..9 and 20..27; then map 1 merges into map 2, whose
// Frame ids (10..19) lie between the absorbed map's. One map remains, with
// every Frame of the three, equal in the replica and the Mapper.
TEST_F(MapperMergeTest, ChainedMergesThreeMaps) {
    BuildThreeMapScene();
    ASSERT_FALSE(HasFatalFailure());
    const SceneFrame& query_a = frames_.back();      // R3, map 3
    const SceneFrame& match_a = frames_[kLoopMatch];  // S1, map 1
    PublishLoop(query_a.id, match_a.id,
                geometry::Compose(MergeError(), ToSim3(query_a.T_true * match_a.T_true.Inverse())),
                PairsBetween(query_a, *atlas_.map(map::MapId{3}), LoopSide::Start, *atlas_.map(map::MapId{1})), 3,
                1);
    Drain();
    std::vector<Published> published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    ASSERT_EQ(published[0].absorbed_map, map::MapId{3});
    ASSERT_EQ(mapper_.active_map(), map::MapId{1});

    const SceneFrame& query_b = frames_[2 * kMapOneFrames + 3];  // map 3's M3, now in map 1
    const SceneFrame& match_b = frames_[kMapOneFrames + 2];      // map 2's view at MiddlePose(2)
    ASSERT_EQ(query_b.side, LoopSide::Middle);
    ASSERT_LT(match_b.id, query_b.id);
    const auto pairs_b = PairsBetween(query_b, replica(), LoopSide::Middle, *atlas_.map(map::MapId{2}));
    ASSERT_GE(pairs_b.size(), 100u);
    PublishLoop(query_b.id, match_b.id, ToSim3(query_b.T_true * match_b.T_true.Inverse()), pairs_b, 1, 2);
    Drain();
    published = TakeCaptured();
    ASSERT_EQ(published.size(), 1u);
    EXPECT_EQ(published[0].map_id, map::MapId{2});
    EXPECT_EQ(published[0].absorbed_map, map::MapId{1});

    EXPECT_EQ(mapper_.map_ids(), std::vector<map::MapId>{map::MapId{2}});
    EXPECT_EQ(mapper_.active_map(), map::MapId{2});
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{2});
    EXPECT_EQ(mapper_.num_merges(), 2u);
    EXPECT_EQ(atlas_.map_ids(), std::vector<map::MapId>{map::MapId{2}});
    EXPECT_EQ(atlas_.active_map(), map::MapId{2});
    std::vector<map::FrameId> all;
    for (const SceneFrame& frame : frames_) all.push_back(frame.id);
    std::sort(all.begin(), all.end());
    EXPECT_EQ(replica().keyframe_ids(), all);
    EXPECT_EQ(mapper_.num_keyframes(), all.size());
    for (const map::FrameId frame : all) EXPECT_NE(mapper_.KeyFrame(frame), nullptr) << "Frame " << frame.value;
    ASSERT_TRUE(replica().CheckConsistency());  // CheckGraph included
    ExpectReplicaConsistent();
    // One tree, rooted at map 2's root.
    std::vector<map::FrameId> roots;
    for (const map::FrameId frame : all) {
        if (!replica().parent(frame)) roots.push_back(frame);
    }
    EXPECT_EQ(roots, std::vector<map::FrameId>{frames_[kMapOneFrames].id});
    for (const map::FrameId frame : all) {
        const geometry::RigidTransformation expected = MapperTestAccess::Pose(mapper_, frame);
        EXPECT_LT((replica().keyframe(frame)->T_cw.translation() - expected.translation()).norm(), 1e-12);
    }
    // The merged map keeps mapping: a Frame of map 2 at its epoch is created.
    PublishReturnFrameTracked(2, replica().map_epoch(), replica().keyframe(frames_.back().id)->observations);
    Drain();
    EXPECT_EQ(TakeCaptured().size(), 2u);
    EXPECT_EQ(mapper_.num_keyframes(), all.size() + 1);
}
} // namespace
} // namespace voyahslam::mapping
