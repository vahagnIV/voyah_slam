#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/map_replica.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <numeric>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace voyahslam::mapping {
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

    // The `k`th added Frame.
    [[nodiscard]] const PublishedFrame& frame(size_t k = 0) const { return frames.at(k); }
};

// Feeds the Mapper's inbox from `tracker_` (stands in for the Tracker) and
// captures what the Mapper publishes in `capture_`; every captured event is
// also applied to `replica_`, as the Tracker would, and must leave it consistent.
class MapperTest : public ::testing::Test {
protected:
    explicit MapperTest(MapperOptions options = {})
        : mapper_(options) {
        tracker_.ConnectAsync(mapper_.inbox());
        mapper_.AddObserver(capture_);
    }
    MapperTest(MapperOptions options, std::shared_ptr<const TVocabulary> vocabulary)
        : mapper_(options, std::move(vocabulary)) {
        tracker_.ConnectAsync(mapper_.inbox());
        mapper_.AddObserver(capture_);
    }

    void Drain() {
        while (mapper_.ProcessPending()) {
        }
    }

    template <typename TMeasurement>
    void PublishInitializationMeasurement(TMeasurement measurement) {
        auto* event = initialization_pool_.Create();
        event->map_id = map::MapId{0};
        event->measurement = std::move(measurement);
        tracker_.Notify(event);
    }

    void PublishTrackingLost(uint32_t map, double timestamp) {
        auto* event = tracking_lost_pool_.Create();
        event->map_id = map::MapId{map};
        event->timestamp = timestamp;
        tracker_.Notify(event);
    }

    // Everything captured so far; `after_each` runs once each event has been
    // applied to the replica.
    std::vector<Published> TakeCaptured(
        const std::function<void(const Published&)>& after_each = {}) {
        std::vector<Published> published;
        while (events::Event* event = listener_.GetNextEvent()) {
            replica_.Apply(*event);
            EXPECT_TRUE(replica_.CheckConsistency());
            Published p{event->type, event->map_id};
            if (event->type == events::EventType::MapTransaction) {
                const auto* transaction = static_cast<TTransaction*>(event);
                p.new_map = transaction->new_map;
                p.start_frame = transaction->start_frame;
                for (const events::NewFrame<Extractor>& added : transaction->added_frames) {
                    PublishedFrame& frame = p.frames.emplace_back();
                    frame.frame_id = added.id;
                    frame.pose = added.T_cw;
                    frame.timestamp =
                        std::visit([](const auto& m) { return m.timestamp(); }, added.measurement);
                    frame.observed_features = added.observed_features;
                    frame.observed_points = added.observed_points;
                    frame.bow = added.bow;
                    frame.features = added.features;
                    if (const auto* mono = std::get_if<TMonocularMeasurement>(&added.measurement)) {
                        frame.descriptors = mono->descriptors();
                    } else if (const auto* binocular =
                                   std::get_if<TBinocularMeasurement>(&added.measurement)) {
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
            }
            ExpectApplied(p);
            if (after_each) {
                after_each(p);
            }
            published.push_back(p);
        }
        return published;
    }

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

    memory::ObjectPool<TInitializationMeasurement, memory::ThreadSafePolicy> initialization_pool_{
        256};
    memory::ObjectPool<events::TrackingLost, memory::ThreadSafePolicy> tracking_lost_pool_{256};
    memory::ObjectPool<TFrameTracked, memory::ThreadSafePolicy> frame_tracked_pool_{256};
    events::Observable tracker_;
    test::SyntheticSequence sequence_ = test::MakeSyntheticSequence();
    TMapper mapper_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
    tracking::MapReplica<Extractor> replica_;
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
    PublishTrackingLost(1, timestamp);
    Drain();
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

    PublishTrackingLost(1, sequence_.frames.back().measurement.timestamp());
    Drain();
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

TEST_F(MapperTest, PooledMapTransactionIsReset) {
    memory::ObjectPool<TTransaction, memory::ThreadSafePolicy> pool(1);
    auto* event = pool.Create();
    event->new_map = true;
    event->added_frames.emplace_back().observed_features = {1, 2, 3};
    event->removed_points = {map::MapPointId{0}, map::MapPointId{1}};
    event->Recycle();

    auto* reused = pool.Create();
    ASSERT_EQ(reused, event);  // the same slot
    EXPECT_FALSE(reused->new_map);
    EXPECT_TRUE(reused->added_frames.empty());
    EXPECT_TRUE(reused->removed_points.empty());
    reused->Recycle();
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
    EXPECT_TRUE(TakeCaptured().empty());

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

TEST_F(MapperTest, StateIsMappingWhenNewMapIsPublished) {
    // A synchronous observer runs on the publishing thread, inside Notify, so
    // it sees the state at the exact moment the new map is published.
    struct Probe {
        TMapper* mapper;
        bool seen = false;
        MapperState state = MapperState::Initializing;
    } probe{&mapper_};
    mapper_.AddSyncObserver(
        [](void* ctx, events::Event* event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event->type == events::EventType::MapTransaction &&
                static_cast<TTransaction*>(event)->new_map) {
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
    tracker_.ConnectAsync(other);

    PublishInitializationMeasurement(test::MakeMonocular(200, 0.0));
    Drain();

    auto* event = other_listener.GetNextEvent();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::InitializationMeasurement);
    const auto& init = *static_cast<TInitializationMeasurement*>(event);
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
    // visible points, posed at Frame 1 unless `pose` is given.
    void PublishFrameTracked(uint32_t map, size_t index, const std::vector<Pair>& pairs,
                             std::optional<map::FrameId> reference = map::FrameId{1},
                             std::optional<size_t> num_inliers = std::nullopt,
                             std::optional<geometry::RigidTransformation> pose = std::nullopt,
                             const std::vector<map::MapPointId>& visible = {}) {
        auto* event = frame_tracked_pool_.Create();
        event->map_id = map::MapId{map};
        event->timestamp = sequence_.frames[index].measurement.timestamp();
        event->pose = pose.value_or(second_.pose);
        event->measurement = sequence_.frames[index].measurement;
        for (const Pair& pair : pairs) {
            event->observed_features.push_back(pair.feature);
            event->observed_points.push_back(pair.point);
        }
        event->visible_points = visible;
        event->reference_frame_id = reference;
        event->stage = events::TrackingStage::LocalMap;
        event->num_matches = static_cast<uint32_t>(pairs.size());
        event->num_inliers = static_cast<uint32_t>(num_inliers.value_or(pairs.size()));
        tracker_.Notify(event);
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
        auto* event = frame_tracked_pool_.Create();
        event->map_id = map::MapId{1};
        event->timestamp = measurement.timestamp();
        event->pose = first.pose;
        event->measurement = std::move(measurement);
        event->observed_features.assign(first.observed_features.begin(),
                                        first.observed_features.begin() + count);
        event->observed_points.assign(first.observed_points.begin(),
                                      first.observed_points.begin() + count);
        event->reference_frame_id = first.frame_id;
        event->num_matches = event->num_inliers = static_cast<uint32_t>(count);
        tracker_.Notify(event);
    }

    // Covisibility of the Mapper and of the replica agree for every Frame.
    void ExpectReplicaConsistent() {
        const map::CovisibilityIndex& mapper = mapper_.covisibility();
        const map::CovisibilityIndex& replica = replica_.covisibility();
        const std::vector<map::FrameId> frames = replica_.keyframe_ids();
        ASSERT_EQ(frames.size(), mapper_.num_keyframes());
        for (const map::FrameId frame : frames) {
            std::vector<map::MapPointId> expected = mapper.Points(frame);
            std::vector<map::MapPointId> actual = replica.Points(frame);
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
                EXPECT_EQ(features(replica), features(mapper)) << "point " << point.value;
            }
            const size_t max_count = frames.size();
            EXPECT_EQ(replica.Covisible(frame, max_count), mapper.Covisible(frame, max_count))
                << "Frame " << frame.value;
        }
        // Same alive points, same references.
        for (uint32_t p = 0; p < replica_.num_points(); ++p) {
            const map::MapPointId point{p};
            const std::optional<map::PointReference> expected = mapper_.Reference(point);
            ASSERT_EQ(replica_.IsAlive(point), expected.has_value()) << "point " << p;
            if (!expected) {
                continue;
            }
            const std::optional<map::PointReference>& actual = replica_.Reference(point);
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
                EXPECT_TRUE(replica_.IsAlive(point)) << "Frame point " << point.value;
            }
        }
        for (const events::TransactionObservation& added : created.added_observations) {
            EXPECT_TRUE(replica_.IsAlive(added.point)) << "added point " << added.point.value;
        }
        for (const map::MapPointId point : created.updated_ids) {
            EXPECT_TRUE(replica_.IsAlive(point)) << "updated point " << point.value;
        }
        for (const events::ReplacedPoint& replaced : created.replaced_points) {
            EXPECT_TRUE(replica_.IsAlive(replaced.winner)) << "winner " << replaced.winner.value;
            EXPECT_FALSE(replica_.IsAlive(replaced.loser)) << "loser " << replaced.loser.value;
            EXPECT_NE(std::find(created.removed_points.begin(), created.removed_points.end(),
                                replaced.loser),
                      created.removed_points.end())
                << "loser " << replaced.loser.value;
        }
        for (const map::MapPointId point : created.removed_points) {
            EXPECT_FALSE(replica_.IsAlive(point)) << "removed point " << point.value;
        }
        // Posed Frames exist, removed ones do not.
        for (const auto& [frame, pose] : created.updated_poses) {
            EXPECT_NE(replica_.keyframe(frame), nullptr) << "posed Frame " << frame.value;
        }
        for (const map::FrameId frame : created.removed_frames) {
            EXPECT_EQ(replica_.keyframe(frame), nullptr) << "removed Frame " << frame.value;
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

    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.num_frame_evaluations(), evaluations);
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_EQ(mapper_.num_keyframes(), 0u);
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
    auto* event = frame_tracked_pool_.Create();
    event->map_id = map::MapId{1};
    event->timestamp = 1.0;
    event->pose = first.pose;
    event->measurement = StereoMeasurement(1.0);
    event->observed_features.assign(first.observed_features.begin(),
                                    first.observed_features.begin() + count);
    event->observed_points.assign(first.observed_points.begin(),
                                  first.observed_points.begin() + count);
    event->reference_frame_id = first.frame_id;
    event->num_matches = event->num_inliers = static_cast<uint32_t>(count);
    tracker_.Notify(event);
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

    auto* event = frame_tracked_pool_.Create();
    event->map_id = map::MapId{1};
    event->timestamp = 0.05;  // well within max_interval: c1b (idle) decides
    event->pose = first.pose;
    event->measurement = StereoMeasurement(0.05);
    event->observed_features.assign(first.observed_features.begin(),
                                    first.observed_features.begin() + count);
    event->observed_points.assign(first.observed_points.begin(),
                                  first.observed_points.begin() + count);
    event->reference_frame_id = first.frame_id;
    event->num_matches = event->num_inliers = static_cast<uint32_t>(count);
    tracker_.Notify(event);
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
        const auto& reference = replica_.Reference(added.observed_points[k]);
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
            const auto& reference = replica_.Reference(point);
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
        auto* event = frame_tracked_pool_.Create();
        event->map_id = map::MapId{1};
        event->timestamp = measurements.back().timestamp();
        event->pose = stereo.frames[index].T_cw;
        event->measurement = measurements.back();
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
        tracker_.Notify(event);
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
        auto* event = frame_tracked_pool_.Create();
        event->map_id = map::MapId{1};
        event->timestamp = timestamp;
        event->pose = pose;
        event->measurement = StereoMeasurement(timestamp);
        for (const Pair& pair : pairs) {
            event->observed_features.push_back(pair.feature);
            event->observed_points.push_back(pair.point);
        }
        event->reference_frame_id = reference;
        event->num_matches = static_cast<uint32_t>(pairs.size());
        event->num_inliers = static_cast<uint32_t>(num_inliers);
        tracker_.Notify(event);
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
            if (!replica_.IsAlive(point)) continue;
            for (const map::PointObservation& o : replica_.covisibility().Observations(point)) {
                const tracking::ReplicaKeyFrame<Extractor>* keyframe = replica_.keyframe(o.frame);
                const auto& left = std::get<TMonocularMeasurement>(keyframe->measurement);
                const geometry::TPoint2DF projected = left.camera().Project(
                    keyframe->T_cw.Transform(geometry::TPoint3DF(replica_.Position(point))));
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
    static void PublishUrgentOnFrameCreated(void* ctx, events::Event* event) {
        auto* self = static_cast<MapperQualityTest*>(ctx);
        if (self->urgent_sent_ || event->type != events::EventType::MapTransaction) return;
        const auto* transaction = static_cast<const TTransaction*>(event);
        if (transaction->new_map || transaction->added_frames.empty()) return;
        self->urgent_sent_ = true;
        auto* urgent = self->frame_tracked_pool_.Create();
        urgent->map_id = map::MapId{0};  // an old map's: ignored once drained
        urgent->priority = true;
        self->urgent_.Notify(urgent);
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
        EXPECT_FALSE(replica_.IsAlive(removed.point));
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
    const auto total = static_cast<uint32_t>(replica_.num_points() - num_points_);
    uint32_t observed_3 = 0;
    for (uint32_t p = static_cast<uint32_t>(num_points_); p < replica_.num_points(); ++p) {
        observed_3 += replica_.IsAlive(map::MapPointId{p}) &&
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
        if (point >= created.start_id && replica_.IsAlive(point)) {
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
    EXPECT_FALSE(replica_.IsAlive(*poor));
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
        for (uint32_t p = 0; p < replica_.num_points(); ++p) {
            const map::MapPointId point{p};
            if (!replica_.IsAlive(point)) continue;
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
        auto* event = frame_tracked_pool_.Create();
        event->map_id = map::MapId{1};
        event->timestamp = measurement.timestamp();
        event->pose = stereo.frames[index].T_cw;
        event->measurement = measurement;
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
        tracker_.Notify(event);
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
        if (point >= created.start_id && scene >= 0 && replica_.IsAlive(point)) {
            recent.emplace(point, scene);
            break;
        }
    }
    ASSERT_TRUE(recent.has_value());
    // Frame 3 does not track it: still 2 observations.
    CreateMonoFrame(second_index + 7);
    ASSERT_TRUE(replica_.IsAlive(recent->first));
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
        alive = replica_.IsAlive(recent->first);
        num_observations = replica_.covisibility().NumObservations(recent->first);
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
            ASSERT_NE(replica_.keyframe(id), nullptr);
            EXPECT_TRUE(replica_.keyframe(id)->T_cw.ToMatrix34().isApprox(pose.ToMatrix34()));
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
    EXPECT_TRUE(replica_.IsAlive(outlier));
    ExpectReplicaConsistent();
}

TEST_F(MapperQualityTest, UrgentEventStopsBa) {
    // The same outlier, but an urgent FrameTracked arrives once transaction 1
    // is out: BA stops before its first iteration and removes nothing.
    urgent_.ConnectAsync(mapper_.inbox());
    SetUpMonoMap();
    mapper_.AddSyncObserver(&MapperQualityTest::PublishUrgentOnFrameCreated, this);
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
            EXPECT_EQ(replica_.keyframe(culled), nullptr);
            removed.push_back(culled);
        }
        ExpectReplicaConsistent();
    }
    EXPECT_FALSE(removed.empty());
    EXPECT_EQ(mapper_.num_keyframes(), 2 + 4 - removed.size());
    EXPECT_NE(replica_.keyframe(first_.frame_id), nullptr);
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
    EXPECT_EQ(replica_.keyframe(f), nullptr);
    size_t num_alive = 0;
    for (const map::MapPointId point : referenced) {
        if (!replica_.IsAlive(point)) continue;
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

} // namespace
} // namespace voyahslam::mapping
