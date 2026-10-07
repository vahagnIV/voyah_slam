#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/map_replica.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
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
    // The parts nothing publishes yet: removed observations, replaced and
    // removed points, updated poses, removed Frames.
    size_t num_other_parts = 0;

    // The `k`th added Frame.
    [[nodiscard]] const PublishedFrame& frame(size_t k = 0) const { return frames.at(k); }
};

// Feeds the Mapper's inbox from `tracker_` (stands in for the Tracker) and
// captures what the Mapper publishes in `capture_`; every captured event is
// also applied to `replica_`, as the Tracker would, and must leave it consistent.
class MapperTest : public ::testing::Test {
protected:
    MapperTest() {
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

    // Everything captured so far.
    std::vector<Published> TakeCaptured() {
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
            }
            published.push_back(p);
        }
        return published;
    }

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
    // inliers (num_inliers = pairs.size() unless given), posed at Frame 1
    // unless `pose` is given.
    void PublishFrameTracked(uint32_t map, size_t index, const std::vector<Pair>& pairs,
                             std::optional<map::FrameId> reference = map::FrameId{1},
                             std::optional<size_t> num_inliers = std::nullopt,
                             std::optional<geometry::RigidTransformation> pose = std::nullopt) {
        auto* event = frame_tracked_pool_.Create();
        event->map_id = map::MapId{map};
        event->timestamp = sequence_.frames[index].measurement.timestamp();
        event->pose = pose.value_or(second_.pose);
        event->measurement = sequence_.frames[index].measurement;
        for (const Pair& pair : pairs) {
            event->observed_features.push_back(pair.feature);
            event->observed_points.push_back(pair.point);
        }
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
class MapperFrameCreationTest : public MapperFrameTrackedTest {
protected:
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
    ASSERT_EQ(published.size(), 1u);
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
    ASSERT_EQ(published.size(), 1u);
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
    ASSERT_EQ(TakeCaptured().size(), 1u);
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
    ASSERT_EQ(published.size(), 1u);
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
    ASSERT_EQ(published.size(), 1u);
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
    ASSERT_EQ(published.size(), 1u);
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
    ASSERT_EQ(published.size(), 1u);
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
    ASSERT_EQ(before.size(), 1u);
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
    ASSERT_EQ(published.size(), 1u);
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
        ASSERT_EQ(published.size(), 1u) << "frame " << index;
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
        ASSERT_EQ(published.size(), 1u) << "frame " << index;
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

} // namespace
} // namespace voyahslam::mapping
