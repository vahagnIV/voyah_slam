#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/memory/object_pool.h>

#include <cstddef>
#include <variant>
#include <vector>

namespace voyahslam::mapping {
namespace {

using Extractor = test::TestExtractor;
using TMapper = Mapper<Extractor>;
using TKeyFrameCreated = events::KeyFrameCreated<Extractor>;
using TInitializationMeasurement = events::InitializationMeasurement<Extractor>;
using TMonocularMeasurement = features::TMonocularMeasurement<Extractor>;
using TBinocularMeasurement = features::TBinocularMeasurement<Extractor>;
using TMapPointsAdded = events::MapPointsAdded<Extractor>;
using TDescriptors = typename Extractor::TDescriptors;

struct Published {
    events::EventType type;
    map::MapId map_id;
    map::FrameId frame_id;
    double timestamp = 0.0;  // of the keyframe's measurement (KeyFrameCreated only)
    geometry::RigidTransformation pose;  // KeyFrameCreated, MapCreated
    // KeyFrameCreated: observations and the measurement's (left image's)
    // descriptors; per left feature stereo depths (binocular only).
    std::vector<uint32_t> observed_features;
    std::vector<map::MapPointId> observed_points;
    TDescriptors descriptors;
    geometry::TRowVectorF depths;
    // MapPointsAdded.
    map::MapPointId start_id{};
    uint32_t count = 0;
    geometry::TPoints3DF positions;
    TDescriptors point_descriptors;
    geometry::TPoints3DF normals;
};

// Feeds the Mapper's inbox from `tracker_` (stands in for the Tracker) and
// captures what the Mapper publishes in `capture_`.
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
            Published p{event->type, event->map_id, map::FrameId{0}};
            if (event->type == events::EventType::KeyFrameCreated) {
                const auto* created = static_cast<TKeyFrameCreated*>(event);
                p.frame_id = created->frame_id;
                p.pose = created->pose;
                p.timestamp = std::visit([](const auto& m) { return m.timestamp(); },
                                         created->measurement);
                p.observed_features = created->observed_features;
                p.observed_points = created->observed_points;
                if (const auto* mono = std::get_if<TMonocularMeasurement>(&created->measurement)) {
                    p.descriptors = mono->descriptors();
                } else if (const auto* binocular =
                               std::get_if<TBinocularMeasurement>(&created->measurement)) {
                    p.descriptors = binocular->left().descriptors();
                    p.depths = binocular->depths();
                }
            } else if (event->type == events::EventType::MapCreated) {
                const auto* created = static_cast<events::MapCreated*>(event);
                p.frame_id = created->frame_id;
                p.pose = created->pose;
            } else if (event->type == events::EventType::MapPointsAdded) {
                const auto* added = static_cast<TMapPointsAdded*>(event);
                p.start_id = added->start_id;
                p.count = added->count();
                p.positions = added->positions.matrix();
                p.point_descriptors = added->descriptors.matrix();
                p.normals = added->normals.matrix();
            }
            published.push_back(p);
        }
        return published;
    }

    static void ExpectPublished(const Published& actual, events::EventType type, uint32_t map,
                                uint32_t frame) {
        EXPECT_EQ(actual.type, type);
        EXPECT_EQ(actual.map_id, map::MapId{map});
        EXPECT_EQ(actual.frame_id, map::FrameId{frame});
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
    events::Observable tracker_;
    test::SyntheticSequence sequence_ = test::MakeSyntheticSequence();
    TMapper mapper_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
};

TEST_F(MapperTest, StereoInitializesFromOneMeasurement) {
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{0});

    PublishInitializationMeasurement(StereoMeasurement(0.0));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 3u);
    ExpectPublished(published[0], events::EventType::MapPointsAdded, 1, 0);
    ExpectPublished(published[1], events::EventType::KeyFrameCreated, 1, 0);
    ExpectPublished(published[2], events::EventType::MapCreated, 1, 0);
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
    ASSERT_EQ(published.size(), 3u);
    const Published& points = published[0];
    const Published& keyframe = published[1];
    ASSERT_EQ(points.type, events::EventType::MapPointsAdded);
    ASSERT_EQ(keyframe.type, events::EventType::KeyFrameCreated);
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

TEST_F(MapperTest, StereoPublishesPointsKeyFrameThenMap) {
    PublishInitializationMeasurement(StereoMeasurement(0.0));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 3u);
    EXPECT_EQ(published[0].type, events::EventType::MapPointsAdded);
    EXPECT_EQ(published[1].type, events::EventType::KeyFrameCreated);
    EXPECT_EQ(published[2].type, events::EventType::MapCreated);
    for (const Published& p : published) {
        EXPECT_EQ(p.map_id, map::MapId{1});
    }

    const Published& points = published[0];
    const Published& keyframe = published[1];
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
    ASSERT_EQ(second.size(), 3u);
    ExpectPublished(second[0], events::EventType::MapPointsAdded, 2, 0);
    ExpectPublished(second[1], events::EventType::KeyFrameCreated, 2, 2);
    ExpectPublished(second[2], events::EventType::MapCreated, 2, 2);
    EXPECT_EQ(second[0].start_id, map::MapPointId{0});
    EXPECT_EQ(second[0].count, num_depths);
    EXPECT_EQ(mapper_.num_map_points(), num_depths);
    EXPECT_EQ(second[1].observed_points.size(), num_depths);
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
    ASSERT_EQ(published.size(), 4u);
    EXPECT_EQ(published[0].type, events::EventType::MapPointsAdded);
    ExpectPublished(published[1], events::EventType::KeyFrameCreated, 1, 0);
    ExpectPublished(published[2], events::EventType::KeyFrameCreated, 1, 1);
    ExpectPublished(published[3], events::EventType::MapCreated, 1, 1);
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.num_keyframes(), 2u);
}

TEST_F(MapperTest, MonoPublishesPointsKeyFramesThenMap) {
    const auto published = InitializeMono(0);

    ASSERT_EQ(published.size(), 4u);
    EXPECT_EQ(published[0].type, events::EventType::MapPointsAdded);
    EXPECT_EQ(published[1].type, events::EventType::KeyFrameCreated);
    EXPECT_EQ(published[2].type, events::EventType::KeyFrameCreated);
    EXPECT_EQ(published[3].type, events::EventType::MapCreated);
    for (const Published& p : published) {
        EXPECT_EQ(p.map_id, map::MapId{1});
    }

    const Published& points = published[0];
    EXPECT_EQ(points.start_id, map::MapPointId{0});
    EXPECT_GT(points.count, 0u);
    EXPECT_EQ(points.count, mapper_.num_map_points());
    ASSERT_EQ(points.positions.cols(), points.count);
    ASSERT_EQ(points.point_descriptors.cols(), points.count);
    ASSERT_EQ(points.normals.cols(), points.count);
    for (uint32_t i = 0; i < points.count; ++i) {
        EXPECT_NEAR(points.normals.col(i).norm(), 1.0, 1e-5) << "point " << i;
        // The normal points from the scene towards both camera centres.
        for (size_t k = 1; k <= 2; ++k) {
            const geometry::TVector3DF centre = published[k].pose.Inverse().translation();
            const geometry::TVector3DF position = points.positions.col(i);
            EXPECT_GT(points.normals.col(i).dot(position - centre), 0.0)
                << "point " << i << " keyframe " << k;
        }
    }

    for (size_t k = 1; k <= 2; ++k) {
        const Published& keyframe = published[k];
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
    for (size_t o = 0; o < published[1].observed_points.size(); ++o) {
        scene_point_in_first[published[1].observed_points[o].value] =
            first_of_feature[published[1].observed_features[o]];
    }
    for (size_t o = 0; o < published[2].observed_points.size(); ++o) {
        const int64_t in_second = second_of_feature[published[2].observed_features[o]];
        const int64_t in_first = scene_point_in_first[published[2].observed_points[o].value];
        EXPECT_GE(in_first, 0) << "observation " << o;
        EXPECT_EQ(in_first, in_second) << "observation " << o;
    }

    // A point's descriptor is the first keyframe's descriptor of its feature.
    const Published& first = published[1];
    for (size_t o = 0; o < first.observed_points.size(); ++o) {
        EXPECT_EQ(points.point_descriptors.col(first.observed_points[o].value),
                  first.descriptors.col(first.observed_features[o]))
            << "observation " << o;
    }
}

TEST_F(MapperTest, MapCreatedCarriesSecondKeyFramePose) {
    const auto published = InitializeMono(0);

    ASSERT_EQ(published.size(), 4u);
    EXPECT_EQ(published[3].frame_id, published[2].frame_id);
    EXPECT_TRUE(published[3].pose.ToMatrix34().isApprox(published[2].pose.ToMatrix34()));
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
    ASSERT_FALSE(second.empty());
    EXPECT_EQ(second[0].type, events::EventType::MapPointsAdded);
    EXPECT_EQ(second[0].map_id, map::MapId{2});
    EXPECT_EQ(second[0].start_id, map::MapPointId{0});
    EXPECT_EQ(second[0].count, mapper_.num_map_points());
}

TEST_F(MapperTest, PooledKeyFrameCreatedIsReset) {
    memory::ObjectPool<TKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    auto* event = pool.Create();
    event->observed_features = {1, 2, 3};
    event->observed_points = {map::MapPointId{0}, map::MapPointId{1}, map::MapPointId{2}};
    event->Recycle();

    auto* reused = pool.Create();
    ASSERT_EQ(reused, event);  // the same slot
    EXPECT_TRUE(reused->observed_features.empty());
    EXPECT_TRUE(reused->observed_points.empty());
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
    ASSERT_EQ(published.size(), 4u);
    EXPECT_EQ(published[0].type, events::EventType::MapPointsAdded);
    EXPECT_EQ(published[1].type, events::EventType::KeyFrameCreated);
    EXPECT_DOUBLE_EQ(published[1].timestamp, sequence_.frames[3].measurement.timestamp());
    EXPECT_EQ(published.back().type, events::EventType::MapCreated);
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
    ASSERT_EQ(published.size(), 3u);
    ExpectPublished(published[0], events::EventType::MapPointsAdded, 2, 0);
    EXPECT_EQ(published[0].start_id, map::MapPointId{0});
    EXPECT_GE(published[0].count, 100u);
    EXPECT_EQ(published[0].count, mapper_.num_map_points());
    ExpectPublished(published[1], events::EventType::KeyFrameCreated, 2, 1);
    ExpectPublished(published[2], events::EventType::MapCreated, 2, 1);
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

TEST_F(MapperTest, StateIsMappingWhenMapCreatedIsPublished) {
    // A synchronous observer runs on the publishing thread, inside Notify, so
    // it sees the state at the exact moment MapCreated is published.
    struct Probe {
        TMapper* mapper;
        bool seen = false;
        MapperState state = MapperState::Initializing;
    } probe{&mapper_};
    mapper_.AddSyncObserver(
        [](void* ctx, events::Event* event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event->type == events::EventType::MapCreated) {
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

} // namespace
} // namespace voyahslam::mapping
