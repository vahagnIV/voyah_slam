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

struct Published {
    events::EventType type;
    map::MapId map_id;
    map::FrameId frame_id;
    double timestamp = 0.0;  // of the keyframe's measurement (KeyFrameCreated only)
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
                p.timestamp = std::visit([](const auto& m) { return m.timestamp(); },
                                         created->measurement);
            } else if (event->type == events::EventType::MapCreated) {
                p.frame_id = static_cast<events::MapCreated*>(event)->frame_id;
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

    // Initializes a map from one stereo measurement and discards the output.
    void InitializeStereo(double timestamp) {
        PublishInitializationMeasurement(test::MakeBinocular(200, timestamp));
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

    PublishInitializationMeasurement(test::MakeBinocular(200, 0.0));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    ExpectPublished(published[0], events::EventType::KeyFrameCreated, 1, 0);
    ExpectPublished(published[1], events::EventType::MapCreated, 1, 0);
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
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
    ASSERT_EQ(published.size(), 3u);
    ExpectPublished(published[0], events::EventType::KeyFrameCreated, 1, 0);
    ExpectPublished(published[1], events::EventType::KeyFrameCreated, 1, 1);
    ExpectPublished(published[2], events::EventType::MapCreated, 1, 1);
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
    EXPECT_EQ(mapper_.num_keyframes(), 2u);
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
    ASSERT_EQ(published.size(), 3u);
    EXPECT_EQ(published[0].type, events::EventType::KeyFrameCreated);
    EXPECT_DOUBLE_EQ(published[0].timestamp, sequence_.frames[3].measurement.timestamp());
    EXPECT_EQ(published.back().type, events::EventType::MapCreated);
    EXPECT_EQ(mapper_.state(), MapperState::Mapping);
}

TEST_F(MapperTest, IgnoresInitializationMeasurementWhileMapping) {
    InitializeStereo(0.0);

    PublishInitializationMeasurement(test::MakeBinocular(200, 1.0));
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
    EXPECT_TRUE(TakeCaptured().empty());

    PublishInitializationMeasurement(test::MakeBinocular(200, 2.0));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 2u);
    ExpectPublished(published[0], events::EventType::KeyFrameCreated, 2, 1);
    ExpectPublished(published[1], events::EventType::MapCreated, 2, 1);
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

    PublishInitializationMeasurement(test::MakeBinocular(200, 0.0));
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
