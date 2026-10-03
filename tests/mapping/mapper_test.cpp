#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/memory/object_pool.h>

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
                p.frame_id = static_cast<TKeyFrameCreated*>(event)->frame_id;
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
    TMapper mapper_{MapperOptions{.min_mono_init_interval = 1.0}};
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

TEST_F(MapperTest, MonoInitializesAfterInterval) {
    PublishInitializationMeasurement(test::MakeMonocular(200, 0.0));
    PublishInitializationMeasurement(test::MakeMonocular(200, 0.5));
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);

    PublishInitializationMeasurement(test::MakeMonocular(200, 1.0));
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
    PublishInitializationMeasurement(test::MakeMonocular(200, 0.0));
    PublishInitializationMeasurement(test::MakeMonocular(50, 0.6));
    PublishInitializationMeasurement(test::MakeMonocular(200, 1.2));
    Drain();
    EXPECT_TRUE(TakeCaptured().empty());
    EXPECT_EQ(mapper_.state(), MapperState::Initializing);

    PublishInitializationMeasurement(test::MakeMonocular(200, 2.2));
    Drain();

    const auto published = TakeCaptured();
    ASSERT_EQ(published.size(), 3u);
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
