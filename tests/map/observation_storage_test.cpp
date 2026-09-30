#include <voyahslam/map/observation_storage.h>

#include <gtest/gtest.h>

#include <type_traits>

namespace voyahslam::map {
namespace {

TEST(ObservationStorageTest, MonocularAdd) {
    ObservationStorage<ObservationType::Monocular> storage;

    storage.Add(MapPointId{10}, FrameId{1}, 5);
    storage.Add(MapPointId{20}, FrameId{2}, 8);
    storage.Add(MapPointId{30}, FrameId{3}, 12);

    EXPECT_EQ(storage.size(), 3u);
    EXPECT_EQ(storage.mappoint_id(0), MapPointId{10});
    EXPECT_EQ(storage.frame_id(1), FrameId{2});
    EXPECT_EQ(storage.feature_index(2), 12);
}

TEST(ObservationStorageTest, StereoAdd) {
    ObservationStorage<ObservationType::Stereo> storage;

    storage.Add(MapPointId{10}, FrameId{1}, 5, 3.5, 7);
    storage.Add(MapPointId{20}, FrameId{2}, 8, 1.2, 3);

    EXPECT_EQ(storage.size(), 2u);
    EXPECT_EQ(storage.mappoint_id(0), MapPointId{10});
    EXPECT_DOUBLE_EQ(storage.depth(0), 3.5);
    EXPECT_EQ(storage.right_feature_index(0), 7);
    EXPECT_DOUBLE_EQ(storage.depth(1), 1.2);
    EXPECT_EQ(storage.right_feature_index(1), 3);
}

TEST(ObservationStorageTest, MonocularStereoOverloadDisabled) {
    using MonoStorage = ObservationStorage<ObservationType::Monocular>;
    static_assert(
        !std::is_invocable_v<
            decltype([](auto& s) -> decltype(
                s.Add(MapPointId{}, FrameId{}, uint16_t{}, precision_t{}, int16_t{})) {}),
            MonoStorage&>,
        "5-arg Add must not compile for Monocular storage");
}

TEST(ObservationStorageTest, StereoMonocularAddSetsDefaults) {
    ObservationStorage<ObservationType::Stereo> storage;

    storage.Add(MapPointId{10}, FrameId{1}, 5);

    EXPECT_EQ(storage.size(), 1u);
    EXPECT_DOUBLE_EQ(storage.depth(0), -1.0);
    EXPECT_EQ(storage.right_feature_index(0), -1);
}

} // namespace
} // namespace voyahslam::map
