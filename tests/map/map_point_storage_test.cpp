#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/map/map_point_storage.h>

#include <gtest/gtest.h>

namespace voyahslam::map {
namespace {

using Storage = MapPointStorage<features::OrbFeatureExtractor>;
using TDescriptor = features::OrbFeatureExtractor::TDescriptor;

geometry::TPoint3DF MakePoint(double x, double y, double z) {
    return geometry::TPoint3DF(x, y, z);
}

TDescriptor MakeDescriptor(uint8_t fill) {
    return TDescriptor::Constant(fill);
}

TEST(MapPointStorageTest, AddAndAccess) {
    Storage storage;

    auto id0 = storage.Add(MakePoint(1, 2, 3), MakeDescriptor(10), MakePoint(0, 0, 1));
    auto id1 = storage.Add(MakePoint(4, 5, 6), MakeDescriptor(20), MakePoint(0, 1, 0));
    auto id2 = storage.Add(MakePoint(7, 8, 9), MakeDescriptor(30), MakePoint(1, 0, 0));

    EXPECT_EQ(storage.size(), 3u);

    EXPECT_DOUBLE_EQ(storage.Position(id0).x(), 1.0);
    EXPECT_DOUBLE_EQ(storage.Position(id1).y(), 5.0);
    EXPECT_DOUBLE_EQ(storage.Position(id2).z(), 9.0);

    EXPECT_EQ(storage.Descriptor(id0)(0), 10);
    EXPECT_EQ(storage.Descriptor(id1)(0), 20);

    EXPECT_DOUBLE_EQ(storage.Normal(id2).x(), 1.0);
}

TEST(MapPointStorageTest, Remove) {
    Storage storage;

    auto id0 = storage.Add(MakePoint(1, 2, 3), MakeDescriptor(10), MakePoint(0, 0, 1));
    auto id1 = storage.Add(MakePoint(4, 5, 6), MakeDescriptor(20), MakePoint(0, 1, 0));

    storage.Remove(id0);

    EXPECT_FALSE(storage.IsValid(id0));
    EXPECT_TRUE(storage.IsValid(id1));
    EXPECT_EQ(storage.size(), 1u);

    EXPECT_DOUBLE_EQ(storage.Position(id1).x(), 4.0);
}

TEST(MapPointStorageTest, ReuseSlot) {
    Storage storage;

    auto id0 = storage.Add(MakePoint(1, 2, 3), MakeDescriptor(10), MakePoint(0, 0, 1));
    storage.Remove(id0);

    auto id_new = storage.Add(MakePoint(99, 88, 77), MakeDescriptor(50), MakePoint(1, 1, 1));

    EXPECT_EQ(id_new.value, id0.value);
    EXPECT_DOUBLE_EQ(storage.Position(id_new).x(), 99.0);
    EXPECT_EQ(storage.Descriptor(id_new)(0), 50);
    EXPECT_TRUE(storage.IsValid(id_new));
}

TEST(MapPointStorageTest, BulkAccess) {
    Storage storage;

    for (int i = 0; i < 10; ++i) {
        storage.Add(MakePoint(i, i + 1, i + 2), MakeDescriptor(i), MakePoint(0, 0, 1));
    }

    EXPECT_GE(storage.positions().cols(), 10);
    EXPECT_EQ(storage.size(), 10u);

    int valid_count = 0;
    for (uint32_t i = 0; i < static_cast<uint32_t>(storage.positions().cols()); ++i) {
        if (storage.IsValid(MapPointId{i})) {
            ++valid_count;
        }
    }
    EXPECT_EQ(valid_count, 10);
}

TEST(MapPointStorageTest, RemoveInvalidIdIsNoop) {
    Storage storage;

    auto id0 = storage.Add(MakePoint(1, 2, 3), MakeDescriptor(10), MakePoint(0, 0, 1));
    storage.Remove(id0);

    storage.Remove(id0);
    EXPECT_EQ(storage.size(), 0u);
}

} // namespace
} // namespace voyahslam::map
