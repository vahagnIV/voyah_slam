#include <voyahslam/memory/column_buffer.h>

#include <gtest/gtest.h>

namespace voyahslam::memory {
namespace {

using TBuffer = ColumnBuffer<double, 3>;

TEST(ColumnBufferTest, StartsEmpty) {
    TBuffer buffer;
    EXPECT_EQ(buffer.cols(), 0);
    EXPECT_EQ(buffer.capacity(), 0);
    EXPECT_EQ(buffer.matrix().cols(), 0);
}

TEST(ColumnBufferTest, ResizeExposesExactlyRequestedColumns) {
    TBuffer buffer;
    buffer.Resize(5);
    EXPECT_EQ(buffer.cols(), 5);
    EXPECT_EQ(buffer.matrix().rows(), 3);
    EXPECT_EQ(buffer.matrix().cols(), 5);
    EXPECT_GE(buffer.capacity(), 5);
}

TEST(ColumnBufferTest, ShrinkAndRegrowWithinCapacityDoNotReallocate) {
    TBuffer buffer;
    buffer.Resize(100);
    const double* data = buffer.matrix().data();
    Eigen::Index capacity = buffer.capacity();

    buffer.Resize(10);
    EXPECT_EQ(buffer.matrix().data(), data);
    buffer.Clear();
    EXPECT_EQ(buffer.cols(), 0);
    buffer.Resize(100);
    EXPECT_EQ(buffer.matrix().data(), data);
    EXPECT_EQ(buffer.capacity(), capacity);
}

TEST(ColumnBufferTest, GrowingPreservesExistingColumns) {
    TBuffer buffer;
    buffer.Resize(2);
    buffer.matrix() << 1, 2,
                       3, 4,
                       5, 6;
    buffer.Resize(1000);
    EXPECT_EQ(buffer.matrix().col(0), Eigen::Vector3d(1, 3, 5));
    EXPECT_EQ(buffer.matrix().col(1), Eigen::Vector3d(2, 4, 6));
}

TEST(ColumnBufferTest, ColumnWritesAreVisibleThroughMatrix) {
    ColumnBuffer<uint8_t, 32> buffer;
    buffer.Resize(3);
    buffer.col(1).setConstant(7);
    EXPECT_EQ(buffer.matrix()(0, 1), 7);
    EXPECT_EQ(buffer.matrix()(31, 1), 7);
}

} // namespace
} // namespace voyahslam::memory
