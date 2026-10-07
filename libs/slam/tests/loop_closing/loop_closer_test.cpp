#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/memory/object_pool.h>

namespace voyahslam::loop_closing {
namespace {

using Extractor = test::TestExtractor;

TEST(LoopCloserTest, AcknowledgesEverything) {
    memory::ObjectPool<events::MapTransaction<Extractor>, memory::ThreadSafePolicy> pool{1};
    events::Observable mapper;
    LoopCloser<Extractor> loop_closer;
    mapper.ConnectAsync(loop_closer.inbox());

    auto* event = pool.Create();
    event->map_id = map::MapId{1};
    mapper.Notify(event);
    ASSERT_EQ(pool.TryCreate(), nullptr);

    EXPECT_TRUE(loop_closer.ProcessPending());
    EXPECT_FALSE(loop_closer.ProcessPending());

    auto* reused = pool.TryCreate();
    EXPECT_NE(reused, nullptr);
    pool.Recycle(reused);
}

} // namespace
} // namespace voyahslam::loop_closing
