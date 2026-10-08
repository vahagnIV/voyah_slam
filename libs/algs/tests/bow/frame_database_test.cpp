#include <gtest/gtest.h>

#include <cstdint>
#include <vector>
#include <voyahslam/bow/frame_database.h>

namespace voyahslam::bow {
namespace {

// L1-normalised vector with equal weights over `words`.
BowVector MakeBow(const std::vector<WordId>& words) {
    BowVector bow;
    for (const WordId word : words) {
        bow.entries.emplace_back(word, 1.0 / static_cast<double>(words.size()));
    }
    return bow;
}

std::vector<WordId> Range(WordId first, WordId last) {
    std::vector<WordId> words;
    for (WordId w = first; w < last; ++w) words.push_back(w);
    return words;
}

TEST(FrameDatabaseTest, QueryRanksByScore) {
    FrameDatabase database;
    database.Add({1}, MakeBow({1, 2, 3, 4}));
    database.Add({2}, MakeBow({1, 2, 3, 4, 5, 6}));
    database.Add({3}, MakeBow({1, 2, 3, 4}));
    const std::vector<DatabaseHit> hits = database.Query(MakeBow({1, 2, 3, 4}));
    ASSERT_EQ(hits.size(), 3u);
    // Frames 1 and 3 are identical to the query (score 1), tied: ascending id.
    EXPECT_EQ(hits[0].frame, map::FrameId{1});
    EXPECT_EQ(hits[1].frame, map::FrameId{3});
    EXPECT_EQ(hits[2].frame, map::FrameId{2});
    EXPECT_NEAR(hits[0].score, 1.0, 1e-12);
    EXPECT_LT(hits[2].score, hits[1].score);
    EXPECT_EQ(hits[0].common_words, 4u);
}

TEST(FrameDatabaseTest, CommonWordThreshold) {
    FrameDatabase database;
    database.Add({1}, MakeBow(Range(0, 10)));   // 10 shared
    database.Add({2}, MakeBow(Range(0, 7)));    // 7 shared: excluded
    database.Add({3}, MakeBow(Range(0, 8)));    // 8 shared: kept
    const std::vector<DatabaseHit> hits = database.Query(MakeBow(Range(0, 10)));
    ASSERT_EQ(hits.size(), 2u);
    for (const DatabaseHit& hit : hits) EXPECT_NE(hit.frame, map::FrameId{2});
}

TEST(FrameDatabaseTest, RemovedFrameNeverReturned) {
    FrameDatabase database;
    database.Add({1}, MakeBow({1, 2, 3}));
    database.Add({2}, MakeBow({1, 2, 3}));
    database.Remove({1});
    database.Remove({99});  // unknown: no-op
    std::vector<DatabaseHit> hits = database.Query(MakeBow({1, 2, 3}));
    ASSERT_EQ(hits.size(), 1u);
    EXPECT_EQ(hits[0].frame, map::FrameId{2});
    EXPECT_FALSE(database.Contains({1}));
    // Re-adding replaces rather than duplicates.
    database.Add({2}, MakeBow({7, 8}));
    EXPECT_TRUE(database.Query(MakeBow({1, 2, 3})).empty());
    EXPECT_EQ(database.Query(MakeBow({7, 8})).size(), 1u);
    database.Clear();
    EXPECT_EQ(database.size(), 0u);
    EXPECT_TRUE(database.Query(MakeBow({7, 8})).empty());
}

TEST(FrameDatabaseTest, EmptyQueryReturnsNothing) {
    FrameDatabase database;
    database.Add({1}, MakeBow({1, 2}));
    EXPECT_TRUE(database.Query(BowVector{}).empty());
    EXPECT_TRUE(database.Query(MakeBow({5, 6})).empty());  // no shared word
    database.Add({2}, BowVector{});                         // empty: not indexed
    EXPECT_FALSE(database.Contains({2}));
}

}  // namespace
}  // namespace voyahslam::bow
