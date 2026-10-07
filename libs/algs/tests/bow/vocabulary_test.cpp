#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/features/sift_feature_extractor.h>

#include <initializer_list>
#include <utility>
#include <vector>

// Every member must compile for both extractors the Mapper is instantiated for.
template class voyahslam::bow::Vocabulary<voyahslam::features::OrbFeatureExtractor>;
template class voyahslam::bow::Vocabulary<voyahslam::features::SIFTFeatureExtractor>;

namespace voyahslam::bow {
namespace {

using Extractor = test::TestExtractor;
using Descriptor = Extractor::TDescriptor;
using Descriptors = Extractor::TDescriptors;
using TestVocabulary = Vocabulary<Extractor>;

// A descriptor with the given bytes set to 0xFF and all others zero.
Descriptor WithBytes(std::initializer_list<int> bytes) {
    Descriptor d = Descriptor::Zero();
    for (const int b : bytes) d[b] = 0xFF;
    return d;
}

// Inner node j (j = 0..2) owns bytes [8j, 8j + 8); leaf m (m = 0..2) adds byte 24 + m.
Descriptor InnerDescriptor(int j) {
    Descriptor d = Descriptor::Zero();
    for (int b = 8 * j; b < 8 * j + 8; ++b) d[b] = 0xFF;
    return d;
}

Descriptor LeafDescriptor(int j, int m) {
    Descriptor d = InnerDescriptor(j);
    d[24 + m] = 0xFF;
    return d;
}

// k = 3, L = 2: root 0; inner nodes 1..3; leaves 4..12, three per inner node.
// Leaf node 4 + 3j + m is word 3j + m and has idf weight (word + 1).
TestVocabulary MakeVocabulary() {
    std::vector<NodeId> parents{0, 0, 0, 0};
    Descriptors descriptors = Descriptors::Zero(Extractor::DescriptorSize, 13);
    std::vector<double> weights(13, 0.0);
    for (int j = 0; j < 3; ++j) descriptors.col(1 + j) = InnerDescriptor(j);
    for (int j = 0; j < 3; ++j) {
        for (int m = 0; m < 3; ++m) {
            const int node = 4 + 3 * j + m;
            parents.push_back(static_cast<NodeId>(1 + j));
            descriptors.col(node) = LeafDescriptor(j, m);
            weights[node] = 3 * j + m + 1.0;
        }
    }
    return TestVocabulary::FromTree(3, 2, std::move(parents), std::move(descriptors), std::move(weights));
}

Descriptors Columns(const std::vector<Descriptor>& columns) {
    Descriptors descriptors(Extractor::DescriptorSize, static_cast<Eigen::Index>(columns.size()));
    for (size_t i = 0; i < columns.size(); ++i) descriptors.col(static_cast<Eigen::Index>(i)) = columns[i];
    return descriptors;
}

std::vector<WordId> Words(const BowVector& bow) {
    std::vector<WordId> words;
    for (const auto& [word, weight] : bow.entries) words.push_back(word);
    return words;
}

TEST(VocabularyTest, ReportsShape) {
    const auto vocabulary = MakeVocabulary();
    EXPECT_EQ(vocabulary.k(), 3);
    EXPECT_EQ(vocabulary.L(), 2);
    EXPECT_EQ(vocabulary.num_words(), 9U);
}

TEST(VocabularyTest, TransformPicksClosestChildPerLevel) {
    const auto vocabulary = MakeVocabulary();
    // Each query is a leaf descriptor with one stray byte, so it is closest to that leaf.
    Descriptor a = LeafDescriptor(2, 1);  // word 7
    a[30] = 0x0F;
    Descriptor b = LeafDescriptor(0, 2);  // word 2
    b[31] = 0x01;
    const Descriptor c = LeafDescriptor(1, 0);  // word 3
    BowVector bow;
    FeatureVector features;
    vocabulary.Transform(Columns({a, b, c}), 0, bow, features);
    EXPECT_EQ(Words(bow), (std::vector<WordId>{2, 3, 7}));
}

TEST(VocabularyTest, TieGoesToFirstChild) {
    const auto vocabulary = MakeVocabulary();
    // Equidistant from inner nodes 2 and 3 (64 bits each), then from all three of node 2's
    // leaves: the first child wins at both levels, giving node 7 (word 3).
    Descriptor tied = InnerDescriptor(1);
    tied += InnerDescriptor(2);
    // Equidistant from all leaves of inner node 1: leaf node 4 (word 0).
    const Descriptor leaf_tie = InnerDescriptor(0);
    BowVector bow;
    FeatureVector features;
    vocabulary.Transform(Columns({tied}), 0, bow, features);
    EXPECT_EQ(Words(bow), (std::vector<WordId>{3}));
    vocabulary.Transform(Columns({leaf_tie}), 0, bow, features);
    EXPECT_EQ(Words(bow), (std::vector<WordId>{0}));
}

TEST(VocabularyTest, BowWeightsAreIdfSumL1Normalised) {
    const auto vocabulary = MakeVocabulary();
    // Word 1 has idf 2.0 and occurs twice; word 0 has idf 1.0 and occurs once.
    const Descriptor w = LeafDescriptor(0, 1);
    const Descriptor v = LeafDescriptor(0, 0);
    BowVector bow;
    FeatureVector features;
    vocabulary.Transform(Columns({w, v, w}), 0, bow, features);
    ASSERT_EQ(bow.entries.size(), 2U);
    EXPECT_EQ(bow.entries[0].first, 0U);
    EXPECT_DOUBLE_EQ(bow.entries[0].second, 1.0 / 5.0);
    EXPECT_EQ(bow.entries[1].first, 1U);
    EXPECT_DOUBLE_EQ(bow.entries[1].second, 4.0 / 5.0);
}

TEST(VocabularyTest, FeatureVectorFilesUnderAncestor) {
    const auto vocabulary = MakeVocabulary();
    const std::vector<Descriptor> columns{LeafDescriptor(1, 2), LeafDescriptor(0, 0), LeafDescriptor(1, 0),
                                          LeafDescriptor(0, 1)};
    BowVector bow;
    FeatureVector features;
    vocabulary.Transform(Columns(columns), 1, bow, features);
    EXPECT_EQ(features.nodes, (std::vector<NodeId>{1, 2}));
    EXPECT_EQ(features.offsets, (std::vector<uint32_t>{0, 2, 4}));
    EXPECT_EQ(features.features, (std::vector<uint32_t>{1, 3, 0, 2}));

    vocabulary.Transform(Columns(columns), 0, bow, features);
    EXPECT_EQ(features.nodes, (std::vector<NodeId>{4, 5, 7, 9}));
    EXPECT_EQ(features.offsets, (std::vector<uint32_t>{0, 1, 2, 3, 4}));
    EXPECT_EQ(features.features, (std::vector<uint32_t>{1, 3, 2, 0}));
}

TEST(VocabularyTest, LevelsUpBeyondDepthFilesUnderRoot) {
    const auto vocabulary = MakeVocabulary();
    const auto descriptors = Columns({LeafDescriptor(2, 2), LeafDescriptor(0, 0)});
    for (const int levels_up : {2, 5}) {
        BowVector bow;
        FeatureVector features;
        vocabulary.Transform(descriptors, levels_up, bow, features);
        EXPECT_EQ(features.nodes, (std::vector<NodeId>{0}));
        EXPECT_EQ(features.offsets, (std::vector<uint32_t>{0, 2}));
        EXPECT_EQ(features.features, (std::vector<uint32_t>{0, 1}));
    }
}

TEST(VocabularyTest, ShallowLeafFilesUnderItself) {
    // Root 0 with a leaf 1 and an inner node 2 whose leaves are 3 and 4 (L = 2).
    // levels_up = 0 files under depth 2, which leaf 1 never reaches: it files under itself.
    Descriptors descriptors = Descriptors::Zero(Extractor::DescriptorSize, 5);
    descriptors.col(1) = WithBytes({0, 1, 2, 3});
    descriptors.col(2) = WithBytes({8, 9, 10, 11});
    descriptors.col(3) = WithBytes({8, 9, 10, 11, 20});
    descriptors.col(4) = WithBytes({8, 9, 10, 11, 21});
    const auto vocabulary = TestVocabulary::FromTree(2, 2, {0, 0, 0, 2, 2}, std::move(descriptors),
                                                     {0.0, 1.0, 0.0, 1.0, 1.0});
    EXPECT_EQ(vocabulary.num_words(), 3U);
    BowVector bow;
    FeatureVector features;
    vocabulary.Transform(Columns({WithBytes({8, 9, 10, 11, 21}), WithBytes({0, 1, 2, 3})}), 0, bow, features);
    EXPECT_EQ(Words(bow), (std::vector<WordId>{0, 2}));
    EXPECT_EQ(features.nodes, (std::vector<NodeId>{1, 4}));
    EXPECT_EQ(features.features, (std::vector<uint32_t>{1, 0}));
}

TEST(VocabularyTest, ZeroWeightWordsAreSkipped) {
    // DBoW2 adds a feature to neither vector when its word's weight is 0.
    Descriptors descriptors = Descriptors::Zero(Extractor::DescriptorSize, 3);
    descriptors.col(1) = WithBytes({0});
    descriptors.col(2) = WithBytes({8});
    const auto vocabulary = TestVocabulary::FromTree(2, 1, {0, 0, 0}, std::move(descriptors), {0.0, 0.0, 3.0});
    BowVector bow;
    FeatureVector features;
    vocabulary.Transform(Columns({WithBytes({0}), WithBytes({8})}), 0, bow, features);
    ASSERT_EQ(bow.entries.size(), 1U);
    EXPECT_EQ(bow.entries[0].first, 1U);
    EXPECT_DOUBLE_EQ(bow.entries[0].second, 1.0);
    EXPECT_EQ(features.nodes, (std::vector<NodeId>{2}));
    EXPECT_EQ(features.features, (std::vector<uint32_t>{1}));
}

TEST(VocabularyTest, EmptyDescriptorsGiveEmptyVectors) {
    const auto vocabulary = MakeVocabulary();
    BowVector bow{{{1, 1.0}}};
    FeatureVector features{{1}, {0, 1}, {0}};
    vocabulary.Transform(Descriptors(Extractor::DescriptorSize, 0), 4, bow, features);
    EXPECT_TRUE(bow.entries.empty());
    EXPECT_TRUE(features.nodes.empty());
    EXPECT_EQ(features.offsets, (std::vector<uint32_t>{0}));
    EXPECT_TRUE(features.features.empty());
}

TEST(VocabularyTest, ScoreIdenticalOneDisjointZero) {
    const BowVector a{{{1, 0.5}, {2, 0.5}}};
    const BowVector b{{{2, 0.5}, {3, 0.5}}};
    const BowVector c{{{4, 0.25}, {7, 0.75}}};
    EXPECT_DOUBLE_EQ(TestVocabulary::Score(a, a), 1.0);
    EXPECT_DOUBLE_EQ(TestVocabulary::Score(a, c), 0.0);
    EXPECT_DOUBLE_EQ(TestVocabulary::Score(a, b), 0.5);
    EXPECT_DOUBLE_EQ(TestVocabulary::Score(b, a), 0.5);
    EXPECT_DOUBLE_EQ(TestVocabulary::Score(a, BowVector{}), 0.0);
}

TEST(VocabularyTest, ScoreOfTransformedSelfIsOne) {
    const auto vocabulary = MakeVocabulary();
    BowVector bow;
    FeatureVector features;
    vocabulary.Transform(Columns({LeafDescriptor(0, 1), LeafDescriptor(2, 0), LeafDescriptor(2, 0)}), 0, bow,
                         features);
    EXPECT_NEAR(TestVocabulary::Score(bow, bow), 1.0, 1e-12);
}

}  // namespace
}  // namespace voyahslam::bow
