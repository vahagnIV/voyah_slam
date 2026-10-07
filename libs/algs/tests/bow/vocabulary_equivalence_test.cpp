// Equivalence with the original DBoW2 (ORB-SLAM3 Thirdparty/DBoW2) on the real ORBvoc.txt.
// The fixture holds our extractor's descriptors of 3 corridor1 frames and DBoW2's
// transform(desc, bow, feat, 4) of them; see tools/dbow2_reference/README.md.
#include <gtest/gtest.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/features/orb_feature_extractor.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace voyahslam::bow {
namespace {

using OrbVocabulary = Vocabulary<features::OrbFeatureExtractor>;
using Descriptors = features::OrbFeatureExtractor::TDescriptors;

// One frame of the fixture: descriptors and DBoW2's BowVector and FeatureVector.
struct ReferenceFrame {
    Descriptors descriptors;
    std::vector<std::pair<WordId, double>> bow;
    std::vector<std::pair<NodeId, std::vector<uint32_t>>> features;
};

// Little-endian reader over the fixture file.
class FixtureReader {
public:
    explicit FixtureReader(const std::string& path) : in_(path, std::ios::binary) {}

    [[nodiscard]] bool ok() const { return static_cast<bool>(in_); }
    [[nodiscard]] bool AtEnd() { return in_.peek() == std::char_traits<char>::eof(); }

    uint32_t U32() {
        unsigned char b[4] = {};
        in_.read(reinterpret_cast<char*>(b), 4);
        return uint32_t{b[0]} | uint32_t{b[1]} << 8 | uint32_t{b[2]} << 16 | uint32_t{b[3]} << 24;
    }

    double F64() {
        unsigned char b[8] = {};
        in_.read(reinterpret_cast<char*>(b), 8);
        uint64_t bits = 0;
        for (int i = 7; i >= 0; --i) bits = bits << 8 | b[i];
        double value = 0.0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    void Bytes(uint8_t* out_data, size_t size) {
        in_.read(reinterpret_cast<char*>(out_data), static_cast<std::streamsize>(size));
    }

private:
    std::ifstream in_;
};

// Format: u32 num_frames | per frame: u32 n | u8 desc[n][32] | u32 bow_count | (u32 word, f64 weight)[bow_count]
//                                    | u32 node_count | per node: u32 node | u32 m | u32 features[m]
std::vector<ReferenceFrame> LoadFixture(const std::string& path) {
    FixtureReader reader(path);
    std::vector<ReferenceFrame> frames(reader.U32());
    for (auto& frame : frames) {
        const uint32_t n = reader.U32();
        frame.descriptors.resize(Descriptors::RowsAtCompileTime, n);
        reader.Bytes(frame.descriptors.data(), static_cast<size_t>(frame.descriptors.size()));
        frame.bow.resize(reader.U32());
        for (auto& [word, weight] : frame.bow) {
            word = reader.U32();
            weight = reader.F64();
        }
        frame.features.resize(reader.U32());
        for (auto& [node, indices] : frame.features) {
            node = reader.U32();
            indices.resize(reader.U32());
            for (auto& index : indices) index = reader.U32();
        }
    }
    EXPECT_TRUE(reader.ok()) << "truncated fixture " << path;
    EXPECT_TRUE(reader.AtEnd()) << "trailing bytes in fixture " << path;
    return frames;
}

TEST(VocabularyEquivalenceTest, MatchesDbow2OnCorridorFrames) {
    const char* vocabulary_path = std::getenv("VOYAHSLAM_ORBVOC");
    if (vocabulary_path == nullptr || *vocabulary_path == '\0') {
        GTEST_SKIP() << "set VOYAHSLAM_ORBVOC to the path of ORB-SLAM3's ORBvoc.txt to run this test";
    }

    const auto start = std::chrono::steady_clock::now();
    const auto vocabulary = OrbVocabulary::LoadText(vocabulary_path);
    const double load_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    ASSERT_TRUE(vocabulary.has_value()) << "cannot load " << vocabulary_path;
    RecordProperty("load_text_seconds", std::to_string(load_seconds));

    const auto frames = LoadFixture(std::string(VOYAH_TEST_DATA_DIR) + "/dbow2_reference.bin");
    ASSERT_EQ(frames.size(), 3u);

    double max_weight_difference = 0.0;
    for (size_t f = 0; f < frames.size(); ++f) {
        SCOPED_TRACE("frame " + std::to_string(f));
        const ReferenceFrame& expected = frames[f];
        BowVector bow;
        FeatureVector features;
        vocabulary->Transform(expected.descriptors, 4, bow, features);

        // Same words, in the same order, with weights within 1e-12.
        ASSERT_EQ(bow.entries.size(), expected.bow.size());
        for (size_t i = 0; i < bow.entries.size(); ++i) {
            ASSERT_EQ(bow.entries[i].first, expected.bow[i].first) << "entry " << i;
            const double difference = std::abs(bow.entries[i].second - expected.bow[i].second);
            max_weight_difference = std::max(max_weight_difference, difference);
            EXPECT_LE(difference, 1e-12) << "word " << expected.bow[i].first;
        }

        // Same nodes, each with the same feature indices.
        ASSERT_EQ(features.nodes.size(), expected.features.size());
        for (size_t i = 0; i < features.nodes.size(); ++i) {
            ASSERT_EQ(features.nodes[i], expected.features[i].first) << "node entry " << i;
            const auto filed = features.FeaturesOf(i);
            EXPECT_EQ(std::vector<uint32_t>(filed.begin(), filed.end()), expected.features[i].second)
                << "node " << features.nodes[i];
        }
    }
    char difference_text[32];
    std::snprintf(difference_text, sizeof(difference_text), "%.3g", max_weight_difference);
    RecordProperty("max_weight_difference", difference_text);
}

}  // namespace
}  // namespace voyahslam::bow
