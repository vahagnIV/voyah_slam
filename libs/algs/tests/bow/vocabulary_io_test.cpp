#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/bow/vocabulary.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace voyahslam::bow {
namespace {

using Extractor = test::TestExtractor;
using Descriptor = Extractor::TDescriptor;
using Descriptors = Extractor::TDescriptors;
using TestVocabulary = Vocabulary<Extractor>;

std::string DataPath(const std::string& name) { return std::string(VOYAH_TEST_DATA_DIR) + "/" + name; }

std::string TempPath(const std::string& name) { return testing::TempDir() + "voyah_bow_io_" + name; }

std::string ReadFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void WriteFile(const std::string& path, const std::string& content) {
    std::ofstream out(path, std::ios::binary);
    out << content;
}

// Writes `content` to a temp file and loads it as text.
std::optional<TestVocabulary> LoadTextContent(const std::string& name, const std::string& content) {
    const std::string path = TempPath(name);
    WriteFile(path, content);
    auto vocabulary = TestVocabulary::LoadText(path);
    std::filesystem::remove(path);
    return vocabulary;
}

std::optional<TestVocabulary> LoadBinaryContent(const std::string& name, const std::string& content) {
    const std::string path = TempPath(name);
    WriteFile(path, content);
    auto vocabulary = TestVocabulary::LoadBinary(path);
    std::filesystem::remove(path);
    return vocabulary;
}

Descriptor LeafDescriptor(int j, int m) {
    Descriptor d = Descriptor::Zero();
    for (int b = 8 * j; b < 8 * j + 8; ++b) d[b] = 0xFF;
    d[24 + m] = 0xFF;
    return d;
}

// One node line of the text format: `parent is_leaf d0..d31 weight`.
std::string NodeLine(int parent, int is_leaf, int fill = 7, int count = 32, const std::string& weight = "1") {
    std::ostringstream line;
    line << parent << ' ' << is_leaf;
    for (int i = 0; i < count; ++i) line << ' ' << fill;
    line << ' ' << weight << '\n';
    return line.str();
}

const std::string kHeader = "2 1 0 0\n";

void ExpectSameTree(const TestVocabulary& a, const TestVocabulary& b) {
    EXPECT_EQ(a.k(), b.k());
    EXPECT_EQ(a.L(), b.L());
    EXPECT_EQ(a.num_words(), b.num_words());
    EXPECT_EQ(a.parents(), b.parents());
    EXPECT_EQ(a.weights(), b.weights());
    ASSERT_EQ(a.descriptors().cols(), b.descriptors().cols());
    EXPECT_TRUE(a.descriptors() == b.descriptors());
}

TEST(VocabularyIoTest, LoadsTinyText) {
    const auto loaded = TestVocabulary::LoadText(DataPath("tiny_voc.txt"));
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->k(), 3);
    EXPECT_EQ(loaded->L(), 2);
    EXPECT_EQ(loaded->num_nodes(), 13U);
    EXPECT_EQ(loaded->num_words(), 9U);

    // The Task 1 tree, built directly.
    std::vector<NodeId> parents{0, 0, 0, 0};
    Descriptors descriptors = Descriptors::Zero(Extractor::DescriptorSize, 13);
    std::vector<double> weights(13, 0.0);
    for (int j = 0; j < 3; ++j) {
        for (int b = 8 * j; b < 8 * j + 8; ++b) descriptors(b, 1 + j) = 0xFF;
        for (int m = 0; m < 3; ++m) {
            const int node = 4 + 3 * j + m;
            parents.push_back(static_cast<NodeId>(1 + j));
            descriptors.col(node) = LeafDescriptor(j, m);
            weights[node] = 3 * j + m + 1.0;
        }
    }
    const auto direct =
        TestVocabulary::FromTree(3, 2, std::move(parents), std::move(descriptors), std::move(weights));
    ExpectSameTree(*loaded, direct);

    Descriptors queries(Extractor::DescriptorSize, 4);
    queries.col(0) = LeafDescriptor(1, 2);
    queries.col(1) = LeafDescriptor(0, 0);
    queries.col(2) = LeafDescriptor(1, 0);
    queries.col(3) = LeafDescriptor(0, 1);
    for (const int levels_up : {0, 1, 2}) {
        BowVector bow_a, bow_b;
        FeatureVector fv_a, fv_b;
        loaded->Transform(queries, levels_up, bow_a, fv_a);
        direct.Transform(queries, levels_up, bow_b, fv_b);
        EXPECT_EQ(bow_a.entries, bow_b.entries);
        EXPECT_EQ(fv_a.nodes, fv_b.nodes);
        EXPECT_EQ(fv_a.offsets, fv_b.offsets);
        EXPECT_EQ(fv_a.features, fv_b.features);
        EXPECT_FALSE(bow_a.entries.empty());
    }
}

TEST(VocabularyIoTest, TrailingNewlinesCreateNoPhantomNode) {
    const auto plain = TestVocabulary::LoadText(DataPath("tiny_voc.txt"));
    const auto padded = TestVocabulary::LoadText(DataPath("tiny_voc_trailing_newlines.txt"));
    ASSERT_TRUE(plain.has_value());
    ASSERT_TRUE(padded.has_value());
    ExpectSameTree(*plain, *padded);

    // No final newline, CRLF line endings and trailing blanks are accepted as well.
    std::string content = ReadFile(DataPath("tiny_voc.txt"));
    content.pop_back();
    const auto no_newline = LoadTextContent("no_newline.txt", content);
    ASSERT_TRUE(no_newline.has_value());
    ExpectSameTree(*plain, *no_newline);
    const auto padded_blanks = LoadTextContent("blanks.txt", content + "\n  \t\n\n");
    ASSERT_TRUE(padded_blanks.has_value());
    ExpectSameTree(*plain, *padded_blanks);
    std::string crlf_content;
    for (const char c : content + "\n") crlf_content += (c == '\n') ? std::string("\r\n") : std::string(1, c);
    const auto crlf = LoadTextContent("crlf.txt", crlf_content);
    ASSERT_TRUE(crlf.has_value());
    ExpectSameTree(*plain, *crlf);
}

TEST(VocabularyIoTest, BinaryRoundTripIdentical) {
    const auto loaded = TestVocabulary::LoadText(DataPath("tiny_voc.txt"));
    ASSERT_TRUE(loaded.has_value());
    const std::string path = TempPath("roundtrip.bin");
    ASSERT_TRUE(loaded->SaveBinary(path));
    const auto reloaded = TestVocabulary::LoadBinary(path);
    ASSERT_TRUE(reloaded.has_value());
    ExpectSameTree(*loaded, *reloaded);

    // Saving the reloaded tree reproduces the same bytes.
    const std::string path2 = TempPath("roundtrip2.bin");
    ASSERT_TRUE(reloaded->SaveBinary(path2));
    EXPECT_EQ(ReadFile(path), ReadFile(path2));
    std::filesystem::remove(path);
    std::filesystem::remove(path2);

    EXPECT_FALSE(loaded->SaveBinary(TempPath("no_such_dir/x.bin")));
}

TEST(VocabularyIoTest, RejectsBadMagicChecksumTruncation) {
    const auto loaded = TestVocabulary::LoadText(DataPath("tiny_voc.txt"));
    ASSERT_TRUE(loaded.has_value());
    const std::string path = TempPath("corrupt.bin");
    ASSERT_TRUE(loaded->SaveBinary(path));
    const std::string good = ReadFile(path);
    std::filesystem::remove(path);
    ASSERT_TRUE(LoadBinaryContent("good.bin", good).has_value());

    std::string bad_magic = good;
    bad_magic[0] = 'X';
    EXPECT_FALSE(LoadBinaryContent("magic.bin", bad_magic).has_value());

    std::string bad_version = good;
    bad_version[7] = 2;
    EXPECT_FALSE(LoadBinaryContent("version.bin", bad_version).has_value());

    std::string bad_payload = good;
    bad_payload[good.size() / 2] ^= 0x01;
    EXPECT_FALSE(LoadBinaryContent("payload.bin", bad_payload).has_value());

    std::string bad_checksum = good;
    bad_checksum.back() ^= 0x01;
    EXPECT_FALSE(LoadBinaryContent("checksum.bin", bad_checksum).has_value());

    EXPECT_FALSE(LoadBinaryContent("truncated.bin", good.substr(0, good.size() - 1)).has_value());
    EXPECT_FALSE(LoadBinaryContent("truncated_header.bin", good.substr(0, 10)).has_value());
    EXPECT_FALSE(LoadBinaryContent("empty.bin", "").has_value());
    EXPECT_FALSE(LoadBinaryContent("trailing.bin", good + "x").has_value());
    EXPECT_FALSE(TestVocabulary::LoadBinary(TempPath("missing.bin")).has_value());
}

TEST(VocabularyIoTest, RejectsDescriptorSizeMismatch) {
    const auto loaded = TestVocabulary::LoadText(DataPath("tiny_voc.txt"));
    ASSERT_TRUE(loaded.has_value());
    const std::string path = TempPath("size.bin");
    ASSERT_TRUE(loaded->SaveBinary(path));
    std::string content = ReadFile(path);
    std::filesystem::remove(path);
    // descriptor_bytes sits after magic (8) + k (4) + L (4).
    content[16] = 16;
    // Fix up the checksum so only the size is wrong.
    uint64_t hash = 14695981039346656037ULL;
    for (size_t i = 0; i + 8 < content.size(); ++i) {
        hash ^= static_cast<uint8_t>(content[i]);
        hash *= 1099511628211ULL;
    }
    for (int i = 0; i < 8; ++i) content[content.size() - 8 + i] = static_cast<char>(hash >> (8 * i));
    EXPECT_FALSE(LoadBinaryContent("size_mismatch.bin", content).has_value());

    // Text: too few and too many descriptor bytes.
    EXPECT_FALSE(LoadTextContent("few.txt", kHeader + NodeLine(0, 1, 7, 31) + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("many.txt", kHeader + NodeLine(0, 1, 7, 33) + NodeLine(0, 1)).has_value());
    EXPECT_TRUE(LoadTextContent("exact.txt", kHeader + NodeLine(0, 1) + NodeLine(0, 1)).has_value());
}

TEST(VocabularyIoTest, RejectsMalformedText) {
    EXPECT_FALSE(TestVocabulary::LoadText(TempPath("missing.txt")).has_value());
    EXPECT_FALSE(LoadTextContent("empty.txt", "").has_value());
    EXPECT_FALSE(LoadTextContent("blank.txt", "\n\n").has_value());
    EXPECT_FALSE(LoadTextContent("header_only.txt", kHeader).has_value());  // root without children
    EXPECT_FALSE(LoadTextContent("short_header.txt", "2 1 0\n" + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("long_header.txt", "2 1 0 0 5\n" + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("bad_header.txt", "two 1 0 0\n" + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("zero_k.txt", "0 1 0 0\n" + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("zero_L.txt", "2 0 0 0\n" + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("garbage_byte.txt", kHeader + NodeLine(0, 1).replace(4, 1, "x")).has_value());
    EXPECT_FALSE(LoadTextContent("byte_256.txt", kHeader + NodeLine(0, 1, 256)).has_value());
    EXPECT_FALSE(LoadTextContent("negative_byte.txt", kHeader + NodeLine(0, 1, -1)).has_value());
    EXPECT_FALSE(LoadTextContent("bad_weight.txt", kHeader + NodeLine(0, 1, 7, 32, "1.5abc")).has_value());
    EXPECT_FALSE(LoadTextContent("nan_weight.txt", kHeader + NodeLine(0, 1, 7, 32, "nan")).has_value());
    EXPECT_FALSE(LoadTextContent("no_weight.txt", kHeader + NodeLine(0, 1, 7, 31) + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("extra_token.txt", kHeader + NodeLine(0, 1, 7, 32, "1 2")).has_value());
    EXPECT_FALSE(LoadTextContent("bad_leaf_flag.txt", kHeader + NodeLine(0, 2)).has_value());
    EXPECT_FALSE(LoadTextContent("negative_parent.txt", kHeader + NodeLine(-1, 1)).has_value());
    // A blank line in the middle is malformed, not skipped.
    EXPECT_FALSE(LoadTextContent("middle_blank.txt", kHeader + NodeLine(0, 1) + "\n" + NodeLine(0, 1)).has_value());
}

TEST(VocabularyIoTest, RejectsBrokenTree) {
    // parent >= id (node 1 names itself, then a forward reference).
    EXPECT_FALSE(LoadTextContent("self_parent.txt", kHeader + NodeLine(1, 1) + NodeLine(0, 1)).has_value());
    EXPECT_FALSE(LoadTextContent("forward_parent.txt", kHeader + NodeLine(2, 1) + NodeLine(0, 1)).has_value());
    // is_leaf = 1 on a node that has children.
    EXPECT_FALSE(
        LoadTextContent("leaf_with_child.txt", kHeader + NodeLine(0, 1) + NodeLine(1, 1)).has_value());
    // is_leaf = 0 on a childless node.
    EXPECT_FALSE(LoadTextContent("inner_without_child.txt", kHeader + NodeLine(0, 0) + NodeLine(0, 1)).has_value());
    // The consistent versions load.
    EXPECT_TRUE(LoadTextContent("consistent.txt", kHeader + NodeLine(0, 0) + NodeLine(1, 1)).has_value());

    // Binary: a valid checksum over an invalid tree (parent[1] = 5 with 3 nodes).
    const auto tiny = LoadTextContent("tiny3.txt", kHeader + NodeLine(0, 0) + NodeLine(1, 1));
    ASSERT_TRUE(tiny.has_value());
    const std::string path = TempPath("broken.bin");
    ASSERT_TRUE(tiny->SaveBinary(path));
    std::string content = ReadFile(path);
    std::filesystem::remove(path);
    const size_t parents_offset = 8 + 4 + 4 + 4 + 8;
    content[parents_offset + 4] = 5;
    uint64_t hash = 14695981039346656037ULL;
    for (size_t i = 0; i + 8 < content.size(); ++i) {
        hash ^= static_cast<uint8_t>(content[i]);
        hash *= 1099511628211ULL;
    }
    for (int i = 0; i < 8; ++i) content[content.size() - 8 + i] = static_cast<char>(hash >> (8 * i));
    EXPECT_FALSE(LoadBinaryContent("broken_tree.bin", content).has_value());
}

TEST(VocabularyIoTest, RejectsUnsupportedScoringOrWeighting) {
    const std::string node = NodeLine(0, 1) + NodeLine(0, 1);
    EXPECT_TRUE(LoadTextContent("ok.txt", "2 1 0 0\n" + node).has_value());
    EXPECT_FALSE(LoadTextContent("weighting_idf.txt", "2 1 0 1\n" + node).has_value());
    EXPECT_FALSE(LoadTextContent("weighting_tf.txt", "2 1 0 2\n" + node).has_value());
    EXPECT_FALSE(LoadTextContent("weighting_binary.txt", "2 1 0 3\n" + node).has_value());
    EXPECT_FALSE(LoadTextContent("scoring_l2.txt", "2 1 1 0\n" + node).has_value());
    EXPECT_FALSE(LoadTextContent("scoring_bhattacharyya.txt", "2 1 4 0\n" + node).has_value());
    EXPECT_FALSE(LoadTextContent("scoring_range.txt", "2 1 9 0\n" + node).has_value());
}

}  // namespace
}  // namespace voyahslam::bow
