// Vocabulary file IO: the DBoW2 / ORB-SLAM text loader and our checksummed binary format.
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/logging/logger.h>

#include <bit>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string_view>
#include <type_traits>

namespace voyahslam::bow {
namespace {

static_assert(std::endian::native == std::endian::little, "the binary vocabulary format is little-endian");

constexpr char kMagic[8] = {'V', 'O', 'Y', 'B', 'O', 'W', '\0', '\1'};
constexpr size_t kHeaderSize = sizeof(kMagic) + 4 + 4 + 4 + 8;
constexpr size_t kChecksumSize = 8;

// DBoW2 enum numbering (ORB-SLAM3 TemplatedVocabulary.h): L1_NORM = 0, TF_IDF = 0.
constexpr int kScoringL1Norm = 0;
constexpr int kWeightingTfIdf = 0;

const std::shared_ptr<spdlog::logger>& Log() {
    static const std::shared_ptr<spdlog::logger> log = Logger::CreateLogger("Vocabulary");
    return log;
}

uint64_t Fnv1a64(const uint8_t* data, size_t size) {
    uint64_t hash = 14695981039346656037ULL;
    for (size_t i = 0; i < size; ++i) {
        hash ^= data[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

template <typename T>
void Append(std::vector<uint8_t>& out, const T& value) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&value);
    out.insert(out.end(), bytes, bytes + sizeof(T));
}

template <typename T>
T ReadAt(const std::vector<uint8_t>& in, size_t offset) {
    T value;
    std::memcpy(&value, in.data() + offset, sizeof(T));
    return value;
}

// A weight (idf) must be finite; DBoW2 itself never produces anything else.
bool ValidWeights(const std::vector<double>& weights) {
    for (const double w : weights) {
        if (!std::isfinite(w)) return false;
    }
    return true;
}

bool IsBlank(std::string_view line) {
    return line.find_first_not_of(" \t\r") == std::string_view::npos;
}

// Splits `line` into whitespace-separated tokens.
void Tokenize(std::string_view line, std::vector<std::string_view>& out_tokens) {
    out_tokens.clear();
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) ++i;
        const size_t begin = i;
        while (i < line.size() && line[i] != ' ' && line[i] != '\t' && line[i] != '\r') ++i;
        if (i > begin) out_tokens.push_back(line.substr(begin, i - begin));
    }
}

bool ParseInt(std::string_view token, long long& out_value) {
    const auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), out_value);
    return error == std::errc() && end == token.data() + token.size();
}

bool ParseDouble(std::string_view token, double& out_value) {
    const std::string copy(token);  // strtod needs a terminated string
    char* end = nullptr;
    out_value = std::strtod(copy.c_str(), &end);
    return end == copy.c_str() + copy.size() && std::isfinite(out_value);
}

}  // namespace

template <typename TFeatureExtractor>
std::optional<Vocabulary<TFeatureExtractor>> Vocabulary<TFeatureExtractor>::LoadText(const std::string& path) {
    using Scalar = typename TFeatureExtractor::TDescriptorScalarType;
    if constexpr (!std::is_same_v<Scalar, uint8_t>) {
        Log()->error("Vocabulary text '{}': unsupported descriptor type (the text format holds byte descriptors)",
                     path);
        return std::nullopt;
    } else {
        constexpr int kDescriptorSize = TFeatureExtractor::DescriptorSize;
        std::ifstream file(path);
        if (!file) {
            Log()->error("Vocabulary text '{}': cannot open file", path);
            return std::nullopt;
        }

        std::string line;
        std::vector<std::string_view> tokens;
        std::getline(file, line);
        Tokenize(line, tokens);
        long long header[4] = {0, 0, 0, 0};
        if (tokens.size() != 4 || !ParseInt(tokens[0], header[0]) || !ParseInt(tokens[1], header[1]) ||
            !ParseInt(tokens[2], header[2]) || !ParseInt(tokens[3], header[3])) {
            Log()->error("Vocabulary text '{}': header must be 'k L scoring weighting'", path);
            return std::nullopt;
        }
        // The same sanity bounds as DBoW2, but k and L must be positive.
        if (header[0] < 1 || header[0] > 20 || header[1] < 1 || header[1] > 10) {
            Log()->error("Vocabulary text '{}': k={} L={} out of range (k 1..20, L 1..10)", path, header[0],
                         header[1]);
            return std::nullopt;
        }
        if (header[2] != kScoringL1Norm || header[3] != kWeightingTfIdf) {
            Log()->error("Vocabulary text '{}': unsupported scoring {} or weighting {} (only L1 / TF-IDF)", path,
                         header[2], header[3]);
            return std::nullopt;
        }

        // Node lines; node 0 is the root and has no line. Blank lines may only trail the file.
        std::vector<NodeId> parents(1, 0);
        std::vector<uint8_t> is_leaf(1, 0);
        std::vector<double> weights(1, 0.0);
        std::vector<uint8_t> bytes(kDescriptorSize, 0);  // column 0 (the root) is zero
        size_t line_number = 1;
        size_t pending_blank_line = 0;
        while (std::getline(file, line)) {
            ++line_number;
            if (IsBlank(line)) {
                if (pending_blank_line == 0) pending_blank_line = line_number;
                continue;
            }
            if (pending_blank_line != 0) {
                Log()->error("Vocabulary text '{}': blank line {} inside the node list", path, pending_blank_line);
                return std::nullopt;
            }
            Tokenize(line, tokens);
            if (tokens.size() != 2 + static_cast<size_t>(kDescriptorSize) + 1) {
                Log()->error("Vocabulary text '{}' line {}: expected {} tokens (parent, is_leaf, {} descriptor "
                             "bytes, weight), found {}",
                             path, line_number, 2 + kDescriptorSize + 1, kDescriptorSize, tokens.size());
                return std::nullopt;
            }
            const size_t id = parents.size();
            long long parent = 0;
            long long leaf = 0;
            if (!ParseInt(tokens[0], parent) || !ParseInt(tokens[1], leaf)) {
                Log()->error("Vocabulary text '{}' line {}: parent or is_leaf is not an integer", path,
                             line_number);
                return std::nullopt;
            }
            if (parent < 0 || static_cast<size_t>(parent) >= id) {
                Log()->error("Vocabulary text '{}' line {}: parent {} must be smaller than node id {}", path,
                             line_number, parent, id);
                return std::nullopt;
            }
            if (leaf != 0 && leaf != 1) {
                Log()->error("Vocabulary text '{}' line {}: is_leaf must be 0 or 1, found {}", path, line_number,
                             leaf);
                return std::nullopt;
            }
            for (int b = 0; b < kDescriptorSize; ++b) {
                long long value = 0;
                if (!ParseInt(tokens[2 + b], value) || value < 0 || value > 255) {
                    Log()->error("Vocabulary text '{}' line {}: descriptor byte {} is not an integer in 0..255",
                                 path, line_number, b);
                    return std::nullopt;
                }
                bytes.push_back(static_cast<uint8_t>(value));
            }
            double weight = 0.0;
            if (!ParseDouble(tokens.back(), weight)) {
                Log()->error("Vocabulary text '{}' line {}: weight is not a finite number", path, line_number);
                return std::nullopt;
            }
            parents.push_back(static_cast<NodeId>(parent));
            is_leaf.push_back(static_cast<uint8_t>(leaf));
            weights.push_back(weight);
        }

        const size_t num_nodes = parents.size();
        if (num_nodes < 2) {
            Log()->error("Vocabulary text '{}': no nodes (the root has no children)", path);
            return std::nullopt;
        }
        // DBoW2 numbers words by the is_leaf flag, so it must agree with childlessness.
        std::vector<uint8_t> has_children(num_nodes, 0);
        for (size_t node = 1; node < num_nodes; ++node) has_children[parents[node]] = 1;
        for (size_t node = 1; node < num_nodes; ++node) {
            if ((is_leaf[node] == 1) == (has_children[node] == 1)) {
                Log()->error("Vocabulary text '{}': node {} has is_leaf={} but {} children", path, node,
                             static_cast<int>(is_leaf[node]), has_children[node] ? "has" : "no");
                return std::nullopt;
            }
        }

        typename TFeatureExtractor::TDescriptors descriptors(kDescriptorSize, static_cast<Eigen::Index>(num_nodes));
        std::memcpy(descriptors.data(), bytes.data(), bytes.size());
        return FromTree(static_cast<int>(header[0]), static_cast<int>(header[1]), std::move(parents),
                        std::move(descriptors), std::move(weights));
    }
}

template <typename TFeatureExtractor>
bool Vocabulary<TFeatureExtractor>::SaveBinary(const std::string& path) const {
    using Scalar = typename TFeatureExtractor::TDescriptorScalarType;
    constexpr uint32_t kDescriptorBytes = TFeatureExtractor::DescriptorSize * sizeof(Scalar);
    const size_t num_nodes = parents_.size();

    std::vector<uint8_t> out;
    out.reserve(kHeaderSize + num_nodes * (4 + kDescriptorBytes + 8) + kChecksumSize);
    out.insert(out.end(), kMagic, kMagic + sizeof(kMagic));
    Append(out, static_cast<uint32_t>(k_));
    Append(out, static_cast<uint32_t>(L_));
    Append(out, kDescriptorBytes);
    Append(out, static_cast<uint64_t>(num_nodes));
    for (const NodeId parent : parents_) Append(out, static_cast<uint32_t>(parent));
    // One column per node, contiguous in column-major storage.
    const auto* descriptor_bytes = reinterpret_cast<const uint8_t*>(descriptors_.data());
    out.insert(out.end(), descriptor_bytes, descriptor_bytes + num_nodes * kDescriptorBytes);
    for (const double weight : weights_) Append(out, weight);
    Append(out, Fnv1a64(out.data(), out.size()));

    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    file.flush();
    if (!file) {
        Log()->error("Vocabulary binary '{}': cannot write file", path);
        return false;
    }
    return true;
}

template <typename TFeatureExtractor>
std::optional<Vocabulary<TFeatureExtractor>> Vocabulary<TFeatureExtractor>::LoadBinary(const std::string& path) {
    using Scalar = typename TFeatureExtractor::TDescriptorScalarType;
    constexpr uint32_t kDescriptorBytes = TFeatureExtractor::DescriptorSize * sizeof(Scalar);

    std::ifstream file(path, std::ios::binary);
    if (!file) {
        Log()->error("Vocabulary binary '{}': cannot open file", path);
        return std::nullopt;
    }
    const std::vector<uint8_t> in{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};

    if (in.size() < kHeaderSize + kChecksumSize) {
        Log()->error("Vocabulary binary '{}': truncated ({} bytes)", path, in.size());
        return std::nullopt;
    }
    if (std::memcmp(in.data(), kMagic, sizeof(kMagic)) != 0) {
        Log()->error("Vocabulary binary '{}': wrong magic or version", path);
        return std::nullopt;
    }
    const auto k = ReadAt<uint32_t>(in, 8);
    const auto L = ReadAt<uint32_t>(in, 12);
    const auto descriptor_bytes = ReadAt<uint32_t>(in, 16);
    const auto num_nodes = ReadAt<uint64_t>(in, 20);
    if (descriptor_bytes != kDescriptorBytes) {
        Log()->error("Vocabulary binary '{}': descriptor size {} does not match the extractor's {}", path,
                     descriptor_bytes, kDescriptorBytes);
        return std::nullopt;
    }
    // Bound the node count by the file size before any size arithmetic can overflow.
    const size_t per_node = 4 + kDescriptorBytes + 8;
    if (num_nodes > (in.size() - kHeaderSize - kChecksumSize) / per_node ||
        in.size() != kHeaderSize + num_nodes * per_node + kChecksumSize) {
        Log()->error("Vocabulary binary '{}': truncated or oversized for {} nodes ({} bytes)", path, num_nodes,
                     in.size());
        return std::nullopt;
    }
    const size_t checksummed = in.size() - kChecksumSize;
    if (ReadAt<uint64_t>(in, checksummed) != Fnv1a64(in.data(), checksummed)) {
        Log()->error("Vocabulary binary '{}': checksum mismatch", path);
        return std::nullopt;
    }
    if (k == 0 || L == 0 || k > 0x7FFFFFFFU || L > 0x7FFFFFFFU || num_nodes < 2 || num_nodes > 0xFFFFFFFFULL) {
        Log()->error("Vocabulary binary '{}': invalid k={} L={} nodes={}", path, k, L, num_nodes);
        return std::nullopt;
    }

    const size_t n = num_nodes;
    std::vector<NodeId> parents(n);
    std::memcpy(parents.data(), in.data() + kHeaderSize, n * 4);
    typename TFeatureExtractor::TDescriptors descriptors(TFeatureExtractor::DescriptorSize,
                                                         static_cast<Eigen::Index>(n));
    std::memcpy(descriptors.data(), in.data() + kHeaderSize + n * 4, n * kDescriptorBytes);
    std::vector<double> weights(n);
    std::memcpy(weights.data(), in.data() + kHeaderSize + n * (4 + kDescriptorBytes), n * 8);

    if (parents[0] != 0) {
        Log()->error("Vocabulary binary '{}': the root's parent must be 0", path);
        return std::nullopt;
    }
    for (size_t node = 1; node < n; ++node) {
        if (parents[node] >= node) {
            Log()->error("Vocabulary binary '{}': node {} has parent {}, which is not smaller", path, node,
                         parents[node]);
            return std::nullopt;
        }
    }
    if (!ValidWeights(weights)) {
        Log()->error("Vocabulary binary '{}': a weight is not finite", path);
        return std::nullopt;
    }
    return FromTree(static_cast<int>(k), static_cast<int>(L), std::move(parents), std::move(descriptors),
                    std::move(weights));
}

template class Vocabulary<features::OrbFeatureExtractor>;

}  // namespace voyahslam::bow
