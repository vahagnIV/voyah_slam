#ifndef VOYAHSLAM_BOW_BOW_VECTOR_H
#define VOYAHSLAM_BOW_BOW_VECTOR_H

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace voyahslam::bow {

/// A leaf of the vocabulary tree, numbered in ascending node-id order of the leaves.
using WordId = uint32_t;
/// A vocabulary tree node; node 0 is the root.
using NodeId = uint32_t;

/// Weighted words of one image (DBoW2 BowVector): sorted by word, L1-normalised.
struct BowVector {
    std::vector<std::pair<WordId, double>> entries;
};

/// Feature indices of one image grouped by vocabulary node (DBoW2 FeatureVector), in CSR form.
/// `nodes` is sorted; the features of `nodes[i]` are `features[offsets[i] .. offsets[i + 1])`,
/// in ascending order. `offsets` has `nodes.size() + 1` entries.
struct FeatureVector {
    std::vector<NodeId> nodes;
    std::vector<uint32_t> offsets;
    std::vector<uint32_t> features;

    /// The feature indices filed under `nodes[i]`.
    [[nodiscard]] std::span<const uint32_t> FeaturesOf(size_t i) const {
        assert(i + 1 < offsets.size());
        return {features.data() + offsets[i], features.data() + offsets[i + 1]};
    }
};

/// DBoW2 L1 similarity of two L1-normalised vectors: 1 when identical, 0 when disjoint.
inline double Score(const BowVector& a, const BowVector& b) {
    // DBoW2 L1Scoring: over shared words, accumulate |x - y| - |x| - |y|, then negate and halve.
    double score = 0.0;
    auto ia = a.entries.begin();
    auto ib = b.entries.begin();
    while (ia != a.entries.end() && ib != b.entries.end()) {
        if (ia->first == ib->first) {
            score += std::abs(ia->second - ib->second) - std::abs(ia->second) - std::abs(ib->second);
            ++ia;
            ++ib;
        } else if (ia->first < ib->first) {
            ++ia;
        } else {
            ++ib;
        }
    }
    return -score / 2.0;
}

}  // namespace voyahslam::bow

#endif  // VOYAHSLAM_BOW_BOW_VECTOR_H
