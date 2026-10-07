#ifndef VOYAHSLAM_BOW_VOCABULARY_H
#define VOYAHSLAM_BOW_VOCABULARY_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include <voyahslam/bow/bow_vector.h>

namespace voyahslam::bow {

/// A DBoW2 vocabulary tree (TF-IDF weighting, L1 scoring) over the descriptors of
/// `TFeatureExtractor`, stored as flat arrays in node-id order. Immutable once built.
///
/// Port of DBoW2's `TemplatedVocabulary` (ORB-SLAM3 `Thirdparty/DBoW2`) without OpenCV:
/// descriptors come from `TFeatureExtractor::TDescriptors` (one per column) and distances
/// from `TFeatureExtractor::Distance`.
template <typename TFeatureExtractor>
class Vocabulary {
public:
    using TDescriptor = typename TFeatureExtractor::TDescriptor;
    using TDescriptors = typename TFeatureExtractor::TDescriptors;

    /// Builds the vocabulary from its nodes in id order. `parents[0]` is ignored (node 0 is
    /// the root) and every other parent id is smaller than its node's id; descriptor column 0
    /// and `weights[0]` are unused. Children and word ids are derived from the parents.
    /// Preconditions (asserted; loaders validate first): `k`, `L` > 0, the three inputs have
    /// one entry per node, and the root has children.
    static Vocabulary FromTree(int k, int L, std::vector<NodeId> parents, TDescriptors descriptors,
                               std::vector<double> weights);

    /// Converts descriptors to a BowVector (each occurrence of a word adds its idf weight,
    /// then L1 normalisation) and a FeatureVector that files feature i under the node its
    /// descent passes at depth `L - levels_up` (the root when `levels_up >= L`, the leaf
    /// itself when the leaf is shallower). Words of weight 0 are skipped, as in DBoW2.
    void Transform(const TDescriptors& descriptors, int levels_up, BowVector& out_bow,
                   FeatureVector& out_features) const;

    /// DBoW2 L1 similarity of two L1-normalised vectors: 1 when identical, 0 when disjoint.
    [[nodiscard]] static double Score(const BowVector& a, const BowVector& b);

    [[nodiscard]] size_t num_words() const { return num_words_; }
    [[nodiscard]] int k() const { return k_; }
    [[nodiscard]] int L() const { return L_; }

private:
    static constexpr WordId kNoWord = std::numeric_limits<WordId>::max();

    Vocabulary() = default;

    [[nodiscard]] bool IsLeaf(NodeId node) const { return child_offsets_[node] == child_offsets_[node + 1]; }
    /// The child of `node` closest to `descriptor`; ties go to the first child in stored order.
    [[nodiscard]] NodeId ClosestChild(NodeId node, const TDescriptor& descriptor) const;

    int k_ = 0;
    int L_ = 0;
    size_t num_words_ = 0;
    std::vector<NodeId> parents_;
    TDescriptors descriptors_;
    std::vector<double> weights_;
    std::vector<uint32_t> child_offsets_;  // children of node n: children_[child_offsets_[n] .. [n + 1])
    std::vector<NodeId> children_;         // in ascending id order
    std::vector<WordId> word_ids_;         // kNoWord for inner nodes and the root
};

template <typename TFeatureExtractor>
Vocabulary<TFeatureExtractor> Vocabulary<TFeatureExtractor>::FromTree(int k, int L, std::vector<NodeId> parents,
                                                                      TDescriptors descriptors,
                                                                      std::vector<double> weights) {
    const size_t num_nodes = parents.size();
    assert(k > 0 && L > 0);
    assert(num_nodes >= 2);
    assert(static_cast<size_t>(descriptors.cols()) == num_nodes && weights.size() == num_nodes);

    Vocabulary vocabulary;
    vocabulary.k_ = k;
    vocabulary.L_ = L;
    parents[0] = 0;

    // Children in CSR form; filling in node-id order keeps each child list ascending.
    vocabulary.child_offsets_.assign(num_nodes + 1, 0);
    for (size_t node = 1; node < num_nodes; ++node) {
        assert(parents[node] < node);
        ++vocabulary.child_offsets_[parents[node] + 1];
    }
    for (size_t node = 0; node < num_nodes; ++node) {
        vocabulary.child_offsets_[node + 1] += vocabulary.child_offsets_[node];
    }
    vocabulary.children_.resize(num_nodes - 1);
    std::vector<uint32_t> next(vocabulary.child_offsets_.begin(), vocabulary.child_offsets_.end() - 1);
    for (size_t node = 1; node < num_nodes; ++node) {
        vocabulary.children_[next[parents[node]]++] = static_cast<NodeId>(node);
    }
    assert(!vocabulary.IsLeaf(0));

    // Words are the leaves in node-id order (DBoW2 createWords).
    vocabulary.word_ids_.assign(num_nodes, kNoWord);
    for (size_t node = 1; node < num_nodes; ++node) {
        if (vocabulary.IsLeaf(static_cast<NodeId>(node))) {
            vocabulary.word_ids_[node] = static_cast<WordId>(vocabulary.num_words_++);
        }
    }

    vocabulary.parents_ = std::move(parents);
    vocabulary.descriptors_ = std::move(descriptors);
    vocabulary.weights_ = std::move(weights);
    return vocabulary;
}

template <typename TFeatureExtractor>
NodeId Vocabulary<TFeatureExtractor>::ClosestChild(NodeId node, const TDescriptor& descriptor) const {
    const uint32_t begin = child_offsets_[node];
    const uint32_t end = child_offsets_[node + 1];
    NodeId best = children_[begin];
    auto best_distance = TFeatureExtractor::Distance(descriptor, descriptors_.col(best));
    for (uint32_t c = begin + 1; c < end; ++c) {
        const NodeId child = children_[c];
        const auto distance = TFeatureExtractor::Distance(descriptor, descriptors_.col(child));
        if (distance < best_distance) {  // strict: ties keep the earlier child
            best_distance = distance;
            best = child;
        }
    }
    return best;
}

template <typename TFeatureExtractor>
void Vocabulary<TFeatureExtractor>::Transform(const TDescriptors& descriptors, int levels_up, BowVector& out_bow,
                                              FeatureVector& out_features) const {
    assert(levels_up >= 0);
    out_bow.entries.clear();
    out_features.nodes.clear();
    out_features.offsets.assign(1, 0);
    out_features.features.clear();

    const int filing_depth = L_ - levels_up;  // <= 0: everything files under the root
    const auto num_features = static_cast<uint32_t>(descriptors.cols());
    std::vector<std::pair<NodeId, uint32_t>> leaves;  // (leaf, feature)
    std::vector<std::pair<NodeId, uint32_t>> filed;   // (filing node, feature)
    leaves.reserve(num_features);
    filed.reserve(num_features);
    for (uint32_t feature = 0; feature < num_features; ++feature) {
        const TDescriptor descriptor = descriptors.col(feature);
        NodeId node = 0;
        NodeId filing_node = 0;
        for (int depth = 1; !IsLeaf(node); ++depth) {
            node = ClosestChild(node, descriptor);
            if (depth <= filing_depth) filing_node = node;
        }
        if (weights_[node] > 0) {
            leaves.emplace_back(node, feature);
            filed.emplace_back(filing_node, feature);
        }
    }

    // Word ids ascend with leaf ids, so sorting by leaf sorts by word. Each word's weights
    // are summed in feature order, then the vector is L1-normalised, as in DBoW2.
    std::sort(leaves.begin(), leaves.end());
    for (size_t i = 0; i < leaves.size();) {
        const NodeId leaf = leaves[i].first;
        double weight = 0.0;
        for (; i < leaves.size() && leaves[i].first == leaf; ++i) weight += weights_[leaf];
        out_bow.entries.emplace_back(word_ids_[leaf], weight);
    }
    double norm = 0.0;
    for (const auto& entry : out_bow.entries) norm += std::abs(entry.second);
    if (norm > 0.0) {
        for (auto& entry : out_bow.entries) entry.second /= norm;
    }

    std::sort(filed.begin(), filed.end());
    for (const auto& [node, feature] : filed) {
        if (out_features.nodes.empty() || out_features.nodes.back() != node) {
            if (!out_features.nodes.empty()) {
                out_features.offsets.push_back(static_cast<uint32_t>(out_features.features.size()));
            }
            out_features.nodes.push_back(node);
        }
        out_features.features.push_back(feature);
    }
    if (!out_features.nodes.empty()) {
        out_features.offsets.push_back(static_cast<uint32_t>(out_features.features.size()));
    }
}

template <typename TFeatureExtractor>
double Vocabulary<TFeatureExtractor>::Score(const BowVector& a, const BowVector& b) {
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

#endif  // VOYAHSLAM_BOW_VOCABULARY_H
