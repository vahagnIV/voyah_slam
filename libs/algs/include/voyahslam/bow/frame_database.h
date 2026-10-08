#ifndef VOYAHSLAM_BOW_FRAME_DATABASE_H
#define VOYAHSLAM_BOW_FRAME_DATABASE_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>
#include <voyahslam/bow/bow_vector.h>
#include <voyahslam/map/frame_id.h>

namespace voyahslam::bow {

/// A Frame found by FrameDatabase::Query.
struct DatabaseHit {
    map::FrameId frame;
    uint32_t common_words;
    double score;  // bow::Score against the query
};

/// Inverted index from word to the Frames containing it, for place recognition. It keeps each
/// Frame's BowVector, so it can score the Frame and edit exactly its word lists on removal.
/// Single-threaded; each owner (the Tracker's replica) has its own.
class FrameDatabase {
public:
    /// Indexes `frame`, replacing an earlier entry of the same id. An empty `bow` is not indexed.
    void Add(map::FrameId frame, const BowVector& bow) {
        Remove(frame);
        if (bow.entries.empty()) return;
        for (const auto& [word, weight] : bow.entries) words_[word].push_back(frame);
        frames_.emplace(frame, bow);
    }

    /// Drops `frame`; a no-op if unknown.
    void Remove(map::FrameId frame) {
        const auto it = frames_.find(frame);
        if (it == frames_.end()) return;
        for (const auto& [word, weight] : it->second.entries) {
            const auto list = words_.find(word);
            if (list == words_.end()) continue;
            std::erase(list->second, frame);
            if (list->second.empty()) words_.erase(list);
        }
        frames_.erase(it);
    }

    void Clear() {
        words_.clear();
        frames_.clear();
    }

    [[nodiscard]] size_t size() const { return frames_.size(); }
    [[nodiscard]] bool Contains(map::FrameId frame) const { return frames_.contains(frame); }

    /// The Frames sharing at least one word with `bow` and at least `min_common_ratio` times as
    /// many words as the best-sharing Frame, with their L1 score. Sorted by score descending, ties
    /// by ascending FrameId. Empty when nothing is shared.
    [[nodiscard]] std::vector<DatabaseHit> Query(const BowVector& bow,
                                                 double min_common_ratio = 0.8) const {
        std::map<map::FrameId, uint32_t> common;
        for (const auto& [word, weight] : bow.entries) {
            const auto list = words_.find(word);
            if (list == words_.end()) continue;
            for (const map::FrameId frame : list->second) ++common[frame];
        }
        uint32_t max_common = 0;
        for (const auto& [frame, count] : common) max_common = std::max(max_common, count);
        std::vector<DatabaseHit> hits;
        if (max_common == 0) return hits;
        const double threshold = min_common_ratio * static_cast<double>(max_common);
        for (const auto& [frame, count] : common) {
            if (static_cast<double>(count) < threshold) continue;
            hits.push_back({frame, count, Score(bow, frames_.at(frame))});
        }
        std::sort(hits.begin(), hits.end(), [](const DatabaseHit& a, const DatabaseHit& b) {
            return a.score != b.score ? a.score > b.score : a.frame < b.frame;
        });
        return hits;
    }

private:
    std::unordered_map<WordId, std::vector<map::FrameId>> words_;
    std::map<map::FrameId, BowVector> frames_;
};

}  // namespace voyahslam::bow

#endif  // VOYAHSLAM_BOW_FRAME_DATABASE_H
