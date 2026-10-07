#ifndef VOYAHSLAM_TRACKING_LOCAL_MAP_H
#define VOYAHSLAM_TRACKING_LOCAL_MAP_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <utility>
#include <vector>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::tracking {

// The part of the replica a measurement is tracked against (ORB-SLAM3
// UpdateLocalKeyFrames / UpdateLocalPoints, without a spanning tree).
struct LocalMap {
    std::vector<map::FrameId> frames;  // K1 by match count, then K2
    std::vector<bool> point_mask;      // by MapPointId::value: alive and seen by a local Frame
    bool all_points = false;           // no matched point has a known Frame: every point is local
};

namespace detail {

// The Frames of `replica` observing the alive matched points, with the number
// of matched points each observes: by that count descending, ties by FrameId
// ascending.
template <typename TFeatureExtractor>
std::vector<std::pair<map::FrameId, size_t>> FramesObservingMatches(
    const MapReplica<TFeatureExtractor>& replica, const std::vector<int64_t>& matches) {
    std::map<map::FrameId, size_t> counts;
    for (const int64_t id : matches) {
        if (id < 0 || id > std::numeric_limits<uint32_t>::max()) {
            continue;
        }
        const map::MapPointId point{static_cast<uint32_t>(id)};
        if (!replica.IsAlive(point)) {
            continue;
        }
        for (const map::PointObservation& observation : replica.covisibility().Observations(point)) {
            if (replica.keyframe(observation.frame) != nullptr) {
                ++counts[observation.frame];
            }
        }
    }
    std::vector<std::pair<map::FrameId, size_t>> frames(counts.begin(), counts.end());
    std::stable_sort(frames.begin(), frames.end(),
                     [](const auto& a, const auto& b) { return a.second > b.second; });
    return frames;
}

} // namespace detail

// K1: the Frames observing any alive match (per feature, MapPointId value or
// -1), by match count descending, ties by id. K2: each K1 Frame's top
// `local_neighbours` covisible Frames not yet included. At most
// `max_local_frames`, K1 first. The mask holds the alive points of those
// Frames; with no known Frame, all_points is set instead.
template <typename TFeatureExtractor>
LocalMap BuildLocalMap(const MapReplica<TFeatureExtractor>& replica,
                       const std::vector<int64_t>& matches, size_t local_neighbours,
                       size_t max_local_frames) {
    LocalMap local;
    const auto k1 = detail::FramesObservingMatches(replica, matches);
    if (k1.empty()) {
        local.all_points = true;
        return local;
    }
    for (const auto& [frame, count] : k1) {
        if (local.frames.size() >= max_local_frames) {
            break;
        }
        local.frames.push_back(frame);
    }
    const size_t num_k1 = local.frames.size();
    for (size_t i = 0; i < num_k1 && local.frames.size() < max_local_frames; ++i) {
        for (const auto& [neighbour, shared] :
             replica.covisibility().Covisible(local.frames[i], local_neighbours)) {
            if (local.frames.size() >= max_local_frames) {
                break;
            }
            if (replica.keyframe(neighbour) == nullptr ||
                std::find(local.frames.begin(), local.frames.end(), neighbour) !=
                    local.frames.end()) {
                continue;
            }
            local.frames.push_back(neighbour);
        }
    }

    local.point_mask.assign(replica.num_points(), false);
    for (const map::FrameId frame : local.frames) {
        for (const map::MapPointId point : replica.covisibility().Points(frame)) {
            if (replica.IsAlive(point)) {
                local.point_mask[point.value] = true;
            }
        }
    }
    return local;
}

// The K1 Frame observing the most inlier matches (per feature, MapPointId
// value or -1), ties by the smaller id; nullopt if no inlier has a known Frame.
template <typename TFeatureExtractor>
std::optional<map::FrameId> ReferenceFrame(const MapReplica<TFeatureExtractor>& replica,
                                           const std::vector<int64_t>& inlier_matches) {
    const auto frames = detail::FramesObservingMatches(replica, inlier_matches);
    if (frames.empty()) {
        return std::nullopt;
    }
    return frames.front().first;
}

// The point source of features::SearchFromLocalMap restricted to a local map:
// forwards to the replica, with IsAlive also requiring the local mask.
template <typename TFeatureExtractor>
class LocalPointSource {
public:
    LocalPointSource(const MapReplica<TFeatureExtractor>& replica, const LocalMap& local)
        : replica_(replica), local_(local) {}

    [[nodiscard]] size_t num_points() const { return replica_.num_points(); }
    [[nodiscard]] bool IsAlive(map::MapPointId id) const {
        return replica_.IsAlive(id) &&
               (local_.all_points ||
                (id.value < local_.point_mask.size() && local_.point_mask[id.value]));
    }
    [[nodiscard]] decltype(auto) Position(map::MapPointId id) const { return replica_.Position(id); }
    [[nodiscard]] decltype(auto) Descriptor(map::MapPointId id) const {
        return replica_.Descriptor(id);
    }
    [[nodiscard]] decltype(auto) Normal(map::MapPointId id) const { return replica_.Normal(id); }
    [[nodiscard]] decltype(auto) Reference(map::MapPointId id) const {
        return replica_.Reference(id);
    }

private:
    const MapReplica<TFeatureExtractor>& replica_;
    const LocalMap& local_;
};

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_LOCAL_MAP_H
