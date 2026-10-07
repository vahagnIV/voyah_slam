#include <algorithm>
#include <voyahslam/map/covisibility_index.h>

namespace voyahslam::map {
namespace {

const std::vector<PointObservation> kNoObservations;
const std::vector<MapPointId> kNoPoints;

} // namespace

void CovisibilityIndex::Add(FrameId frame, uint32_t feature, MapPointId point) {
    if (Contains(frame, point)) {
        return;
    }
    if (point.value >= observations_.size()) {
        observations_.resize(static_cast<size_t>(point.value) + 1);
    }
    observations_[point.value].push_back({frame, feature});
    points_[frame.value].push_back(point);
}

void CovisibilityIndex::Clear() {
    observations_.clear();
    points_.clear();
}

const std::vector<PointObservation>& CovisibilityIndex::Observations(MapPointId point) const {
    return point.value < observations_.size() ? observations_[point.value] : kNoObservations;
}

const std::vector<MapPointId>& CovisibilityIndex::Points(FrameId frame) const {
    const auto it = points_.find(frame.value);
    return it == points_.end() ? kNoPoints : it->second;
}

bool CovisibilityIndex::Contains(FrameId frame, MapPointId point) const {
    const std::vector<PointObservation>& observations = Observations(point);
    return std::any_of(observations.begin(), observations.end(),
                       [&](const PointObservation& o) { return o.frame == frame; });
}

std::vector<std::pair<FrameId, uint32_t>> CovisibilityIndex::Covisible(FrameId frame,
                                                                       size_t max_count) const {
    std::unordered_map<uint32_t, uint32_t> shared;  // by FrameId::value
    for (const MapPointId point : Points(frame)) {
        for (const PointObservation& observation : Observations(point)) {
            if (observation.frame != frame) {
                ++shared[observation.frame.value];
            }
        }
    }
    std::vector<std::pair<FrameId, uint32_t>> covisible;
    covisible.reserve(shared.size());
    for (const auto& [id, count] : shared) {
        covisible.emplace_back(FrameId{id}, count);
    }
    std::sort(covisible.begin(), covisible.end(), [](const auto& a, const auto& b) {
        return a.second != b.second ? a.second > b.second : a.first < b.first;
    });
    if (covisible.size() > max_count) {
        covisible.resize(max_count);
    }
    return covisible;
}

} // namespace voyahslam::map
