#ifndef VOYAHSLAM_MAPPING_MAPPER_H
#define VOYAHSLAM_MAPPING_MAPPER_H

#include <voyahslam/events/event_node.h>

namespace voyahslam::mapping {

// Owns the authoritative map and is its only writer: creates keyframes and map
// points, runs local BA and culling, and performs loop closure. Publishes map
// diffs; consumes NewKeyFrameCandidate (Tracker) and LoopDetected (LoopCloser).
template <typename TFeatureExtractor>
class Mapper : public events::EventNode {
public:
    void Start();
    void Stop();
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAPPER_H
