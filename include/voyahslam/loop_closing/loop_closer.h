#ifndef VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
#define VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H

#include <voyahslam/events/event_node.h>

namespace voyahslam::loop_closing {

// Detects loops on its own replica of the map and publishes LoopDetected.
// Detection only — the Mapper closes the loop. Consumes the Mapper's map diffs.
template <typename TFeatureExtractor>
class LoopCloser : public events::EventNode {
public:
    void Start();
    void Stop();
};

} // namespace voyahslam::loop_closing

#endif // VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
