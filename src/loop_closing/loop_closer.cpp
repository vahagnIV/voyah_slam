#include <voyahslam/features/instantiation.h>
#include <voyahslam/loop_closing/loop_closer.h>

namespace voyahslam::loop_closing {

template <typename TFeatureExtractor>
LoopCloser<TFeatureExtractor>::~LoopCloser() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Start() {
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Stop() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
bool LoopCloser<TFeatureExtractor>::ProcessPending() {
    bool processed = false;
    while (inbox_listener_.GetNextEvent() != nullptr) {
        processed = true;  // acknowledged by the next GetNextEvent()
    }
    return processed;
}

INSTANTIATE_CLASS(LoopCloser)

} // namespace voyahslam::loop_closing
