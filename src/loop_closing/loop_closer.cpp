#include <voyahslam/features/instantiation.h>
#include <voyahslam/loop_closing/loop_closer.h>

namespace voyahslam::loop_closing {

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Start() {}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Stop() {}

INSTANTIATE_CLASS(LoopCloser)

} // namespace voyahslam::loop_closing
