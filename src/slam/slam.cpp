#include <voyahslam/features/instantiation.h>
#include <voyahslam/slam/slam.h>

namespace voyahslam::slam {

template <typename TFeatureExtractor>
Slam<TFeatureExtractor>::Slam() {
    tracker_.AddObserver(mapper_.inbox());
    mapper_.AddObserver(tracker_.inbox());
    mapper_.AddObserver(loop_closer_.inbox());
    loop_closer_.AddObserver(mapper_.inbox());
}

template <typename TFeatureExtractor>
Slam<TFeatureExtractor>::~Slam() {
    Stop();
}

template <typename TFeatureExtractor>
void Slam<TFeatureExtractor>::Start() {
    mapper_.Start();
    loop_closer_.Start();
    tracker_.Start();
}

template <typename TFeatureExtractor>
void Slam<TFeatureExtractor>::Stop() {
    tracker_.Stop();
    loop_closer_.Stop();
    mapper_.Stop();
}

INSTANTIATE_CLASS(Slam)

} // namespace voyahslam::slam
