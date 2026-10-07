#include <utility>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/slam/slam.h>

namespace voyahslam::slam {

template <typename TFeatureExtractor>
Slam<TFeatureExtractor>::Slam(mapping::MapperOptions mapper_options,
                              tracking::TrackerOptions tracker_options,
                              std::shared_ptr<const typename TMapper::TVocabulary> vocabulary)
    : tracker_(tracker_options), mapper_(mapper_options, std::move(vocabulary)) {
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
