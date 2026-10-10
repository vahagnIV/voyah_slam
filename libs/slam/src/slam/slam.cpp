#include <utility>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/slam/slam.h>

namespace voyahslam::slam {

template <typename TFeatureExtractor>
Slam<TFeatureExtractor>::Slam(mapping::MapperOptions mapper_options,
                              tracking::TrackerOptions tracker_options,
                              std::shared_ptr<const typename TMapper::TVocabulary> vocabulary,
                              loop_closing::LoopCloserOptions loop_closer_options,
                              global_ba::GlobalBaOptions global_ba_options)
    : tracker_(tracker_options, vocabulary),
      mapper_(mapper_options, vocabulary),
      loop_closer_(loop_closer_options, std::move(vocabulary)) {
    tracker_.AddObserver(mapper_.inbox());
    mapper_.AddObserver(tracker_.inbox());
    mapper_.AddObserver(loop_closer_.inbox());
    loop_closer_.AddObserver(mapper_.inbox());
    if (loop_closer_options.enabled && global_ba_options.enabled) {
        global_ba_.emplace(global_ba_options);
        mapper_.AddObserver(global_ba_->inbox());
        global_ba_->AddObserver(mapper_.inbox());
    }
}

template <typename TFeatureExtractor>
Slam<TFeatureExtractor>::~Slam() {
    Stop();
}

template <typename TFeatureExtractor>
void Slam<TFeatureExtractor>::Start() {
    mapper_.Start();
    if (global_ba_) global_ba_->Start();
    loop_closer_.Start();
    tracker_.Start();
}

template <typename TFeatureExtractor>
void Slam<TFeatureExtractor>::Stop() {
    tracker_.Stop();
    loop_closer_.Stop();
    if (global_ba_) global_ba_->Stop();
    mapper_.Stop();
}

INSTANTIATE_CLASS(Slam)

} // namespace voyahslam::slam
