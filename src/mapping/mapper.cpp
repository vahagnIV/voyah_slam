#include <voyahslam/features/instantiation.h>
#include <voyahslam/mapping/mapper.h>

namespace voyahslam::mapping {

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Start() {}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Stop() {}

INSTANTIATE_CLASS(Mapper)

} // namespace voyahslam::mapping
