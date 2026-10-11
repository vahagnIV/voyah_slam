#ifndef VOYAHSLAM_FEATURES_INSTANTIATION_H
#define VOYAHSLAM_FEATURES_INSTANTIATION_H

#include <voyahslam/features/orb_feature_extractor.h>

#define INSTANTIATE_CLASS(class_name) \
    template class class_name<voyahslam::features::OrbFeatureExtractor>;

#endif // VOYAHSLAM_FEATURES_INSTANTIATION_H
