#ifndef VOYAHSLAM_FEATURES_BRUTE_FORCE_MATCHER_H
#define VOYAHSLAM_FEATURES_BRUTE_FORCE_MATCHER_H

#include <voyahslam/features/keypoint_matches.h>

namespace voyahslam::features {

template <typename TFeatureExtractor>
class BruteForceMatcher {
public:
    typedef typename TFeatureExtractor::TDescriptors TDescriptorList;
    typedef typename TFeatureExtractor::TDescriptor TDescriptorElem;
    typedef typename TFeatureExtractor::distance_f distance_f;
    typedef typename TFeatureExtractor::TDescriptorScalarType TDescType;

    explicit BruteForceMatcher(distance_f distane_function) : distance_function_(distane_function) {}

    KeypointMatches Match(const TDescriptorList& descriptor_list1, const TDescriptorList& descriptor_list2) const;

private:
    distance_f distance_function_;
};

template <typename TFeatureExtractor>
KeypointMatches BruteForceMatcher<TFeatureExtractor>::
Match(const TDescriptorList& setA, const TDescriptorList& setB) const {
    size_t sizeA = setA.cols(), sizeB = setB.cols();
    std::vector<size_t> bestMatchA(sizeA, -1), bestMatchB(sizeB, -1);
    std::vector<TDescType> bestDistA(sizeA, std::numeric_limits<TDescType>::max());
    std::vector<TDescType> bestDistB(sizeB, std::numeric_limits<TDescType>::max());

#pragma omp parallel for
    for (size_t i = 0; i < sizeA; i++) {
        for (size_t j = 0; j < sizeB; j++) {
            TDescType dist = distance_function_(setA.col(i), setB.col(j));
            if (dist < bestDistA[i]) {
                bestDistA[i] = dist;
                bestMatchA[i] = j;
            }
        }
    }

#pragma omp parallel for
    for (size_t j = 0; j < sizeB; j++) {
        for (size_t i = 0; i < sizeA; i++) {
            TDescType dist = distance_function_(setB.col(j), setA.col(i));
            if (dist < bestDistB[j]) {
                bestDistB[j] = dist;
                bestMatchB[j] = i;
            }
        }
    }

    KeypointMatches matches;
    for (size_t i = 0; i < sizeA; i++) {
        size_t j = bestMatchA[i];
        if (j != -1 && bestMatchB[j] == i) matches.add_match(i, j, bestDistA[i]);
    }

    return matches;
}

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_BRUTE_FORCE_MATCHER_H
