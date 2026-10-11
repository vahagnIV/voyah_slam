#ifndef VOYAHSLAM_TESTS_SUPPORT_TEST_MEASUREMENTS_H
#define VOYAHSLAM_TESTS_SUPPORT_TEST_MEASUREMENTS_H

#include <cstddef>
#include <memory>
#include <utility>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/features/stereo_rig.h>

namespace voyahslam::test {

using TestExtractor = features::OrbFeatureExtractor;
using TestImageFeatures = features::ImageFeatures<TestExtractor::TDescriptorScalarType,
                                                  TestExtractor::DescriptorSize>;

// `measurement` as events and replicas hold it (shared, immutable), for tests
// that build a NewFrame or a FrameTracked themselves.
template <typename TFeatureExtractor = TestExtractor>
features::TSharedMeasurement<TFeatureExtractor> Share(features::TMeasurement<TFeatureExtractor> measurement) {
    return std::make_shared<const features::TMeasurement<TFeatureExtractor>>(std::move(measurement));
}

inline geometry::TMatrix33F TestIntrinsics() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = 500.0F;
    K(1, 1) = 500.0F;
    K(0, 2) = 320.0F;
    K(1, 2) = 240.0F;
    return K;
}

// Points lie within +-100 px of the principal point, so no feature is dropped
// by the 85 degree ray cut.
inline TestImageFeatures MakeImageFeatures(size_t num_features) {
    const int n = static_cast<int>(num_features);
    geometry::TPoints2DF points = geometry::TPoints2DF::Random(2, n) * 100.0F;
    points.row(0).array() += 320.0F;
    points.row(1).array() += 240.0F;
    geometry::TRowVectorF strengths = geometry::TRowVectorF::Random(n);
    geometry::TRowVectorF angles = geometry::TRowVectorF::Random(n);
    TestImageFeatures::TDescriptors descriptors =
        TestImageFeatures::TDescriptors::Random(TestExtractor::DescriptorSize, n);
    return TestImageFeatures(std::move(points), std::move(strengths), std::move(angles),
                             std::move(descriptors),
                             features::TLevels::Zero(n));
}

inline features::TMonocularMeasurement<TestExtractor> MakeMonocular(size_t num_features,
                                                                    double timestamp) {
    return features::TMonocularMeasurement<TestExtractor>(MakeImageFeatures(num_features),
                                                          TestIntrinsics(), nullptr, timestamp);
}

inline features::TBinocularMeasurement<TestExtractor> MakeBinocular(size_t num_features,
                                                                    double timestamp) {
    features::StereoRig rig{
        .K_left = TestIntrinsics(),
        .K_right = TestIntrinsics(),
        .T_rl = geometry::RigidTransformation(geometry::TMatrix33F::Identity(),
                                                      geometry::TVector3DF(-0.12, 0.0, 0.0)),
        .baseline = 0.12,
    };
    return features::TBinocularMeasurement<TestExtractor>(
        MakeImageFeatures(num_features), MakeImageFeatures(num_features), rig, timestamp);
}

} // namespace voyahslam::test

#endif // VOYAHSLAM_TESTS_SUPPORT_TEST_MEASUREMENTS_H
