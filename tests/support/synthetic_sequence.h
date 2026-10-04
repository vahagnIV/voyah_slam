#ifndef VOYAHSLAM_TESTS_SUPPORT_SYNTHETIC_SEQUENCE_H
#define VOYAHSLAM_TESTS_SUPPORT_SYNTHETIC_SEQUENCE_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <voyahslam/defs.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>

#include "support/test_measurements.h"

namespace voyahslam::test {

struct SyntheticSequenceOptions {
    size_t num_points = 300;
    size_t num_frames = 30;
    double step = 0.05;            ///< Camera centre moves +x per frame (world units).
    double yaw_step_deg = 0.2;     ///< Rotation about the camera y axis per frame.
    double min_depth = 4.0, max_depth = 8.0;
    double pixel_noise = 0.5;      ///< Gaussian sigma, px.
    int bit_flips = 3;             ///< Descriptor bits flipped per point per frame.
    double angle_noise = 0.02;     ///< rad.
    size_t num_extra = 20;         ///< Unmatched random features per frame.
    double frame_interval = 0.05;  ///< s.
    uint32_t seed = 1;
};

struct SyntheticFrame {
    features::TMonocularMeasurement<TestExtractor> measurement;
    geometry::RigidTransformation T_cw;
    /// Per feature (in measurement order): scene point index, -1 for extras.
    std::vector<Eigen::Index> point_of_feature;
};

struct SyntheticSequence {
    geometry::TPoints3DF points_w;  ///< World frame = frame 0's camera.
    std::vector<SyntheticFrame> frames;
};

/// fx = fy = 190, principal point at the centre of a 512 x 512 image.
inline geometry::TMatrix33F SyntheticIntrinsics() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = 190.0;
    K(1, 1) = 190.0;
    K(0, 2) = 256.0;
    K(1, 2) = 256.0;
    return K;
}

namespace detail {

constexpr double kSyntheticImageSize = 512.0;

using SyntheticDescriptor = TestImageFeatures::TDescriptor;

inline SyntheticDescriptor RandomDescriptor(std::mt19937& rng) {
    std::uniform_int_distribution<int> byte(0, 255);
    SyntheticDescriptor descriptor;
    for (Eigen::Index i = 0; i < descriptor.size(); ++i) {
        descriptor[i] = static_cast<uint8_t>(byte(rng));
    }
    return descriptor;
}

inline SyntheticDescriptor FlipBits(SyntheticDescriptor descriptor, int count, std::mt19937& rng) {
    std::uniform_int_distribution<int> bit(0, static_cast<int>(descriptor.size()) * 8 - 1);
    for (int i = 0; i < count; ++i) {
        const int b = bit(rng);
        descriptor[b / 8] ^= static_cast<uint8_t>(1U << (b % 8));
    }
    return descriptor;
}

}  // namespace detail

inline SyntheticSequence MakeSyntheticSequence(const SyntheticSequenceOptions& options = {}) {
    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<double> pixel(20.0, 492.0);
    std::uniform_real_distribution<double> depth(options.min_depth, options.max_depth);
    std::uniform_real_distribution<double> angle(0.0, 2.0 * std::numbers::pi);
    std::uniform_real_distribution<double> any_pixel(0.0, detail::kSyntheticImageSize);
    std::normal_distribution<double> unit_normal(0.0, 1.0);

    const geometry::TMatrix33F K = SyntheticIntrinsics();
    const geometry::TMatrix33F K_inverse = K.inverse();
    const auto n = static_cast<Eigen::Index>(options.num_points);

    SyntheticSequence sequence;
    sequence.points_w.resize(3, n);
    std::vector<detail::SyntheticDescriptor> base_descriptors;
    std::vector<double> base_angles;
    for (Eigen::Index i = 0; i < n; ++i) {
        const geometry::TVector3DF ray(pixel(rng), pixel(rng), 1.0);
        sequence.points_w.col(i) = depth(rng) * (K_inverse * ray);
        base_descriptors.push_back(detail::RandomDescriptor(rng));
        base_angles.push_back(angle(rng));
    }

    const double yaw_step = options.yaw_step_deg * std::numbers::pi / 180.0;
    for (size_t k = 0; k < options.num_frames; ++k) {
        const geometry::TMatrix33F R_cw =
            Eigen::AngleAxis<double>(static_cast<double>(k) * yaw_step,
                                     geometry::TVector3DF::UnitY())
                .toRotationMatrix();
        const geometry::TVector3DF centre(static_cast<double>(k) * options.step, 0.0, 0.0);
        const geometry::RigidTransformation T_cw(R_cw, -R_cw * centre);
        const geometry::TPoints3DF points_c = T_cw.Transform(sequence.points_w);

        std::vector<Eigen::Index> point_of_feature;
        std::vector<geometry::TPoint2DF> pixels;
        std::vector<detail::SyntheticDescriptor> descriptors;
        std::vector<double> angles;
        for (Eigen::Index i = 0; i < n; ++i) {
            if (points_c(2, i) <= 0.0) {
                continue;
            }
            geometry::TPoint2DF projected = (K * points_c.col(i)).hnormalized();
            projected += options.pixel_noise * geometry::TPoint2DF(unit_normal(rng), unit_normal(rng));
            if (projected.x() < 0.0 || projected.x() >= detail::kSyntheticImageSize ||
                projected.y() < 0.0 || projected.y() >= detail::kSyntheticImageSize) {
                continue;
            }
            point_of_feature.push_back(i);
            pixels.push_back(projected);
            descriptors.push_back(detail::FlipBits(base_descriptors[i], options.bit_flips, rng));
            angles.push_back(base_angles[i] + options.angle_noise * unit_normal(rng));
        }
        for (size_t e = 0; e < options.num_extra; ++e) {
            point_of_feature.push_back(-1);
            pixels.emplace_back(any_pixel(rng), any_pixel(rng));
            descriptors.push_back(detail::RandomDescriptor(rng));
            angles.push_back(angle(rng));
        }

        const auto count = static_cast<Eigen::Index>(point_of_feature.size());
        std::vector<Eigen::Index> order(count);
        for (Eigen::Index i = 0; i < count; ++i) {
            order[i] = i;
        }
        std::shuffle(order.begin(), order.end(), rng);

        geometry::TPoints2DF image_points(2, count);
        geometry::TRowVectorF strengths = geometry::TRowVectorF::Ones(count);
        geometry::TRowVectorF feature_angles(count);
        TestImageFeatures::TDescriptors feature_descriptors(TestExtractor::DescriptorSize, count);
        SyntheticFrame frame;
        frame.T_cw = T_cw;
        for (Eigen::Index j = 0; j < count; ++j) {
            const Eigen::Index src = order[j];
            image_points.col(j) = pixels[src];
            feature_angles[j] = angles[src];
            feature_descriptors.col(j) = descriptors[src];
            frame.point_of_feature.push_back(point_of_feature[src]);
        }
        frame.measurement = features::TMonocularMeasurement<TestExtractor>(
            TestImageFeatures(std::move(image_points), std::move(strengths),
                              std::move(feature_angles), std::move(feature_descriptors),
                              features::TLevels::Zero(count)),
            K, nullptr, static_cast<double>(k) * options.frame_interval);
        assert(static_cast<size_t>(frame.measurement.points().cols()) ==
               frame.point_of_feature.size());
        sequence.frames.push_back(std::move(frame));
    }
    return sequence;
}

}  // namespace voyahslam::test

#endif  // VOYAHSLAM_TESTS_SUPPORT_SYNTHETIC_SEQUENCE_H
