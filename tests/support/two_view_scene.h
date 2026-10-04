#ifndef VOYAHSLAM_TESTS_SUPPORT_TWO_VIEW_SCENE_H
#define VOYAHSLAM_TESTS_SUPPORT_TWO_VIEW_SCENE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include <voyahslam/defs.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/geometry/two_view_correspondences.h>

namespace voyahslam::test {

struct TwoViewScene {
    geometry::TwoViewCorrespondences correspondences;
    geometry::RigidTransformation T_21;  ///< Camera 1 -> camera 2.
    geometry::TPoints3DF points;         ///< Ground-truth points in camera 1.
    Eigen::Array<bool, Eigen::Dynamic, 1> is_outlier;
};

/// 5 degrees about a fixed non-axis-aligned axis, translation 0.5 m.
inline geometry::RigidTransformation DefaultTransform() {
    const geometry::TVector3DF axis = geometry::TVector3DF(1.0, 2.0, 3.0).normalized();
    const double angle = 5.0 * M_PI / 180.0;
    const geometry::TMatrix33F rotation = Eigen::AngleAxis<double>(angle, axis).toRotationMatrix();
    const geometry::TVector3DF translation = geometry::TVector3DF(1.0, 0.3, 0.2).normalized() * 0.5;
    return geometry::RigidTransformation(rotation, translation);
}

/// Pixel noise expressed in normalized units.
inline double NormalizedSigma(double sigma_px, double focal = 190.0) {
    return sigma_px / focal;
}

namespace detail {

// Projects points through T_21, adds noise and replaces a fraction with outliers.
inline TwoViewScene FinishScene(const geometry::TPoints3DF& points, double outlier_fraction,
                                double sigma, const geometry::RigidTransformation& T_21,
                                std::mt19937& rng) {
    const Eigen::Index n = points.cols();
    const geometry::TPoints3DF points_2 = T_21.Transform(points);

    TwoViewScene scene;
    scene.T_21 = T_21;
    scene.points = points;
    scene.correspondences.sigma = sigma;
    scene.correspondences.x1.resize(2, n);
    scene.correspondences.x2.resize(2, n);
    scene.is_outlier = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(n, false);

    std::normal_distribution<double> noise(0.0, sigma > 0.0 ? sigma : 1.0);
    std::uniform_real_distribution<double> outlier_coordinate(-0.6, 0.6);
    for (Eigen::Index i = 0; i < n; ++i) {
        for (int row = 0; row < 2; ++row) {
            scene.correspondences.x1(row, i) = points(row, i) / points(2, i);
            scene.correspondences.x2(row, i) = points_2(row, i) / points_2(2, i);
        }
        if (sigma > 0.0) {
            for (int row = 0; row < 2; ++row) {
                scene.correspondences.x1(row, i) += noise(rng);
                scene.correspondences.x2(row, i) += noise(rng);
            }
        }
    }

    const auto num_outliers = static_cast<Eigen::Index>(
        std::llround(outlier_fraction * static_cast<double>(n)));
    std::vector<Eigen::Index> order(static_cast<size_t>(n));
    std::iota(order.begin(), order.end(), Eigen::Index{0});
    std::shuffle(order.begin(), order.end(), rng);
    for (Eigen::Index k = 0; k < num_outliers; ++k) {
        const Eigen::Index i = order[static_cast<size_t>(k)];
        scene.is_outlier(i) = true;
        scene.correspondences.x2(0, i) = outlier_coordinate(rng);
        scene.correspondences.x2(1, i) = outlier_coordinate(rng);
    }
    return scene;
}

} // namespace detail

/// Default half field of view of MakeGeneralScene: tan(30 degrees).
inline const double kGeneralHalfFovTan = std::tan(30.0 * M_PI / 180.0);

/// General 3D scene: depth uniform in [4, 8]; normalized coordinates u, v of view 1 uniform in
/// [-half_fov_tan, half_fov_tan] (default: within +-30 degrees of the optical axis).
inline TwoViewScene MakeGeneralScene(size_t n, double outlier_fraction, double sigma,
                                     uint32_t seed, const geometry::RigidTransformation& T_21,
                                     double half_fov_tan = kGeneralHalfFovTan) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> direction(-half_fov_tan, half_fov_tan);
    std::uniform_real_distribution<double> depth(4.0, 8.0);
    geometry::TPoints3DF points(3, static_cast<Eigen::Index>(n));
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
        const double z = depth(rng);
        points.col(i) << z * direction(rng), z * direction(rng), z;
    }
    return detail::FinishScene(points, outlier_fraction, sigma, T_21, rng);
}

/// Planar scene: points on the plane z = 5 in camera 1, optionally tilted about the y axis
/// (through the point (0, 0, 5)) by tilt_deg; normalized coordinates u, v of view 1 uniform in
/// [-half_fov_tan, half_fov_tan].
inline TwoViewScene MakePlanarScene(size_t n, double outlier_fraction, double sigma,
                                    uint32_t seed, const geometry::RigidTransformation& T_21,
                                    double tilt_deg = 0.0, double half_fov_tan = 0.5) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> direction(-half_fov_tan, half_fov_tan);
    const double tilt = tilt_deg * M_PI / 180.0;
    const geometry::TVector3DF normal(std::sin(tilt), 0.0, std::cos(tilt));
    const geometry::TVector3DF plane_point(0.0, 0.0, 5.0);
    const double plane_offset = normal.dot(plane_point);
    geometry::TPoints3DF points(3, static_cast<Eigen::Index>(n));
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
        const geometry::TVector3DF ray(direction(rng), direction(rng), 1.0);
        points.col(i) = ray * (plane_offset / normal.dot(ray));
    }
    return detail::FinishScene(points, outlier_fraction, sigma, T_21, rng);
}

} // namespace voyahslam::test

#endif // VOYAHSLAM_TESTS_SUPPORT_TWO_VIEW_SCENE_H
