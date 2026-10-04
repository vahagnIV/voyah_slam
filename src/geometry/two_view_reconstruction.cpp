#include "voyahslam/geometry/two_view_reconstruction.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "voyahslam/estimators/essential_estimator.h"
#include "voyahslam/estimators/homography_estimator.h"
#include "voyahslam/geometry/geometry.h"
#include "voyahslam/geometry/two_view_decomposition.h"

namespace voyahslam::geometry {

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
// The parallax reported for a candidate is that of the (k+1)-th largest-parallax good point.
constexpr size_t kParallaxRank = 50;

struct CandidateCheck {
    TPoints3DF points;
    Eigen::Array<bool, Eigen::Dynamic, 1> triangulated;
    size_t num_good = 0;
    double parallax_deg = 0.0;
};

// ORB-SLAM3 CheckRT in normalized coordinates.
CandidateCheck CheckCandidate(const RigidTransformation& a_T_21, const TwoViewCorrespondences& a_correspondences,
                              const Eigen::Array<bool, Eigen::Dynamic, 1>& a_inliers,
                              const TwoViewOptions& a_options) {
    const Eigen::Index n = a_correspondences.size();
    const double max_squared_error = 4.0 * a_correspondences.sigma * a_correspondences.sigma;
    const TMatrix34F P1 = RigidTransformation::Identity().ToMatrix34();
    const TMatrix34F P2 = a_T_21.ToMatrix34();

    CandidateCheck check;
    check.points = TPoints3DF::Zero(3, n);
    check.triangulated = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(n, false);
    std::vector<double> parallax_cos;

    for (Eigen::Index i = 0; i < n; ++i) {
        if (!a_inliers(i)) {
            continue;
        }
        const TPoint2DF x1 = a_correspondences.x1.col(i);
        const TPoint2DF x2 = a_correspondences.x2.col(i);
        TPoint3DF p1;
        if (!Triangulate(x1.homogeneous(), x2.homogeneous(), P1, P2, p1) || !p1.allFinite()) {
            continue;
        }
        const double cos_parallax = ComputeParallaxCos(p1, P2);
        if (!std::isfinite(cos_parallax)) {
            continue;
        }
        const bool near = cos_parallax < a_options.max_point_parallax_cos;
        const TPoint3DF p2 = a_T_21.Transform(p1);
        // For far points (cos >= threshold) the depth sign is unreliable and is not tested.
        if ((p1.z() <= 0.0 || p2.z() <= 0.0) && near) {
            continue;
        }
        if ((p1.hnormalized() - x1).squaredNorm() > max_squared_error ||
            (p2.hnormalized() - x2).squaredNorm() > max_squared_error) {
            continue;
        }

        ++check.num_good;
        parallax_cos.push_back(cos_parallax);
        if (near) {
            check.points.col(i) = p1;
            check.triangulated(i) = true;
        }
    }

    if (!parallax_cos.empty()) {
        std::sort(parallax_cos.begin(), parallax_cos.end());
        const size_t k = std::min(kParallaxRank, parallax_cos.size() - 1);
        check.parallax_deg = std::acos(parallax_cos[k]) * kRadToDeg;
    }
    return check;
}

} // namespace

PoseSelection SelectPose(std::span<const RigidTransformation> a_candidates,
                         const TwoViewCorrespondences& a_correspondences,
                         const Eigen::Array<bool, Eigen::Dynamic, 1>& a_inliers, double a_uniqueness_ratio,
                         const TwoViewOptions& a_options) {
    PoseSelection selection;
    if (a_candidates.empty()) {
        return selection;
    }

    std::vector<size_t> num_good(a_candidates.size());
    CandidateCheck best;
    for (size_t c = 0; c < a_candidates.size(); ++c) {
        CandidateCheck check = CheckCandidate(a_candidates[c], a_correspondences, a_inliers, a_options);
        num_good[c] = check.num_good;
        if (c == 0 || check.num_good > best.num_good) {
            selection.best_index = c;
            best = std::move(check);
        }
    }

    selection.points = std::move(best.points);
    selection.triangulated = std::move(best.triangulated);
    selection.num_good = best.num_good;
    selection.parallax_deg = best.parallax_deg;

    const auto num_inliers = static_cast<double>(a_inliers.count());
    const double min_good =
        std::max(a_options.min_good_fraction * num_inliers, static_cast<double>(a_options.min_triangulated));
    const double max_other_good = a_uniqueness_ratio * static_cast<double>(selection.num_good);
    bool unique = true;
    for (size_t c = 0; c < num_good.size(); ++c) {
        if (c != selection.best_index && static_cast<double>(num_good[c]) > max_other_good) {
            unique = false;
        }
    }
    selection.success = static_cast<double>(selection.num_good) > min_good && unique &&
                        selection.parallax_deg >= a_options.min_parallax_deg;
    return selection;
}

TwoViewReconstructor::TwoViewReconstructor(TwoViewOptions a_options) : options_(std::move(a_options)) {}

TwoViewReconstruction TwoViewReconstructor::Reconstruct(const TwoViewCorrespondences& a_correspondences) const {
    TwoViewReconstruction reconstruction;
    // The estimators' scores and the reprojection gate divide by sigma.
    if (!(a_correspondences.sigma > 0.0)) {
        return reconstruction;
    }

    estimators::RansacOptions essential_ransac = options_.ransac;
    essential_ransac.seed = options_.ransac.seed + 1;
    const auto homography = estimators::Ransac<estimators::HomographyEstimator>().Run(a_correspondences,
                                                                                      options_.ransac);
    const auto essential = estimators::Ransac<estimators::EssentialEstimator>().Run(a_correspondences,
                                                                                    essential_ransac);
    if (!homography.success && !essential.success) {
        return reconstruction;
    }
    reconstruction.homography_score = homography.success ? homography.score.score : 0.0;
    reconstruction.essential_score = essential.success ? essential.score.score : 0.0;
    // A calibrated E fits planar points as well as general ones, so the scores cannot tell the
    // models apart; H wins when it explains nearly everything E does (planar scene or rotation).
    const bool use_homography =
        homography.success &&
        (!essential.success || static_cast<double>(homography.score.num_inliers) >=
                                   options_.homography_inlier_ratio *
                                       static_cast<double>(essential.score.num_inliers));

    std::vector<RigidTransformation> candidates;
    PoseSelection selection;
    if (use_homography) {
        reconstruction.model = TwoViewModel::Homography;
        for (const HomographyCandidate& candidate : DecomposeHomography(homography.model)) {
            candidates.push_back(candidate.T_21);
        }
        selection = SelectPose(candidates, a_correspondences, homography.score.inliers,
                               options_.homography_uniqueness, options_);
    } else {
        reconstruction.model = TwoViewModel::Essential;
        const auto essential_candidates = DecomposeEssential(essential.model);
        candidates.assign(essential_candidates.begin(), essential_candidates.end());
        selection = SelectPose(candidates, a_correspondences, essential.score.inliers,
                               options_.essential_uniqueness, options_);
    }
    if (candidates.empty()) {
        return reconstruction;
    }

    reconstruction.success = selection.success;
    reconstruction.T_21 = candidates[selection.best_index];
    reconstruction.points = std::move(selection.points);
    reconstruction.triangulated = std::move(selection.triangulated);
    reconstruction.num_triangulated = static_cast<size_t>(reconstruction.triangulated.count());
    reconstruction.parallax_deg = selection.parallax_deg;
    return reconstruction;
}

} // namespace voyahslam::geometry
