#ifndef VOYAHSLAM_ESTIMATORS_RANSAC_H
#define VOYAHSLAM_ESTIMATORS_RANSAC_H

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <random>
#include <span>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/estimators/two_view_estimator.h"
#include "voyahslam/geometry/two_view_correspondences.h"

namespace voyahslam::estimators {

struct RansacOptions {
    size_t max_iterations = 2000;
    size_t min_iterations = 50;
    double confidence = 0.99;
    uint32_t seed = 0;
    /// Rounds of local refinement after sampling (only for estimators that provide Refine).
    size_t refinement_rounds = 3;
};

template <typename TModel>
struct RansacResult {
    bool success = false;
    TModel model;
    ModelScore score;
    size_t iterations = 0;
};

/// Generic adaptive RANSAC over a minimal-sample two-view estimator.
/// If the estimator also provides
///   TModel Refine(const TModel&, const TwoViewCorrespondences&, const Eigen::Array<bool, Eigen::Dynamic, 1>&) const
/// the best model is then refined on its inliers for up to RansacOptions::refinement_rounds
/// rounds, each kept only if it scores strictly higher (local optimization).
template <TwoViewEstimator TEstimator>
class Ransac {
public:
    explicit Ransac(TEstimator a_estimator = {}) : estimator_(std::move(a_estimator)) {}

    [[nodiscard]] RansacResult<typename TEstimator::TModel> Run(
        const geometry::TwoViewCorrespondences& a_correspondences, const RansacOptions& a_options) const {
        constexpr size_t kSampleSize = TEstimator::kSampleSize;
        RansacResult<typename TEstimator::TModel> result;
        const auto n = static_cast<size_t>(a_correspondences.size());
        if (n < kSampleSize) {
            return result;
        }

        std::mt19937 rng(a_options.seed);
        std::vector<Eigen::Index> indices(n);
        std::iota(indices.begin(), indices.end(), Eigen::Index{0});

        size_t required = a_options.max_iterations;
        while (result.iterations < std::clamp(required, a_options.min_iterations, a_options.max_iterations)) {
            ++result.iterations;

            // Partial Fisher-Yates: the first kSampleSize entries are a uniform distinct sample.
            for (size_t i = 0; i < kSampleSize; ++i) {
                std::uniform_int_distribution<size_t> pick(i, n - 1);
                std::swap(indices[i], indices[pick(rng)]);
            }
            const std::span<const Eigen::Index> sample(indices.data(), kSampleSize);

            bool improved = false;
            for (auto& model : estimator_.Estimate(a_correspondences, sample)) {
                ModelScore score = estimator_.Score(model, a_correspondences);
                if (score.score > result.score.score) {
                    result.success = true;
                    result.model = std::move(model);
                    result.score = std::move(score);
                    improved = true;
                }
            }
            if (improved) {
                required = RequiredIterations(result.score.num_inliers, n, kSampleSize, a_options);
            }
        }
        if (result.success) {
            RefineResult(a_correspondences, a_options, result);
        }
        return result;
    }

private:
    static constexpr bool kHasRefine = requires(const TEstimator estimator,
                                                const typename TEstimator::TModel& model,
                                                const geometry::TwoViewCorrespondences& correspondences,
                                                const Eigen::Array<bool, Eigen::Dynamic, 1>& inliers) {
        { estimator.Refine(model, correspondences, inliers) } -> std::same_as<typename TEstimator::TModel>;
    };

    // Refine on the current inliers, re-score, keep only strict improvements.
    void RefineResult(const geometry::TwoViewCorrespondences& a_correspondences, const RansacOptions& a_options,
                      RansacResult<typename TEstimator::TModel>& out_result) const {
        if constexpr (kHasRefine) {
            for (size_t round = 0; round < a_options.refinement_rounds; ++round) {
                auto model = estimator_.Refine(out_result.model, a_correspondences, out_result.score.inliers);
                ModelScore score = estimator_.Score(model, a_correspondences);
                if (!(score.score > out_result.score.score)) break;
                out_result.model = std::move(model);
                out_result.score = std::move(score);
            }
        }
    }

    static size_t RequiredIterations(size_t a_num_inliers, size_t a_n, size_t a_k, const RansacOptions& a_options) {
        const double w = static_cast<double>(a_num_inliers) / static_cast<double>(a_n);
        if (w <= 0.0) {
            return a_options.max_iterations;
        }
        const double all_inliers = std::pow(w, static_cast<double>(a_k));
        if (all_inliers >= 1.0) {
            return a_options.min_iterations;
        }
        const double required = std::ceil(std::log(1.0 - a_options.confidence) / std::log(1.0 - all_inliers));
        if (!(required < static_cast<double>(a_options.max_iterations))) {
            return a_options.max_iterations;
        }
        return static_cast<size_t>(std::max(required, 0.0));
    }

    TEstimator estimator_;
};

} // namespace voyahslam::estimators

#endif // VOYAHSLAM_ESTIMATORS_RANSAC_H
