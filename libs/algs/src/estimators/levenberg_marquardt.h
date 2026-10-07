#ifndef VOYAHSLAM_SRC_ESTIMATORS_LEVENBERG_MARQUARDT_H
#define VOYAHSLAM_SRC_ESTIMATORS_LEVENBERG_MARQUARDT_H

#include <algorithm>

#include <Eigen/Dense>

namespace voyahslam::estimators::internal {

inline constexpr int kMaxLmAcceptedSteps = 20;
inline constexpr int kMaxLmTrials = 40;  ///< Accepted plus rejected steps.
inline constexpr double kMinRelativeDecrease = 1e-6;
inline constexpr double kInitialLambda = 1e-3;
inline constexpr double kJacobianStep = 1e-6;

/// Levenberg-Marquardt minimization of the summed squared residuals over a kDof-dimensional
/// local parameterization of a model (numeric central-difference Jacobian).
///
/// a_residuals(delta, out_r): residuals of the current model moved by delta (delta = 0 is the
///   current model); returns false when that model is invalid (e.g. non-finite).
/// a_update(delta): moves the current model by delta (called only for accepted steps).
///
/// Stops after kMaxLmAcceptedSteps accepted steps or kMaxLmTrials attempted steps (accepted plus
/// rejected), whichever comes first, or early once an accepted step decreases the cost by less
/// than kMinRelativeDecrease (relative). Returns false if the starting model
/// is invalid (no update was made).
template <int kDof, typename TResiduals, typename TUpdate>
bool MinimizeLevenbergMarquardt(const TResiduals& a_residuals, const TUpdate& a_update) {
    using TParams = Eigen::Matrix<double, kDof, 1>;
    using TNormal = Eigen::Matrix<double, kDof, kDof>;

    Eigen::VectorXd r;
    if (!a_residuals(TParams::Zero().eval(), r)) return false;
    double cost = r.squaredNorm();

    double lambda = kInitialLambda;
    bool linearize = true;
    TNormal normal;
    TParams gradient;
    Eigen::VectorXd r_plus, r_minus, r_new;
    int accepted = 0;
    for (int trial = 0; trial < kMaxLmTrials && accepted < kMaxLmAcceptedSteps && cost > 0.0; ++trial) {
        if (linearize) {
            Eigen::MatrixXd J(r.size(), kDof);
            for (int k = 0; k < kDof; ++k) {
                const TParams step = TParams::Unit(k) * kJacobianStep;
                if (!a_residuals(step, r_plus) || !a_residuals((-step).eval(), r_minus)) return true;
                J.col(k) = (r_plus - r_minus) / (2.0 * kJacobianStep);
            }
            normal = J.transpose() * J;
            gradient = J.transpose() * r;
            linearize = false;
        }

        // Marquardt scaling; the floor keeps directions with a zero diagonal damped.
        const double floor = 1e-12 * std::max(1.0, normal.diagonal().maxCoeff());
        TNormal damped = normal;
        damped.diagonal() += lambda * normal.diagonal().cwiseMax(floor);
        const TParams delta = damped.ldlt().solve(-gradient);

        if (delta.allFinite() && a_residuals(delta, r_new) && r_new.squaredNorm() < cost) {
            const double new_cost = r_new.squaredNorm();
            const double relative_decrease = (cost - new_cost) / cost;
            a_update(delta);
            ++accepted;
            cost = new_cost;
            r.swap(r_new);
            lambda *= 0.1;
            linearize = true;
            if (relative_decrease < kMinRelativeDecrease) break;
        } else {
            lambda *= 10.0;
        }
    }
    return true;
}

} // namespace voyahslam::estimators::internal

#endif // VOYAHSLAM_SRC_ESTIMATORS_LEVENBERG_MARQUARDT_H
