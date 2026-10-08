#include "voyahslam/geometry/pnp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <utility>

#include <Eigen/Dense>

namespace voyahslam::geometry {

namespace {

// World triangles whose angle sine at the first vertex is below this are treated as
// collinear: X = [d12 d13 d12×d13] is then (nearly) singular and the rotation undefined.
constexpr double kMinCollinearSine = 1e-6;
// A recovered R with ‖RᵀR - I‖ above this is numerical garbage (degenerate configuration).
constexpr double kMaxOrthonormalityError = 1e-6;
constexpr int kCubicNewtonIterations = 50;
constexpr int kMinCubicNewtonIterations = 7;
constexpr int kLambdaRefinementIterations = 5;

// Real roots of x² + b x + c = 0 without cancellation; false when they are complex.
bool SolveQuadratic(double b, double c, double& out_r1, double& out_r2) {
    const double discriminant = b * b - 4.0 * c;
    if (discriminant < 0.0) return false;
    const double y = std::sqrt(discriminant);
    if (b < 0.0) {
        out_r1 = 0.5 * (-b + y);
        out_r2 = 0.5 * (-b - y);
    } else {
        out_r1 = 2.0 * c / (-b + y);
        out_r2 = 2.0 * c / (-b - y);
    }
    return true;
}

// One real root of x³ + b x² + c x + d = 0, as the Lambda Twist reference: the initial guess
// is the extreme root of a second-order expansion at a stationary point (or the inflection
// point when the cubic is monotonic), then Newton-Raphson (at least 7 steps, then until the
// residual vanishes).
double SolveCubicRealRoot(double b, double c, double d) {
    const auto f = [&](double x) { return ((x + b) * x + c) * x + d; };
    double r;
    if (b * b >= 3.0 * c) {
        // Two stationary points t1 <= t2.
        const double v = std::sqrt(b * b - 3.0 * c);
        const double t1 = (-b - v) / 3.0;
        const double k1 = f(t1);
        if (k1 > 0.0) {
            // Leftmost root of k1 + (3 t1 + b)(x - t1)² = 0.
            r = t1 - std::sqrt(-k1 / (3.0 * t1 + b));
        } else {
            const double t2 = (-b + v) / 3.0;
            // Rightmost root of k2 + (3 t2 + b)(x - t2)² = 0.
            r = t2 + std::sqrt(-f(t2) / (3.0 * t2 + b));
        }
    } else {
        r = -b / 3.0;
        if (std::abs((3.0 * r + 2.0 * b) * r + c) < 1e-4) r += 1.0;
    }
    for (int i = 0; i < kCubicNewtonIterations; ++i) {
        const double fx = f(r);
        if (i >= kMinCubicNewtonIterations && std::abs(fx) <= std::numeric_limits<double>::epsilon()) break;
        const double dfx = (3.0 * r + 2.0 * b) * r + c;
        if (dfx == 0.0) break;
        r -= fx / dfx;
    }
    return r;
}

// Gauss-Newton on the three law-of-cosines residuals
//   r_ij = λi² + λj² + b_ij λi λj - a_ij,  b_ij = -2 yi·yj,  a_ij = ‖xi - xj‖²,
// keeping a step only if it lowers ‖r‖₁.
void RefineLambdas(double a12, double a13, double a23, double b12, double b13, double b23, TVector3DF& out_lambda) {
    const auto residual = [&](const TVector3DF& l) {
        return TVector3DF(l(0) * l(0) + l(1) * l(1) + b12 * l(0) * l(1) - a12,
                          l(0) * l(0) + l(2) * l(2) + b13 * l(0) * l(2) - a13,
                          l(1) * l(1) + l(2) * l(2) + b23 * l(1) * l(2) - a23);
    };
    TVector3DF r = residual(out_lambda);
    for (int i = 0; i < kLambdaRefinementIterations; ++i) {
        if (r.lpNorm<1>() < 1e-10) break;
        const TVector3DF& l = out_lambda;
        TMatrix33F J;
        J << 2.0 * l(0) + b12 * l(1), 2.0 * l(1) + b12 * l(0), 0.0,
             2.0 * l(0) + b13 * l(2), 0.0, 2.0 * l(2) + b13 * l(0),
             0.0, 2.0 * l(1) + b23 * l(2), 2.0 * l(2) + b23 * l(1);
        const Eigen::FullPivLU<TMatrix33F> lu(J);
        if (!lu.isInvertible()) break;
        const TVector3DF candidate = out_lambda - lu.solve(r);
        const TVector3DF candidate_r = residual(candidate);
        if (!(candidate_r.lpNorm<1>() < r.lpNorm<1>())) break;
        out_lambda = candidate;
        r = candidate_r;
    }
}

bool IsValidPose(const TMatrix33F& R, const TVector3DF& t) {
    return R.allFinite() && t.allFinite() &&
           (R.transpose() * R - TMatrix33F::Identity()).norm() < kMaxOrthonormalityError && R.determinant() > 0.0;
}

// Raw-pixel χ² of world point X under T_cw; +inf when it is not in front of the camera.
double PixelChi2(const Camera& camera, const RigidTransformation& T_cw, const TPoint3DF& X, const TPoint2DF& pixel,
                 double inv_sigma2) {
    const TPoint3DF Xc = T_cw.Transform(X);
    if (!(Xc.z() > 0.0)) return std::numeric_limits<double>::infinity();
    const double chi2 = (camera.Project(Xc) - pixel).squaredNorm() * inv_sigma2;
    return std::isnan(chi2) ? std::numeric_limits<double>::infinity() : chi2;
}

}  // namespace

std::vector<RigidTransformation> SolveP3P(const std::array<TPoint3DF, 3>& world,
                                          const std::array<TPoint3DF, 3>& bearings) {
    // Persson, Nordberg. Lambda Twist: An Accurate Fast Robust Perspective Three Point (P3P)
    // Solver. ECCV 2018. Notation follows the paper and its reference implementation.
    const TVector3DF y1 = bearings[0].normalized();
    const TVector3DF y2 = bearings[1].normalized();
    const TVector3DF y3 = bearings[2].normalized();
    const TPoint3DF& x1 = world[0];
    const TPoint3DF& x2 = world[1];
    const TPoint3DF& x3 = world[2];

    const TVector3DF d12 = x1 - x2;
    const TVector3DF d13 = x1 - x3;
    const TVector3DF d23 = x2 - x3;
    const TVector3DF d12xd13 = d12.cross(d13);
    if (!(d12xd13.norm() > kMinCollinearSine * d12.norm() * d13.norm())) return {};

    const double a12 = d12.squaredNorm();
    const double a13 = d13.squaredNorm();
    const double a23 = d23.squaredNorm();
    const double b12 = -2.0 * y1.dot(y2);
    const double b13 = -2.0 * y1.dot(y3);
    const double b23 = -2.0 * y2.dot(y3);

    // det(D1 + γ D2) = 0: the cubic p3 γ³ + p2 γ² + p1 γ + p0 (paper eq. 13).
    const double c12 = -0.5 * b12;
    const double c13 = -0.5 * b13;
    const double c23 = -0.5 * b23;
    const double blob = c12 * c23 * c13 - 1.0;
    const double s13_sq = 1.0 - c13 * c13;
    const double s23_sq = 1.0 - c23 * c23;
    const double s12_sq = 1.0 - c12 * c12;
    double p3 = a13 * (a23 * s13_sq - a13 * s23_sq);
    double p2 = 2.0 * blob * a23 * a13 + a13 * (2.0 * a12 + a13) * s23_sq + a23 * (a23 - a12) * s13_sq;
    double p1 = a23 * (a13 - a23) * s12_sq - a12 * a12 * s23_sq - 2.0 * a12 * (blob * a23 + a13 * s23_sq);
    double p0 = a12 * (a12 * s23_sq - a23 * s12_sq);

    // Divide by the larger of p3, p0; in the second case solve for 1/γ.
    double gamma;
    if (std::abs(p3) >= std::abs(p0)) {
        if (p3 == 0.0) return {};
        gamma = SolveCubicRealRoot(p2 / p3, p1 / p3, p0 / p3);
    } else {
        gamma = 1.0 / SolveCubicRealRoot(p1 / p0, p2 / p0, p3 / p0);
    }
    if (!std::isfinite(gamma)) return {};

    // D0 = D1 + γ D2 is a degenerate conic (one zero eigenvalue).
    TMatrix33F D0;
    D0(0, 0) = a23 * (1.0 - gamma);
    D0(0, 1) = 0.5 * a23 * b12;
    D0(0, 2) = -0.5 * a23 * b13 * gamma;
    D0(1, 1) = a23 - a12 + a13 * gamma;
    D0(1, 2) = 0.5 * b23 * (a13 * gamma - a12);
    D0(2, 2) = gamma * (a13 - a23) - a12;
    D0(1, 0) = D0(0, 1);
    D0(2, 0) = D0(0, 2);
    D0(2, 1) = D0(1, 2);
    if (!D0.allFinite()) return {};

    // The two nonzero eigenvalues σ0, σ1 (|σ0| >= |σ1|) and their eigenvectors.
    const Eigen::SelfAdjointEigenSolver<TMatrix33F> eigen(D0);
    if (eigen.info() != Eigen::Success) return {};
    std::array<int, 3> order{0, 1, 2};
    std::sort(order.begin(), order.end(), [&](int i, int j) {
        return std::abs(eigen.eigenvalues()(i)) > std::abs(eigen.eigenvalues()(j));
    });
    const double sigma0 = eigen.eigenvalues()(order[0]);
    const double sigma1 = eigen.eigenvalues()(order[1]);
    const TVector3DF u0 = eigen.eigenvectors().col(order[0]);
    const TVector3DF u1 = eigen.eigenvectors().col(order[1]);
    if (sigma0 == 0.0) return {};
    const double v = std::sqrt(std::max(0.0, -sigma1 / sigma0));

    // λ lies on the planes (u0 ∓ s u1)·λ = 0 with s = ±v; write λ1 = w0 λ2 + w1 λ3, then
    // λ3 = τ λ2 with τ a root of a quadratic, and λ2 from the a23 equation.
    std::vector<TVector3DF> lambdas;
    lambdas.reserve(4);
    for (const double s : {v, -v}) {
        const double w2 = 1.0 / (s * u1(0) - u0(0));
        const double w0 = (u0(1) - s * u1(1)) * w2;
        const double w1 = (u0(2) - s * u1(2)) * w2;
        const double a = 1.0 / ((a13 - a12) * w1 * w1 - a12 * b13 * w1 - a12);
        const double b = (a13 * b12 * w1 - a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13)) * a;
        const double c = ((a13 - a12) * w0 * w0 + a13 * b12 * w0 + a13) * a;
        double tau1 = 0.0;
        double tau2 = 0.0;
        if (!SolveQuadratic(b, c, tau1, tau2)) continue;
        for (const double tau : {tau1, tau2}) {
            if (!(tau > 0.0)) continue;
            const double d = a23 / (tau * (b23 + tau) + 1.0);
            if (!(d > 0.0)) continue;
            const double l2 = std::sqrt(d);
            const double l3 = tau * l2;
            const double l1 = w0 * l2 + w1 * l3;
            if (l1 >= 0.0) lambdas.emplace_back(l1, l2, l3);
        }
    }

    // R maps X = [d12 d13 d12×d13] onto the same construction from the scaled rays.
    const TMatrix33F X_inverse = (TMatrix33F() << d12, d13, d12xd13).finished().inverse();
    std::vector<RigidTransformation> poses;
    poses.reserve(lambdas.size());
    for (TVector3DF& lambda : lambdas) {
        RefineLambdas(a12, a13, a23, b12, b13, b23, lambda);
        const TVector3DF ry1 = lambda(0) * y1;
        const TVector3DF ry2 = lambda(1) * y2;
        const TVector3DF ry3 = lambda(2) * y3;
        const TVector3DF yd1 = ry1 - ry2;
        const TVector3DF yd2 = ry1 - ry3;
        const TMatrix33F Y = (TMatrix33F() << yd1, yd2, yd1.cross(yd2)).finished();
        const TMatrix33F R = Y * X_inverse;
        const TVector3DF t = ry1 - R * x1;
        if (IsValidPose(R, t)) poses.emplace_back(R, t);
    }
    return poses;
}

PnpResult SolvePnPRansac(const Camera& camera, const std::vector<TPoint3DF>& world,
                         const std::vector<TPoint2DF>& pixels, const std::vector<double>& sigmas,
                         const PnpOptions& options) {
    PnpResult result;
    const size_t n = world.size();
    if (pixels.size() != n || sigmas.size() != n || n < 4) return result;
    result.inlier.assign(n, false);

    std::vector<TPoint3DF> bearings(n);
    std::vector<double> inv_sigma2(n);
    for (size_t i = 0; i < n; ++i) {
        bearings[i] = camera.Unproject(pixels[i]).normalized();
        inv_sigma2[i] = 1.0 / (sigmas[i] * sigmas[i]);
    }

    std::mt19937_64 rng(options.seed);
    std::vector<size_t> indices(n);
    std::iota(indices.begin(), indices.end(), size_t{0});

    RigidTransformation best_T_cw;
    size_t best_inliers = 0;
    double best_chi2 = std::numeric_limits<double>::infinity();
    int required = options.max_iterations;
    while (result.iterations < required) {
        ++result.iterations;

        // Partial Fisher-Yates: the first 4 entries are a uniform distinct sample.
        for (size_t i = 0; i < 4; ++i) {
            std::uniform_int_distribution<size_t> pick(i, n - 1);
            std::swap(indices[i], indices[pick(rng)]);
        }
        const std::array<TPoint3DF, 3> sample_world{world[indices[0]], world[indices[1]], world[indices[2]]};
        const std::array<TPoint3DF, 3> sample_bearings{bearings[indices[0]], bearings[indices[1]],
                                                       bearings[indices[2]]};
        if (!(sample_bearings[0].allFinite() && sample_bearings[1].allFinite() && sample_bearings[2].allFinite())) {
            continue;
        }

        // The 4th point picks among the candidates.
        const size_t check = indices[3];
        const RigidTransformation* hypothesis = nullptr;
        double hypothesis_chi2 = std::numeric_limits<double>::infinity();
        const std::vector<RigidTransformation> candidates = SolveP3P(sample_world, sample_bearings);
        for (const RigidTransformation& T_cw : candidates) {
            const double chi2 = PixelChi2(camera, T_cw, world[check], pixels[check], inv_sigma2[check]);
            if (chi2 < hypothesis_chi2) {
                hypothesis_chi2 = chi2;
                hypothesis = &T_cw;
            }
        }
        if (hypothesis == nullptr) continue;

        size_t num_inliers = 0;
        double total_chi2 = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double chi2 = PixelChi2(camera, *hypothesis, world[i], pixels[i], inv_sigma2[i]);
            if (chi2 < options.chi2) {
                ++num_inliers;
                total_chi2 += chi2;
            }
        }
        if (num_inliers > best_inliers || (num_inliers == best_inliers && num_inliers > 0 && total_chi2 < best_chi2)) {
            best_T_cw = *hypothesis;
            best_inliers = num_inliers;
            best_chi2 = total_chi2;
            // log(1 - conf) / log(1 - w³). w = 1 gives 0 (stop); a w³ that rounds 1 - w³ to 1
            // gives +inf, and NaN fails the comparison: both keep the cap.
            const double w = static_cast<double>(best_inliers) / static_cast<double>(n);
            const double needed = std::ceil(std::log(1.0 - options.confidence) / std::log1p(-w * w * w));
            required = needed < static_cast<double>(options.max_iterations) ? static_cast<int>(std::max(needed, 0.0))
                                                                             : options.max_iterations;
        }
    }

    if (best_inliers < options.min_inliers) return result;
    result.success = true;
    result.T_cw = best_T_cw;
    result.num_inliers = best_inliers;
    for (size_t i = 0; i < n; ++i) {
        result.inlier[i] = PixelChi2(camera, best_T_cw, world[i], pixels[i], inv_sigma2[i]) < options.chi2;
    }
    return result;
}

}  // namespace voyahslam::geometry
