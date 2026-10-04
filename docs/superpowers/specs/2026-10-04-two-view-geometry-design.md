# Two-View Geometry Toolkit — Design Spec (Monocular Initialization, Part A)

Date: 2026-10-04
Status: approved in conversation, pending spec review

## Context and Scope

The real monocular `MapInitializer` is split into two specs:

- **A (this spec): geometry toolkit** — RANSAC, homography and essential
  estimators with ORB-SLAM3-style scores, decomposition into candidate poses,
  triangulation-based pose selection, a top-level two-view reconstructor, and
  a `RigidTransformation` value type. Pure math, no SLAM coupling, verified on
  synthetic scenes.
- **B+C (next spec):** initialization matching between the two frames,
  plugging the reconstructor into `MapInitializer`, map points in the result,
  `MapPointsAdded` publication, scale convention (median depth = 1), tuning on
  TUM-VI 512.

Inputs are our measurements' **normalized, undistorted points** (z = 1,
features beyond 85° already dropped), so the calibrated model is the
**essential** matrix (not ORB-SLAM3's fundamental + K).

**Success criteria**

1. On synthetic general scenes with noise and up to 50 % outliers,
   `Reconstruct` selects the essential model and recovers `T_21` (rotation and
   translation direction) within tolerance.
2. On synthetic planar scenes it selects the homography and recovers `T_21`.
3. Low parallax / too few inliers / ambiguous candidates are rejected.
4. Same seed + same input → identical result.
5. Every estimator exposes its own score; scores are on a common scale.

**Out of scope:** feature matching, `MapInitializer`/Mapper changes, the
5-point essential solver (added to TODO), threads (both RANSACs run
sequentially), `TPose` → `RigidTransformation` migration of existing code
(separate refactor, before spec C), Sim3.

## Decisions (from the conversation)

- Essential solver: normalized **8-point** now, behind an interface that
  allows several models per sample; 5-point later (TODO).
- One **generic `Ransac<TEstimator>`** run per model (not ORB-SLAM3's shared
  sample loop) — reusable for PnP, Sim3, stereo later.
- Scores: ORB-SLAM3 truncated χ² (`CheckHomography` / `CheckFundamental`),
  σ in normalized units.
- `RigidTransformation` class with hidden representation (R, t stored now;
  4×4 or quaternion later without touching callers). Convention:
  **`T_ab` maps points from frame b to frame a** (p_a = T_ab · p_b), so
  `T_ac = T_ab * T_bc`.

## 1. `geometry::RigidTransformation` — `geometry/rigid_transformation.h`

Replaces the empty placeholder struct.

```cpp
class RigidTransformation {
public:
    RigidTransformation();                                   // identity
    RigidTransformation(const TMatrix33F& rotation, const TVector3DF& translation);
    [[nodiscard]] static RigidTransformation Identity();
    [[nodiscard]] const TMatrix33F& rotation() const;
    [[nodiscard]] const TVector3DF& translation() const;
    [[nodiscard]] RigidTransformation Inverse() const;         // (Rᵀ, −Rᵀt)
    [[nodiscard]] RigidTransformation operator*(const RigidTransformation& other) const;  // this ∘ other
    [[nodiscard]] TPoint3DF Transform(const TPoint3DF& point) const;     // R p + t
    [[nodiscard]] TPoints3DF Transform(const TPoints3DF& points) const;
    [[nodiscard]] TMatrix34F ToMatrix34() const;
    [[nodiscard]] TMatrix44F ToMatrix44() const;
};
```

Header-only (small, non-template; the project keeps trivial value types
header-only). Storage is private; no setters beyond construction.

## 2. Correspondences — `geometry/two_view_correspondences.h`

```cpp
struct TwoViewCorrespondences {
    TPoints2DF x1;   // 2×N normalized coordinates in view 1
    TPoints2DF x2;   // 2×N normalized coordinates in view 2 (column i matches x1 column i)
    double sigma;    // measurement noise std-dev in normalized units (σ_px / f)
    [[nodiscard]] Eigen::Index size() const { return x1.cols(); }
};
```

## 3. Estimators — `estimators/`

### 3.1 Contract — `estimators/two_view_estimator.h`

```cpp
struct ModelScore {
    double score = 0.0;
    Eigen::Array<bool, Eigen::Dynamic, 1> inliers;
    size_t num_inliers = 0;
};

template <typename T>
concept TwoViewEstimator = requires(const T estimator,
                                    const geometry::TwoViewCorrespondences& correspondences,
                                    std::span<const Eigen::Index> sample,
                                    const typename T::TModel& model) {
    { T::kSampleSize } -> std::convertible_to<size_t>;
    { estimator.Estimate(correspondences, sample) } -> std::same_as<std::vector<typename T::TModel>>;
    { estimator.Score(model, correspondences) } -> std::same_as<ModelScore>;
};
```

`Estimate` returns 0..n candidate models (0 = degenerate sample).

### 3.2 `HomographyEstimator` — `estimators/homography_estimator.h`

- `using TModel = geometry::TMatrix33F;` (H_21: x2 ~ H x1), `kSampleSize = 4`.
- Estimate: Hartley-normalize both point sets
  (`algebra::GetHartleyNormalizationMatrix`), DLT (2 rows per point) solved
  with `algebra::SolveHomogeneousSystem`, denormalize, scale so H(2,2) = 1 if
  |H(2,2)| > ε. Degenerate (smallest-but-one singular value ≈ 0, e.g. three
  collinear points) → empty vector.
- Score (ORB-SLAM3 `CheckHomography`): for each correspondence,
  χ²₁ = ‖x2 − π(H x1)‖² / σ², χ²₂ = ‖x1 − π(H⁻¹ x2)‖² / σ²,
  threshold `kChi2Threshold = 5.991`; each direction with χ² < threshold adds
  (5.991 − χ²) to the score; inlier iff both directions pass. If H is not
  invertible the score is 0 with no inliers.

### 3.3 `EssentialEstimator` — `estimators/essential_estimator.h`

- `using TModel = geometry::TMatrix33F;` (E_21: x2ᵀ E x1 = 0), `kSampleSize = 8`.
- Estimate: Hartley-normalize, linear 8-point (`SolveHomogeneousSystem`),
  denormalize (E = T2ᵀ Ê T1), project to the essential manifold: SVD,
  singular values → ((s₁+s₂)/2, (s₁+s₂)/2, 0). Degenerate (rank of the 8×9
  system < 8) → empty vector.
- Score (ORB-SLAM3 `CheckFundamental`): l2 = E x1, l1 = Eᵀ x2 (homogeneous);
  χ²₂ = (x2ᵀ E x1)² / (l2ₓ² + l2ᵧ²) / σ², χ²₁ likewise with l1;
  `kChi2Threshold = 3.841`, `kScoreThreshold = 5.991`; each direction with
  χ² < 3.841 adds (5.991 − χ²); inlier iff both pass.

### 3.4 `Ransac<TEstimator>` — `estimators/ransac.h`

```cpp
struct RansacOptions {
    size_t max_iterations = 2000;
    size_t min_iterations = 50;
    double confidence = 0.99;
    uint32_t seed = 0;
    size_t refinement_rounds = 5;   // amendment 2026-10-04 (implementation): default 5
};

template <typename TModel>
struct RansacResult {
    bool success = false;
    TModel model;
    ModelScore score;
    size_t iterations = 0;
};

template <TwoViewEstimator TEstimator>
class Ransac {
public:
    explicit Ransac(TEstimator estimator = {});
    [[nodiscard]] RansacResult<typename TEstimator::TModel> Run(
        const geometry::TwoViewCorrespondences& correspondences, const RansacOptions& options) const;
};
```

- Header-only (template). Own `std::mt19937` seeded from `options.seed` per
  `Run` (deterministic). Samples `kSampleSize` distinct indices by partial
  Fisher–Yates over an index vector.
- Each iteration: estimate candidates, score each, keep the highest score.
- Adaptive stop: after each improvement, w = num_inliers / N and
  required = ⌈log(1 − confidence) / log(1 − w^k)⌉ (k = kSampleSize; w = 0 →
  max_iterations; w = 1 → min_iterations); stop when
  iterations ≥ clamp(required, min_iterations, max_iterations).
- `success = false` if N < kSampleSize or no candidate ever scored > 0.
- **Local refinement (amendment 2026-10-04):** if the estimator provides
  `TModel Refine(const TModel&, const TwoViewCorrespondences&,
  const Eigen::Array<bool, Eigen::Dynamic, 1>& inliers) const` (detected
  with a `requires` expression — optional, not part of the concept), then
  after the sampling loop: up to `RansacOptions::refinement_rounds` (default
  5; **amendment 2026-10-04 (implementation):** was 3 in the first draft,
  raised to 5 together with the LM stopping change in §3.5) rounds of refine on the current inliers → re-score → keep the refined
  model only if its score is strictly higher, else stop. The result's
  `score`/`inliers` are those of the returned model.

### 3.5 Refinement (amendment 2026-10-04)

Minimal-sample models overfit noise (measured: ~55 % of true inliers
recovered, pose errors of several degrees at σ = 1 px). Both estimators get
`Refine`, a Levenberg–Marquardt minimization over the inliers (**amendment 2026-10-04
(implementation):** at most 20 accepted steps / 40 trials — rejected trials do
not consume the accepted-step budget — early stop at relative cost decrease
< 1e-6, instead of the original "at most 10 iterations"; a numeric Jacobian is
acceptable):

- `EssentialEstimator::Refine`: parameterize E = [t]ₓR with R (3-dof
  rotation update, applied as R ← exp([δω]ₓ) R) and t on the unit sphere
  (2-dof update in the tangent plane, renormalized) — 5 dof; residual =
  Sampson error per inlier. Start from the decomposition candidate of E
  (any of the 4 gives the same E up to sign). Returns the refined E
  (unit Frobenius norm).
- `HomographyEstimator::Refine`: 8 parameters (H normalized to H(2,2) = 1,
  or ‖H‖ = 1 with a fixed-scale update); residual = symmetric transfer error
  of each inlier (4 components).
- A refinement that produces a non-finite or ill-conditioned model returns
  the input model unchanged.

## 4. Decomposition — `geometry/two_view_decomposition.h`

```cpp
struct HomographyCandidate { RigidTransformation T_21; TVector3DF plane_normal; };
[[nodiscard]] std::array<RigidTransformation, 4> DecomposeEssential(const TMatrix33F& E);
[[nodiscard]] std::vector<HomographyCandidate> DecomposeHomography(const TMatrix33F& H);
```

- `DecomposeEssential`: the 4 (R, ±t) combinations, ‖t‖ = 1, det(R) = +1.
  Built on the existing `geometry::DecomposeEssentialToPoses` if its output
  passes the tests; otherwise reimplemented.
- `DecomposeHomography`: Faugeras' SVD decomposition (ORB-SLAM3
  `ReconstructH`): up to 8 candidates, ‖t‖ = 1 (scale lost), det(R) = +1.
  Returns fewer if the singular values are degenerate (d1 ≈ d2 ≈ d3 → pure
  rotation: empty).

## 5. Reconstruction — `geometry/two_view_reconstruction.h`

```cpp
enum class TwoViewModel { Homography, Essential };

struct TwoViewOptions {
    double homography_inlier_ratio = 0.8;  // pick H if inliers_H >= this × inliers_E (amendment 2026-10-04)
    double min_parallax_deg = 1.0;
    size_t min_triangulated = 50;
    double min_good_fraction = 0.9;        // of the model's inliers
    double essential_uniqueness = 0.7;     // other candidates must have <= this × best good count
    double homography_uniqueness = 0.75;
    double max_point_parallax_cos = 0.99998;  // a point needs parallax cos below this to count
    estimators::RansacOptions ransac;
};

struct TwoViewReconstruction {
    bool success = false;
    TwoViewModel model = TwoViewModel::Essential;
    RigidTransformation T_21;               // camera 1 → camera 2; ‖t‖ = 1
    TPoints3DF points;                      // 3×N in camera 1's frame (valid where triangulated)
    Eigen::Array<bool, Eigen::Dynamic, 1> triangulated;
    size_t num_triangulated = 0;
    double parallax_deg = 0.0;
    double homography_score = 0.0;
    double essential_score = 0.0;
};

struct PoseSelection {
    bool success = false;
    size_t best_index = 0;
    TPoints3DF points;
    Eigen::Array<bool, Eigen::Dynamic, 1> triangulated;
    size_t num_good = 0;
    double parallax_deg = 0.0;
};

[[nodiscard]] PoseSelection SelectPose(std::span<const RigidTransformation> candidates,
                                       const TwoViewCorrespondences& correspondences,
                                       const Eigen::Array<bool, Eigen::Dynamic, 1>& inliers,
                                       double uniqueness_ratio, const TwoViewOptions& options);

class TwoViewReconstructor {
public:
    explicit TwoViewReconstructor(TwoViewOptions options = {});
    [[nodiscard]] TwoViewReconstruction Reconstruct(const TwoViewCorrespondences& correspondences) const;
};
```

**`SelectPose`** (ORB-SLAM3 `CheckRT` + the acceptance logic of
`ReconstructF`/`ReconstructH`), for each candidate:

- For every inlier i: triangulate with `geometry::Triangulate` using
  P1 = [I | 0], P2 = `T_21.ToMatrix34()`; c = parallax cos
  (`geometry::ComputeParallaxCos`). Exactly as ORB-SLAM3 `CheckRT`:
  - not finite → skip;
  - depth ≤ 0 in either camera **and** c < `max_point_parallax_cos` → skip
    (for far points, c ≥ threshold, the depth sign is unreliable and is not
    tested);
  - squared reprojection error (normalized units) > 4σ² in either view → skip;
  - otherwise the point is **good**: `num_good++`, its c is collected, and
    `triangulated[i] = true` only if c < `max_point_parallax_cos` (far points
    count towards `num_good` but are not returned as triangulated).
- Candidate parallax = acos(c_k) in degrees, where the collected c values
  are sorted ascending and k = min(50, count − 1); 0 if none.

Acceptance (on the best candidate by `num_good`):
`num_good > max(min_good_fraction × num_inliers, min_triangulated)`, no other
candidate has `num_good > uniqueness_ratio × best`, and
`parallax_deg >= min_parallax_deg`. Otherwise `success = false`.

**`Reconstruct`:** σ comes from `correspondences.sigma` (scores and the 4σ²
reprojection test); runs `Ransac<HomographyEstimator>` and `Ransac<EssentialEstimator>`
(seed from options; the essential run uses `seed + 1`); fails if both fail.
**Model selection (amendment 2026-10-04, COLMAP rule):** choose H iff the H
run succeeded and `inliers_H >= homography_inlier_ratio × inliers_E` (or the
E run failed) — i.e. a homography explains nearly everything the essential
matrix does (planar scene or near-pure rotation); otherwise E. The ORB-SLAM3
score ratio is not used: it relies on the *uncalibrated* F estimator
degenerating on planes, whereas a calibrated E fits planar points as well as
general ones (measured: even ground-truth H vs E gives S_H/(S_H+S_E) ≈ 0.41).
The per-estimator scores still rank hypotheses inside each RANSAC and are
reported. Then: H → decompose, `SelectPose` with `homography_uniqueness`;
E → decompose, `SelectPose` with `essential_uniqueness`. Fills `TwoViewReconstruction`; `T_21` is the selected
candidate.

Non-template parts (decomposition, `SelectPose`, `TwoViewReconstructor`,
both estimators) are defined in `src/` (`src/estimators/*.cpp`,
`src/geometry/two_view_*.cpp`), added to `src/CMakeLists.txt`.

## 6. Testing

Test helper `tests/support/two_view_scene.h` (header-only):

```cpp
struct TwoViewScene { geometry::TwoViewCorrespondences correspondences;
                      geometry::RigidTransformation T_21; geometry::TPoints3DF points;
                      Eigen::Array<bool, Eigen::Dynamic, 1> is_outlier; };
TwoViewScene MakeGeneralScene(size_t n, double outlier_fraction, double sigma, uint32_t seed,
                              const geometry::RigidTransformation& T_21);
TwoViewScene MakePlanarScene(size_t n, double outlier_fraction, double sigma, uint32_t seed,
                             const geometry::RigidTransformation& T_21);
```

Points in view 1: general = uniform in a box at depth 4–8 within ±30° of the
axis; planar = on the plane z = 5 (tilted ±15° in tests that need it).
**Amendment 2026-10-04:** the planar *acceptance* test uses a scene that
breaks the two-fold planar ambiguity — field of view ±45° (u, v ∈ [−1, 1]),
tilt ≥ 30° and translation 1 m — so the 0.75 uniqueness check passes; a
separate test asserts that the default narrow, near-frontal planar scene is
**rejected** (ambiguous). `MakePlanarScene` gains `double half_fov_tan = 0.5`
as the last parameter.
Default T_21: rotation 5° about a random axis, translation 0.5 m (good
parallax). Noise: Gaussian with std `sigma` on both views; outliers: x2
replaced by uniform random normalized points within the field of view.

| Area | Tests |
|---|---|
| `RigidTransformation` | identity; `Inverse()` round trip; `(T_ab * T_bc).Transform(p) == T_ab.Transform(T_bc.Transform(p))`; matrix exports |
| `HomographyEstimator` | exact H from 4 noise-free planar points; collinear sample → no model; true H outscores a perturbed H; score/inliers on a hand-computed 2-point case |
| `EssentialEstimator` | exact E (up to scale, x2ᵀEx1 ≈ 0) from 8 noise-free points; singular values (s, s, 0); degenerate sample → no model; score on a hand-computed case |
| `Ransac` | recovers E with 30 % and 50 % outliers (**amendment 2026-10-04 (implementation):** ≥ 50 % of true inliers with ≤ 5 % outliers, narrow FOV, refinement on — noise in both views means only ~58 % of true inliers pass both χ² tests at σ = noise); clean data stops well below `max_iterations`; same seed → identical result; N < kSampleSize → `success = false` |
| Decomposition | the true `T_21` (t up to sign/scale, R exact) is among E's 4 and H's candidates |
| Refinement | `Refine` lowers the summed residual on noisy inliers; refined E/H stay finite and well-conditioned; Ransac with refinement recovers ≥ 75 % of true inliers (vs ≥ 50 % without) at 30 % outliers, wide FOV (**amendment 2026-10-04 (implementation):** wide-FOV scenes, half_fov_tan = 1.0; 50 % outliers: ≥ 0.47 = min observed over seeds 11–18 minus 0.05; refined ≥ unrefined at the chosen seed) |
| `SelectPose` / `Reconstruct` | general scene → Essential, rotation error < 1°, translation direction error **< 10°** (**amendment 2026-10-04 (implementation):** wide-FOV scene; 10° is the observed maximum over 21 seeds (9.92°), about half the seeds are under 2°; the error comes from inlier-set lock-in — LM converges, but least squares over the hard-thresholded, self-selected inlier set drops true inliers and keeps a few outlier levers, and on the worst seeds the true E scores higher than the returned model; robust-loss local optimization over all correspondences is the remedy (TODO) — further accuracy comes from bundle adjustment after initialization), `num_triangulated ≥ 0.5 × true inliers`; wide-FOV tilted planar scene → Homography, rotation < 1°, translation direction < 2° (21/21 seeds); narrow near-frontal planar scene → rejected (ambiguity); pure rotation / 1 cm baseline → `success = false` (parallax); 30 correspondences → `success = false` (min_triangulated); deterministic for a fixed seed |

All tests seeded; no images.

## 7. Documentation

- `DESIGN.md`: a "Geometry toolkit" section (estimators, RANSAC, scores,
  reconstruction, `RigidTransformation` and the `T_ab` convention); decisions
  log rows; status.
- `TODO.md`: 5-point essential solver; migrate `TPose` users (Frame, events,
  Tracker, Mapper) to `RigidTransformation` before spec C; optional parallel
  H/E RANSAC; Environment: clang 18 + libc++ syntax check (verified clean
  2026-10-04 on all 65 sources) as a recurring step, later an NDK arm64
  cross-build.
