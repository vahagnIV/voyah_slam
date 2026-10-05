# Bundle Adjustment After Monocular Initialization (g2o) — Design Spec

Date: 2026-10-05
Status: approved in conversation, pending spec review

## Context and Scope

Monocular initialization (spec `2026-10-04-mono-initialization-design.md`)
returns the two-view reconstruction directly. The two-view toolkit has a known
accuracy limit: inlier-set lock-in leaves a 1–10° translation-direction error
on some scenes (TODO). ORB-SLAM3 runs a global bundle adjustment right after
monocular initialization and then rescales the map by median depth. This spec
adds that step, using g2o.

**In scope**

1. A vendored, pristine upstream g2o (`20241228_git`) under `third_party/g2o/`,
   with our own Conan 2 recipe.
2. A README describing the full build, including the one-time
   `conan create` of g2o.
3. An `optimization::BundleAdjust` component with plain Eigen types in its
   public API. g2o stays private.
4. Bundle adjustment inside mono `MapInitializer::TryInitialize`, followed by
   outlier removal, median-depth rescaling and a minimum-points check.
5. Tests, verification and documentation.

**Out of scope (→ TODO.md)**

- Bundle adjustment for stereo initialization. With one keyframe there is
  nothing to adjust.
- Local and global bundle adjustment during mapping. They will reuse
  `BundleAdjust`.
- A pixel-space (distorted) fisheye residual.
- A g2o build with a clang/libc++ or Android NDK profile.

**Success criteria**

1. Synthetic sequence, seeds 1–5: the bundle adjustment reaches the ML
   optimum — its translation-direction error is within 0.1° of an oracle (the
   same adjustment started from the ground truth) on every seed, its χ² is
   below the two-view estimate's, and the mean direction error over the seeds
   is ≤ the mean without it. The map is still median-depth-1 scaled.
   *Amended during implementation (Ruling O): init error at low parallax is
   noise-limited; tests check convergence to the ML optimum. BA runs on the
   unscaled problem (baseline 1, depths ~40), so it needs ~40 plain iterations to
   converge; rescaling first would converge faster (left for tuning).*
2. `BundleAdjust` recovers a perturbed noise-free two-view problem exactly, up
   to the free scale, and flags gross outliers.
3. A clean machine can follow `README.md` to build g2o through Conan, then the
   project, and pass all tests.
4. `monocular_full_slam` on corridor1_1024 still initializes. The log shows the
   cost reduction and the outliers that were removed.

## Decisions (from the conversation)

- **g2o source:** a local Conan recipe plus pristine vendored upstream source
  in `third_party/g2o/`. Upgrades replace `src/` and bump the version. There
  is no patch set.
- **Encapsulation:** g2o is linked PRIVATE and no g2o type appears in a public
  header, the same rule as for OpenCV and Pangolin.
- **Residual:** on the undistorted normalized plane (z = 1). The Jacobian has
  no K and no lens model. σ is `sigma_px · level_scale^level / f`, the same
  simplification as the stereo matcher.
- **Scope:** mono initialization only. ORB-SLAM3 settings: two stages like
  its local BA (5 Huber iterations, deactivate outliers, 10 plain iterations), first keyframe fixed, bundle adjustment before rescaling, and a
  reset if fewer than 50 points survive.

## 1. Vendored g2o, recipe, build

```
third_party/
  README.md        upstream URL, tag 20241228_git, commit eec325a1, license, update procedure
  g2o/
    conanfile.py   our recipe
    src/           pristine upstream source of tag 20241228_git (incl. LICENSE)
```

**Recipe** (`third_party/g2o/conanfile.py`, Conan 2):

- **Identity and inputs.** `name = "g2o"`, `version = "20241228"`; created as
  `g2o/20241228@voyah/stable`. `exports_sources = "src/*"`;
  `requires = "eigen/3.4.0"`. Settings are `os`, `compiler`, `build_type` and
  `arch`. Options are `shared` (default False) and `fPIC` (default True).
- **Build.** `CMakeToolchain` and `CMakeDeps`, configured on `src/`. Upstream
  options:
  - `BUILD_SHARED_LIBS=OFF`, `BUILD_TESTING=OFF`.
  - `G2O_USE_CHOLMOD=OFF`, `G2O_USE_CSPARSE=OFF`, `G2O_USE_OPENGL=OFF`,
    `G2O_USE_LOGGING=OFF`.
  - `G2O_BUILD_APPS=OFF`, `G2O_BUILD_EXAMPLES=OFF`,
    `G2O_BUILD_BENCHMARKS=OFF`.

  These names are verified against the tag's `CMakeLists.txt`, and deviations
  are recorded in `third_party/README.md`.
- **`package_info`.** Exposes CMake targets for at least `g2o::core`,
  `g2o::stuff`, `g2o::types_sba`, `g2o::types_slam3d` and
  `g2o::solver_eigen`, as components with the correct inter-dependencies and
  link order. Upstream's own installed CMake config is not used.

**Project:**

- `conanfile.txt` requires `g2o/20241228@voyah/stable`.
- `src/CMakeLists.txt` calls `find_package(g2o REQUIRED)` and links
  `voyahslam` PRIVATE against the targets it uses.

**README.md:**

- Prerequisites: gcc 13 or clang 18, CMake, Conan 2.
- OpenCV 5.1 and Pangolin 0.9.3, built from source into `/usr/local`, with the
  configure flags used.
- One-time g2o packaging:
  `conan create third_party/g2o --build=missing -s build_type=Debug` and the
  same for `Release`.
- The project build: Conan install and CMake for Debug
  (`cmake-build-debug/build/Debug`) and Release (`build/build/Release`), then
  `ctest`.
- Running the apps: dataset path, `--rate`, `--max-frames`, `--show`,
  `--viewer`.

## 2. `optimization::BundleAdjust`

Public header `include/voyahslam/optimization/bundle_adjustment.h`, free of
g2o:

```cpp
namespace voyahslam::optimization {

struct BundleAdjustmentOptions {
    int robust_iterations = 5;    // stage 1, Huber kernel
    int iterations = 10;          // stage 2, no kernel, outlier edges deactivated
    bool robust = true;           // false: one plain pass of robust_iterations + iterations
    double huber_chi2 = 5.991;    // δ² of the kernel and the outlier test (2 dof, 95 %)
};

struct BaObservation {
    size_t keyframe;                    // index into keyframe_poses
    size_t point;                       // column of points
    geometry::TPoint2DF measurement;    // undistorted normalized coordinates (z = 1)
    double sigma;                       // normalized units
};

struct BaProblem {
    std::vector<geometry::RigidTransformation> keyframe_poses;   // T_cw
    std::vector<bool> keyframe_fixed;
    geometry::TPoints3DF points;                                 // world frame
    std::vector<BaObservation> observations;
};

struct BaResult {
    bool success = false;
    std::vector<geometry::RigidTransformation> keyframe_poses;
    geometry::TPoints3DF points;
    std::vector<double> chi2;      // per observation (deactivated ones too), final, without kernel
    std::vector<bool> inlier;      // chi2 <= huber_chi2 && camera-frame depth > 0
    double initial_cost = 0.0;     // plain chi2 sum over all observations at the start
    double final_cost = 0.0;       // plain chi2 sum over the edges active in the last pass
    int iterations = 0;            // total over both stages
};

[[nodiscard]] BaResult BundleAdjust(const BaProblem& problem,
                                    const BundleAdjustmentOptions& options);
}
```

**Residual and Jacobians** (`src/optimization/projection.h`, internal, no
g2o):

- Camera-frame point: `Xc = R·X + t` (from `T_cw`).
- Projection: `π(Xc) = (x/z, y/z)`. Residual: `e = π(Xc) − measurement`.
  Information: `I / sigma²`.
- `∂π/∂Xc = [[1/z, 0, −x/z²], [0, 1/z, −y/z²]]`.
- Pose block: `∂π/∂Xc · [−[Xc]× | I]`, in g2o `SE3Expmap` update order
  (rotation, then translation).
- Point block: `∂π/∂Xc · R`.
- Exposed as `geometry::TPoint2DF ProjectNormalized(const RigidTransformation&
  T_cw, const TPoint3DF& X)` and
  `void ProjectionJacobians(const RigidTransformation& T_cw, const TPoint3DF& X,
  Eigen::Matrix<precision_t,2,6>& out_pose, Eigen::Matrix<precision_t,2,3>&
  out_point)`.

**Implementation** (`src/optimization/bundle_adjustment.cpp`, the only g2o
user):

- **Graph.** `VertexSE3Expmap` per keyframe (fixed if `keyframe_fixed`);
  `VertexPointXYZ` per point, marginalized. A custom edge
  `EdgeNormalizedProjection` (`BaseBinaryEdge<2, Vector2, VertexPointXYZ,
  VertexSE3Expmap>`) uses the helper above for `computeError` and
  `linearizeOplus`.
- **Solver.** `BlockSolver_6_3` + `LinearSolverEigen` +
  `OptimizationAlgorithmLevenberg`.
- **Two stages** (like ORB-SLAM3's `LocalBundleAdjustment`). Stage 1:
  `robust_iterations` with the Huber kernel, δ = √`huber_chi2`. Then every edge
  with plain χ² > `huber_chi2` or camera-frame depth ≤ 0 is deactivated
  (`setLevel(1)`). Stage 2: `iterations` on the remaining level-0 edges with
  no kernel. With `robust == false`: a single plain pass of
  `robust_iterations + iterations`, nothing deactivated.
  *Amended during implementation (Ruling N): two-stage like ORB-SLAM3 LocalBA;
  a single robust pass drifted with gross outliers.*
- **Empty problem.** No keyframes, no points or no observations →
  `success = false`. An optimizer that fails to initialize or to run →
  `success = false`.
- **After optimization.** Read back the poses and points. For every edge,
  deactivated ones included, compute the plain χ² and the camera-frame depth
  at the final estimate, then fill `chi2` and `inlier`. `initial_cost` is the
  plain χ² sum over all edges at the start; `final_cost` the plain χ² sum over
  the edges active in the last pass. `iterations` is the total performed.

## 3. Integration into mono initialization

`MapInitializerOptions` gains:

```cpp
bool bundle_adjust = true;
optimization::BundleAdjustmentOptions bundle_adjustment{.robust_iterations = 5,
                                                        .iterations = 40};
double level_scale = 1.2;          // σ_level = sigma_px · level_scale^level
size_t min_points_after_ba = 50;
```

*Amended during implementation (Ruling O): init error at low parallax is
noise-limited; tests check convergence to the ML optimum. BA runs on the
unscaled problem (baseline 1, depths ~40), so it needs ~40 plain iterations to
converge; rescaling first would converge faster (left for tuning).*

Mono `TryInitialize`, after `TwoViewReconstructor` succeeds and before the
current scaling step. When `bundle_adjust == false`, all of this is skipped
and behaviour is exactly as today.

1. **Build `BaProblem`.**
   - Keyframes: keyframe 0 = Identity, fixed; keyframe 1 = unscaled `T_21`,
     free.
   - Points: the triangulated points (camera-1 frame).
   - Two observations per point. `measurement` = the matched feature's
     normalized point (first two rows). `sigma = sigma_px ·
     level_scale^level / K(0,0)` of that view.
2. **`BundleAdjust`** (the same two-stage call as §2, with
   `bundle_adjustment` options). If `success == false`, treat it as a reconstruction
   failure: keep the reference and return nullopt.
3. **Drop outliers.** Drop every point with a non-inlier observation in either
   view.
4. **Rescale.** Take the median depth of the survivors in camera 1. If it is
   non-finite or ≤ 0, treat it as failure. Scale the points and `t` by
   1/median.
5. **Minimum points.** Fewer than `min_points_after_ba` survivors → failure:
   keep the reference and return nullopt.
6. **Log.** A debug-level line: initial and final cost, iterations, outliers
   removed, survivors.

The result types, the Mapper, stereo initialization and the viewer are
unchanged.

## 4. Testing, verification, docs

| Area | Tests |
|---|---|
| Projection helper | `ProjectNormalized` is correct at known points. `ProjectionJacobians` equals central finite differences (pose via the left-multiplied exponential map in g2o order, point directly), within 1e-6, at 10 random (pose, point) pairs. The test target adds `src/` as a private include directory. |
| `BundleAdjust`, clean | Two-view scene from `tests/support/two_view_scene.h`, noise-free normalized measurements. Keyframe 1 is perturbed (2° rotation, 10 % translation) and the points by 5 %. After adjustment, with poses and points aligned by the scale `s = ‖t_true‖/‖t_est‖`: rotation error < 1e-6 rad, translation within 1e-6, points within 1e-6. The fixed keyframe is unchanged (exact). |
| `BundleAdjust`, noise + outliers | 0.5 px noise (in normalized units via f), plus 10 % of observations replaced by random positions. ≥ 95 % of the corrupted points have an observation with `inlier == false`. Oracle: the same call (same start, same options) on the inlier observations only; translation-direction error ≤ oracle error + 0.5° and < 2°. `final_cost < initial_cost`. (Amended, Ruling N: the absolute 0.5° bound was below the ML error of this geometry.) |
| `BundleAdjust`, edges | An empty problem → `success == false`. A point seen only by the fixed keyframe stays finite. `robust = false` changes `final_cost` relative to the robust run on the outlier scene. |
| `MapInitializer` | Synthetic sequence, seeds 1–5, `bundle_adjust` on vs off (same initializing frame). Per seed: direction error within 0.1° of the oracle (the same `BundleAdjust` problem and options started from the ground truth, unscaled baseline 1); χ² of the BA estimate < χ² of the two-view estimate on the same observations. Mean direction error with ≤ without. (Amended, Ruling O: low-parallax error is noise-limited; the test checks convergence to the ML optimum.) Median depth == 1 (1e-9). Points ≥ `min_points_after_ba`. With `min_points_after_ba = 100000` there is no initialization (frames 0–10). Existing initializer, Mapper and Slam tests stay green with the default (on). |
| ThreadSanitizer | `Mapper*:Slam*`. |
| Clean-machine build | Follow `README.md` with a fresh `CONAN_HOME` (scratch directory): `conan create third_party/g2o` (Debug and Release), the project build, `ctest`. Record it in the report. |
| Smoke runs | `monocular_full_slam` on corridor1_1024 `--rate=0 --max-frames=200`: init frame, points before/after the adjustment, cost reduction and outliers from the debug line (run with debug logging enabled if needed), versus the baseline (~frame 121, 73 points). `binocular_full_slam --max-frames=20` unchanged (355 points). |
| clang-18 + libc++ | Syntax check of our sources; `bundle_adjustment.h` and `projection.h` included. |

**Docs**

- `DESIGN.md`:
  - The optimization component: g2o private behind Eigen types, the
    normalized-plane residual and its Jacobian.
  - The bundle adjustment in mono initialization: order (adjust → drop
    outliers → rescale → minimum points).
  - The vendored g2o and its recipe.
  - Decisions-log rows dated 2026-10-05.
- `TODO.md`:
  - Remove "bundle adjustment after initialization".
  - Add: g2o with a clang/libc++ and NDK profile; local and global bundle
    adjustment reusing `BundleAdjust`.
  - Merge the pixel-space residual item so it covers both the stereo matcher
    and the bundle adjustment.
- `third_party/README.md`: the upstream URL, tag, commit, license and update
  procedure.
- `README.md`: as in section 1.
