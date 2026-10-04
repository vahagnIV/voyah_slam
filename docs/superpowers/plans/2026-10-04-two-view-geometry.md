# Two-View Geometry Toolkit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A tested two-view geometry toolkit — `RigidTransformation`, generic adaptive `Ransac`, homography and essential estimators with ORB-SLAM3 χ² scores, E/H decomposition, triangulation-based pose selection and a `TwoViewReconstructor` — verified on synthetic scenes.

**Architecture:** Estimators satisfy a `TwoViewEstimator` concept (`kSampleSize`, `Estimate` → 0..n models, `Score` → `ModelScore`); `Ransac<TEstimator>` is header-only and seeded; decomposition and `SelectPose` turn the winning model into `T_21` + points following ORB-SLAM3 `CheckRT`; `TwoViewReconstructor` runs both RANSACs and picks H or E by score ratio.

**Tech Stack:** C++20 (concepts, `std::span`), Eigen, GTest.

**Spec:** `docs/superpowers/specs/2026-10-04-two-view-geometry-design.md`

## Global Constraints

- Branch `feat/two-view-geometry`. Conventions: CLAUDE.md, DESIGN.md §7 (PascalCase functions/types, `snake_case_` members, `out_` prefix for non-const reference params, `[[nodiscard]]` const accessors, include guards `VOYAHSLAM_<PATH>_H`, self-contained headers). No OpenCV anywhere in this feature. No virtual functions.
- Namespaces: `voyahslam::geometry` (RigidTransformation, TwoViewCorrespondences, decomposition, reconstruction), `voyahslam::estimators` (ModelScore, concept, estimators, Ransac). Headers under `include/voyahslam/{geometry,estimators}/`, sources under `src/{geometry,estimators}/` (added to `src/CMakeLists.txt`), tests under `tests/{geometry,estimators}/` (added to `tests/CMakeLists.txt`).
- Transform convention: `T_ab` maps points from frame b to frame a (p_a = T_ab · p_b); `T_ac = T_ab * T_bc`. Two-view result `T_21`: camera 1 → camera 2; x2 ~ R x1 + t.
- Points are normalized image coordinates (z = 1). σ is in normalized units (`TwoViewCorrespondences::sigma`).
- Values: H `kSampleSize = 4`, H χ² threshold 5.991 (score adds 5.991 − χ²); E `kSampleSize = 8`, E χ² threshold 3.841, score adds 5.991 − χ²; both: inlier iff both directions pass. RansacOptions defaults `max_iterations = 2000`, `min_iterations = 50`, `confidence = 0.99`, `seed = 0`, `refinement_rounds = 3` (amendment); essential run uses `seed + 1`. TwoViewOptions defaults: `homography_inlier_ratio = 0.8` (amendment: H iff inliers_H ≥ 0.8 × inliers_E), `min_parallax_deg = 1.0`, `min_triangulated = 50`, `min_good_fraction = 0.9`, `essential_uniqueness = 0.7`, `homography_uniqueness = 0.75`, `max_point_parallax_cos = 0.99998`; reprojection gate 4σ².
- Build/test: `cmake -S . -B cmake-build-debug/build/Debug && cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 60`; focused: `cmake-build-debug/build/Debug/tests/voyahslam_tests --gtest_filter='<Suite>*'`. All existing tests stay green. Every new header must also pass `clang++-18 -std=c++20 -stdlib=libc++ -fsyntax-only` (use the include/define flags from `cmake-build-debug/build/Debug/tests/CMakeFiles/voyahslam_tests.dir/flags.make`).

## Review Focus

1. Projective division by ~0 (H maps a point to infinity; epipolar line with zero normal) → that correspondence is an outlier, score stays finite (Task 3 `HomographyScoreFiniteForPointMappedToInfinity`, Task 4 `EssentialScoreFiniteForZeroEpipolarLine`).
2. Degenerate samples: all sample points identical (Hartley scale 0) → `Estimate` returns empty, no NaN models (Task 3 `IdenticalPointsGiveNoModel`, Task 4 same name).
3. Parallel rays / identical correspondences in triangulation → skipped, never counted good (Task 7 `IdenticalCorrespondencesAreNotTriangulated`).
4. Empty or tiny inputs: 0 correspondences, exactly `kSampleSize` correspondences (Task 5 `EmptyInputFails`, `ExactlySampleSizeRuns`).
5. Non-positive σ → `Reconstruct` returns `success = false` without running RANSAC (Task 7 `NonPositiveSigmaFails`).

---

### Task 1: RigidTransformation

**Files:**
- Modify (replace placeholder): `include/voyahslam/geometry/rigid_transformation.h`
- Test: `tests/geometry/rigid_transformation_test.cpp`

**Interfaces:**
- Produces: `geometry::RigidTransformation` exactly as spec §1 (ctor default = identity; ctor `(const TMatrix33F& rotation, const TVector3DF& translation)`; `Identity()`, `rotation()`, `translation()`, `Inverse()`, `operator*`, `Transform(TPoint3DF)`, `Transform(TPoints3DF)`, `ToMatrix34()`, `ToMatrix44()`). Header-only.

- [ ] **Step 1: Failing tests:** `RigidTransformationTest.DefaultIsIdentity` (rotation == I, translation == 0); `.InverseRoundTrip` (T * T.Inverse() ≈ identity, 1e-12); `.CompositionOrder` (for random T_ab, T_bc and point p: `(T_ab * T_bc).Transform(p) ≈ T_ab.Transform(T_bc.Transform(p))`); `.TransformPointsMatchesColumns`; `.MatrixExports` (`ToMatrix34() == [R|t]`, `ToMatrix44()` bottom row `0 0 0 1`).
- [ ] **Step 2: Run** — expected: compile failure (members missing).
- [ ] **Step 3: Implement** per spec §1; private `rotation_`, `translation_`.
- [ ] **Step 4: Run** focused + full suite — pass.
- [ ] **Step 5: Commit** `Add geometry::RigidTransformation (T_ab convention)`.

### Task 2: Correspondences, estimator contract, synthetic scene helper

**Files:**
- Create: `include/voyahslam/geometry/two_view_correspondences.h`, `include/voyahslam/estimators/two_view_estimator.h`, `tests/support/two_view_scene.h`
- Test: `tests/geometry/two_view_scene_test.cpp`

**Interfaces:**
- Consumes: `RigidTransformation` (Task 1).
- Produces: `geometry::TwoViewCorrespondences{ TPoints2DF x1, x2; double sigma; size() }`; `estimators::ModelScore{ double score; Eigen::Array<bool, Eigen::Dynamic, 1> inliers; size_t num_inliers; }`; concept `estimators::TwoViewEstimator<T>` (spec §3.1); test helpers in `voyahslam::test`: `TwoViewScene{ correspondences, T_21, points, is_outlier }`, `MakeGeneralScene(size_t n, double outlier_fraction, double sigma, uint32_t seed, const RigidTransformation& T_21)`, `MakePlanarScene(...)` (same params; plane z = 5 in camera 1, optional tilt via an extra `double tilt_deg = 0` last parameter), `DefaultTransform()` (5° about a fixed non-axis-aligned axis, translation 0.5 m), `NormalizedSigma(double sigma_px, double focal = 190.0)`.

- [ ] **Step 1: Failing tests:** `TwoViewSceneTest.NoiseFreeGeneralSceneSatisfiesEpipolarConstraint` (|x2ᵀ [t]ₓ R x1| < 1e-10 for all non-outliers); `.PlanarPointsLieOnPlane`; `.OutlierFractionRespected` (n = 200, 0.3 → 60 outliers); `.SameSeedSameScene`.
- [ ] **Step 2: Run** — expected: compile failure.
- [ ] **Step 3: Implement.** Scene per spec §6 (general: uniform in depth 4–8, within ±30° of the optical axis; noise Gaussian std `sigma` on both views; outliers: x2 replaced by uniform points in [−0.6, 0.6]²). Use `std::mt19937(seed)`.
- [ ] **Step 4: Run** — pass.
- [ ] **Step 5: Commit** `Add TwoViewCorrespondences, estimator concept, synthetic scene helper`.

### Task 3: HomographyEstimator

**Files:**
- Create: `include/voyahslam/estimators/homography_estimator.h`, `src/estimators/homography_estimator.cpp`
- Test: `tests/estimators/homography_estimator_test.cpp`

**Interfaces:**
- Consumes: Task 2 types; `algebra::GetHartleyNormalizationMatrix`, `algebra::SolveHomogeneousSystem`.
- Produces: `estimators::HomographyEstimator { using TModel = geometry::TMatrix33F; static constexpr size_t kSampleSize = 4; static constexpr double kChi2Threshold = 5.991; std::vector<TModel> Estimate(const geometry::TwoViewCorrespondences&, std::span<const Eigen::Index> sample) const; ModelScore Score(const TModel&, const geometry::TwoViewCorrespondences&) const; }` — satisfies `TwoViewEstimator` (static_assert in the test).

- [ ] **Step 1: Failing tests:** `HomographyEstimatorTest.ExactFromFourPlanarPoints` (noise-free planar scene, sample of 4 → 1 model, all points satisfy ‖x2 − π(H x1)‖ < 1e-9); `.CollinearSampleGivesNoModel`; `.IdenticalPointsGiveNoModel`; `.TrueModelOutscoresPerturbed` (σ = NormalizedSigma(1)); `.HandComputedScore` (H = I, σ = 0.01, two correspondences: one exact → contributes 2 × 5.991 and is an inlier; one with x2 offset by 0.02 in x → χ² = 4 each direction → contributes 2 × 1.991, inlier); `.HomographyScoreFiniteForPointMappedToInfinity` (H whose third row sends x1 to w = 0 → that point is an outlier, score finite).
- [ ] **Step 2: Run** — compile failure.
- [ ] **Step 3: Implement** per spec §3.2. Degenerate: return empty if Hartley scale is non-finite/zero or the second-smallest singular value of the 8×9 system < 1e-10·largest. Points with |w| < 1e-12 in either direction fail that direction.
- [ ] **Step 4: Run** focused + full — pass; clang-18/libc++ syntax check of the new header — clean.
- [ ] **Step 5: Commit** `Add HomographyEstimator (4-point DLT, ORB-SLAM3 score)`.

### Task 4: EssentialEstimator

**Files:**
- Create: `include/voyahslam/estimators/essential_estimator.h`, `src/estimators/essential_estimator.cpp`
- Test: `tests/estimators/essential_estimator_test.cpp`

**Interfaces:**
- Consumes: Task 2 types; algebra helpers.
- Produces: `estimators::EssentialEstimator { using TModel = geometry::TMatrix33F; static constexpr size_t kSampleSize = 8; static constexpr double kChi2Threshold = 3.841; static constexpr double kScoreThreshold = 5.991; Estimate(...); Score(...); }` (same signatures as Task 3).

- [ ] **Step 1: Failing tests:** `EssentialEstimatorTest.ExactFromEightPoints` (noise-free general scene → |x2ᵀ E x1| < 1e-9 for all inliers, and E ∝ [t]ₓR of the ground truth up to sign/scale: ‖E/‖E‖ ∓ E_gt/‖E_gt‖‖ < 1e-6 for one sign); `.SingularValuesOnEssentialManifold` (s1 ≈ s2, s3 ≈ 0, rel. 1e-9); `.DegenerateSampleGivesNoModel` (8 points all with the same x1); `.IdenticalPointsGiveNoModel`; `.TrueModelOutscoresPerturbed`; `.HandComputedScore` (E = [t]ₓ for t = (1,0,0), σ = 0.01; a correspondence 0.01 off its epipolar line → χ² = 1 per direction → contributes 2 × 4.991, inlier); `.EssentialScoreFiniteForZeroEpipolarLine` (correspondence where E x1 = 0, e.g. x1 at the epipole → outlier, finite score).
- [ ] **Step 2: Run** — compile failure.
- [ ] **Step 3: Implement** per spec §3.3. Lines with l_x² + l_y² < 1e-24 fail that direction.
- [ ] **Step 4: Run** — pass; clang syntax check — clean.
- [ ] **Step 5: Commit** `Add EssentialEstimator (normalized 8-point, ORB-SLAM3 score)`.

### Task 5: Ransac

**Files:**
- Create: `include/voyahslam/estimators/ransac.h`
- Test: `tests/estimators/ransac_test.cpp`

**Interfaces:**
- Consumes: concept, `ModelScore`, both estimators (tests only).
- Produces: `estimators::RansacOptions`, `estimators::RansacResult<TModel>{ bool success; TModel model; ModelScore score; size_t iterations; }`, `template <TwoViewEstimator TEstimator> class Ransac { explicit Ransac(TEstimator estimator = {}); RansacResult<typename TEstimator::TModel> Run(const geometry::TwoViewCorrespondences&, const RansacOptions&) const; }` (spec §3.4).

- [ ] **Step 1: Failing tests:** `RansacTest.RecoversEssentialWith30PercentOutliers` and `...With50PercentOutliers` (n = 300, σ = NormalizedSigma(1); ≥ 95 % of true inliers flagged inlier, ≤ 5 % of outliers flagged inlier); `.RecoversHomographyOnPlanarScene`; `.CleanDataStopsEarly` (0 % outliers → `iterations < 200`); `.SameSeedSameResult` (two runs → identical model and iterations); `.EmptyInputFails`; `.FewerThanSampleSizeFails`; `.ExactlySampleSizeRuns` (8 noise-free points → success, 8 inliers).
- [ ] **Step 2: Run** — compile failure.
- [ ] **Step 3: Implement** per spec §3.4 (adaptive count as specified; w = 1 → min_iterations; guard log of 0).
- [ ] **Step 4: Run** — pass; clang syntax check — clean.
- [ ] **Step 5: Commit** `Add generic adaptive Ransac<TEstimator>`.

### Task 6: Decomposition

**Files:**
- Create: `include/voyahslam/geometry/two_view_decomposition.h`, `src/geometry/two_view_decomposition.cpp`
- Test: `tests/geometry/two_view_decomposition_test.cpp`

**Interfaces:**
- Consumes: `RigidTransformation`; existing `geometry::DecomposeEssentialToPoses` (verify its convention against these tests; reimplement if it differs).
- Produces: `geometry::HomographyCandidate{ RigidTransformation T_21; TVector3DF plane_normal; }`, `std::array<RigidTransformation, 4> DecomposeEssential(const TMatrix33F& E)`, `std::vector<HomographyCandidate> DecomposeHomography(const TMatrix33F& H)`.

- [ ] **Step 1: Failing tests:** `TwoViewDecompositionTest.EssentialContainsTruePose` (E from ground truth → some candidate with rotation error < 1e-9 and t parallel to t_gt with the same sign); `.EssentialCandidatesAreProperRotations` (det R = 1, ‖t‖ = 1); `.HomographyContainsTruePose` (H = R + t nᵀ/d from a planar scene with tilt 10° → some candidate matches R (1e-9) and t direction (1e-9)); `.HomographyCandidatesAreProperRotations`; `.PureRotationHomographyGivesNoCandidates` (H = R → empty).
- [ ] **Step 2: Run** — compile failure.
- [ ] **Step 3: Implement** per spec §4 (Faugeras / ORB-SLAM3 `ReconstructH`; pure rotation: d1/d2 and d2/d3 within 1.00001 → empty).
- [ ] **Step 4: Run** — pass; clang syntax check — clean.
- [ ] **Step 5: Commit** `Add essential and homography decomposition`.

### Task 7a: Local refinement (amendment 2026-10-04)

**Files:**
- Modify: `include/voyahslam/estimators/{essential_estimator,homography_estimator,ransac}.h`, `src/estimators/{essential_estimator,homography_estimator}.cpp`, `tests/support/two_view_scene.h` (new last parameter `double half_fov_tan = 0.5` on `MakeGeneralScene`/`MakePlanarScene` scaling the u, v range)
- Test: `tests/estimators/essential_estimator_test.cpp`, `tests/estimators/homography_estimator_test.cpp`, `tests/estimators/ransac_test.cpp`, `tests/geometry/two_view_scene_test.cpp`

**Interfaces:**
- Produces: `TModel EssentialEstimator::Refine(const TModel&, const geometry::TwoViewCorrespondences&, const Eigen::Array<bool, Eigen::Dynamic, 1>& inliers) const`; same for `HomographyEstimator`; `RansacOptions::refinement_rounds = 3`; Ransac applies refinement when the estimator has `Refine` (requires-expression), per spec §3.4 amendment and §3.5.

- [ ] **Step 1: Failing tests:** `EssentialEstimatorTest.RefineLowersSampsonError` (estimate from 8 noisy points of a σ = NormalizedSigma(1) scene, refine on the true inliers → summed Sampson error strictly lower, result finite with singular values (s, s, 0)); `HomographyEstimatorTest.RefineLowersTransferError` (same idea, symmetric transfer error); `.RefineKeepsModelOnDegenerateInput` (all-false inliers or < 4 inliers → returns input unchanged); `RansacTest.RefinementImprovesRecovery` (the 30 % / 50 % essential scenes: ≥ 75 % of true inliers flagged, ≤ 5 % outliers); update the existing recovery tests' comment (refinement now on by default); `TwoViewSceneTest.HalfFovScalesPointSpread`.
- [ ] **Step 2: Run** — expected: compile failure (Refine missing).
- [ ] **Step 3: Implement** per spec §3.4 amendment and §3.5 (LM, ≤ 10 iterations, early stop at relative decrease < 1e-6; E: 5-dof (R, t ∈ S²) parameterization; H: 8-dof; non-finite / ill-conditioned result → input returned).
- [ ] **Step 4: Run** focused + full suite — pass; clang syntax check — clean. Report observed recovery numbers.
- [ ] **Step 5: Commit** `Add local refinement (LM) to estimators and Ransac`.

### Task 7: SelectPose and TwoViewReconstructor

> **Amendment 2026-10-04:** model selection uses `homography_inlier_ratio` (H iff inliers_H ≥ 0.8 × inliers_E or the E run failed), not the score ratio. Planar acceptance test `PlanarSceneSelectsHomography` uses `MakePlanarScene(300, 0.2, NormalizedSigma(1), seed, T_21 with 1 m translation, tilt 30°, half_fov_tan = 1.0)`; new test `NarrowFrontalPlanarSceneRejected` (default narrow, tilt 0 scene → `success = false`). Bounds: num_triangulated ≥ 0.5 × true inliers (ledger ruling); pose bounds rotation < 1°, translation direction < 2° unchanged. Work-in-progress for this task is in `git stash` "task7-wip" — restore it with `git stash pop` and continue from it.

**Files:**
- Create: `include/voyahslam/geometry/two_view_reconstruction.h`, `src/geometry/two_view_reconstruction.cpp`
- Test: `tests/geometry/two_view_reconstruction_test.cpp`

**Interfaces:**
- Consumes: Tasks 1–6; existing `geometry::Triangulate`, `geometry::ComputeParallaxCos`.
- Produces: `geometry::TwoViewModel`, `TwoViewOptions`, `TwoViewReconstruction`, `PoseSelection`, `SelectPose(...)`, `TwoViewReconstructor` exactly as spec §5.

- [ ] **Step 1: Failing tests:** `TwoViewReconstructionTest.GeneralSceneSelectsEssential` (n = 300, 20 % outliers, σ = NormalizedSigma(1) → success, model Essential, rotation error < 1°, translation direction error < 2°, num_triangulated ≥ 0.8 × true inliers); `.PlanarSceneSelectsHomography` (tilt 10°, same tolerances, model Homography); `.PureRotationRejected` (translation 0); `.TinyBaselineRejected` (translation 0.01 m); `.TooFewCorrespondencesRejected` (30 points); `.Deterministic` (two runs → same T_21, model, num_triangulated); `.IdenticalCorrespondencesAreNotTriangulated` (`SelectPose` on candidates with x1 == x2 for some inliers → those indices not triangulated, not counted); `.NonPositiveSigmaFails`; `SelectPoseTest.AmbiguousCandidatesRejected` (two copies of the true candidate in the list → uniqueness fails).
- [ ] **Step 2: Run** — compile failure.
- [ ] **Step 3: Implement** per spec §5 (CheckRT rules verbatim from the spec; reprojection error computed in normalized units against 4σ²).
- [ ] **Step 4: Run** — pass; full suite; clang syntax check of all new headers/sources — clean.
- [ ] **Step 5: Commit** `Add SelectPose and TwoViewReconstructor`.

### Task 8: Documentation

**Files:** `docs/DESIGN.md`, `docs/TODO.md`

- [ ] **Step 1: DESIGN.md** — new "Geometry toolkit" section (estimator concept, the two estimators and their scores, Ransac, decomposition, SelectPose/CheckRT rules, TwoViewReconstructor model selection, `RigidTransformation` and the `T_ab` convention); decisions-log rows (8-point now / 5-point later; generic Ransac per model; ORB-SLAM3 scores in normalized units; RigidTransformation with hidden representation); §10 status.
- [ ] **Step 2: TODO.md** — 5-point essential solver; migrate `TPose` users (Frame, events, Tracker, Mapper) to `RigidTransformation` before spec C; optional parallel H/E RANSAC; Environment: clang-18 + libc++ syntax check of all sources as a recurring step (clean on 2026-10-04), later an Android NDK arm64 cross-build.
- [ ] **Step 3: Commit** `DESIGN/TODO: two-view geometry toolkit`.
