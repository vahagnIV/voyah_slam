# Camera Model: Residuals and Gates in Raw-Pixel Space Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every residual and gate in the library (projection matcher, `OptimizePose`, `BundleAdjust`, stereo gate, initialization matcher) works in raw (distorted) pixels through a `geometry::Camera`. Fisheye peripheries are then matched and weighted correctly.

**Architecture:**
- `DistortionModel` moves to `geometry/` and gains an analytic `DistortJacobian`.
- A small `geometry::Camera` (K plus a distortion pointer) provides `Project`, `ProjectJacobian` and `Unproject`.
- Measurements keep their raw keypoints and expose `camera()`.
- Each consumer switches its residual/gate space. Rays stay rays.
- Pinhole results must be exactly invariant.

**Tech Stack:** C++20, Eigen, g2o (PRIVATE), GoogleTest, Conan 2/CMake.

**Spec:** `docs/superpowers/specs/2026-10-05-camera-model-design.md`

## Global Constraints

- **Branch:** `feat/camera-model`. Commit per task. Never merge or push.
- **Naming (Google style):** `PascalCase` types/functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- **Public headers:** no g2o/OpenCV/Pangolin types in any public header (`include/`). g2o is PRIVATE.
- **Residual space:** every residual and gate compares raw pixels with σ = `sigma_px · 1.2^level` (pixels). χ² thresholds are unchanged (5.991 / 9.488).
- **Rays stay rays:** two-view RANSAC, triangulation and the 85° ray test do not change space.
- **Pinhole invariance:** with a null distortion, every consumer's χ², inlier decisions and results equal today's (1e-9). Existing pinhole synthetic tests change only at call sites.
- **Distortion model lifetime:** the model is app-owned and must outlive every measurement, event and `Camera` copy.
- **Lens test coefficients:** TUM-VI cam0 (`dataset-corridor1_1024_16/dso/camchain.yaml`).
  - KB: k = (0.010171079892421483, −0.010816440029919381, 0.005942781769412756, −0.001662284667857643).
  - fx = 380.81042871360756, fy = 380.81194179427075, cx = 510.29465304840727, cy = 514.3304630538506.
- **Build/test:**
  - Full: `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 120`.
  - Focused: `cmake-build-debug/build/Debug/tests/voyahslam_tests --gtest_filter='<Suite>*'`.
  - The Release tree `build/build/Release` must build.
  - All existing tests stay green (387 on master), modulo call-site updates.
- **Header check:** every new or changed header passes `clang++-18 -std=c++20 -stdlib=libc++ -fsyntax-only` with the flags from `voyahslam_tests`' `flags.make`.

## Review Focus

1. **A point exactly on the optical axis (r = 0) or numerically tiny r.** Expected: `DistortJacobian` returns the identity limit with no NaN, and `Camera::ProjectJacobian` stays finite. Test in Task 1: `DistortJacobianAtCenterIsIdentity`.
2. **A pinhole measurement (null distortion) everywhere.** Expected: `camera()` works with a null pointer, and `Project` = (K·Xc).hnormalized(). Test in Task 2: `PinholeCameraProjects`; invariance tests in Tasks 3–5.
3. **A stereo observation with `right` set but no `StereoCameras`.** Expected: `OptimizePose` returns `success = false`. Test in Task 3: `RightWithoutStereoCamerasFails`.
4. **`BaProblem` whose `keyframe_cameras` size differs from `keyframe_poses`.** Expected: `success = false`, not out-of-range access. Test in Task 4: `CameraCountMismatchFails`.
5. **Features dropped by the 85° rule.** Expected: `distorted_points()` stays aligned with `points()`, `levels()` and the descriptors. Test in Task 2: `DistortedPointsCompactedWithDrop`.

---

### Task 1: Move `DistortionModel` into `geometry/`; add `DistortJacobian`

**Files:**
- Move headers: `include/voyahslam/features/distortion_models/{distortion_model,dummy_distortion_model,kannala_brandt_distortion_model}.h` → `include/voyahslam/geometry/distortion_models/`.
- Move sources: `src/features/distortion_models/*.cpp` → `src/geometry/distortion_models/`.
- Move tests: `tests/features/distortion_models/*` → `tests/geometry/distortion_models/`.
- Create: forwarding headers at the old include paths.
- Modify: `src/CMakeLists.txt`, `tests/CMakeLists.txt`, and all in-repo includes (`grep -rln distortion_models include src tests examples vizualization`).

**Interfaces:**
- **Produces:**
  - Namespace `voyahslam::geometry::distortion_models`.
  - The old headers forward: they include the new header and add `namespace voyahslam::features { namespace distortion_models = geometry::distortion_models; }`.
  - `[[nodiscard]] virtual geometry::TMatrix22F DistortJacobian(const geometry::TPoint2DF& undistorted) const = 0;`. Dummy returns identity. KB uses the spec §1.1 formula, with identity when r < 1e-8.
- **Steps:**

- [ ] **Step 1: Do the mechanical move.**
  - `git mv` the files and update the namespaces.
  - Add the forwarding headers.
  - Update in-repo includes to the new path.
  - Build the whole tree and run the full suite. Expected: green with the same count. Commit `git commit -m "Move DistortionModel into geometry/ (forwarding headers at the old path)"`.
- [ ] **Step 2: Write failing tests** in `tests/geometry/distortion_models/kannala_brandt_distortion_model_test.cpp`, using TUM-VI cam0 coefficients:
  - `DistortJacobianMatchesFiniteDifferences`: at r ∈ {1e-6, 0.1, 1.0, tan(80°)} along several directions, plus 10 random points with r ≤ tan(85°). The Jacobian must equal central FD of `Distort` (step 1e-7, tolerance 1e-6 relative).
  - `DistortJacobianAtCenterIsIdentity`: at (0, 0) and (1e-12, 0), J equals I, with no NaN.
  - Dummy: `DummyDistortJacobianIsIdentity`.
- [ ] **Step 3: Run** `--gtest_filter='*Distort*'`. Expected: compile failure.
- [ ] **Step 4: Implement.**
- [ ] **Step 5: Run** the filter, the full suite, and the clang check. Expected: green.
- [ ] **Step 6: Commit** `git commit -m "DistortionModel::DistortJacobian (analytic Kannala-Brandt)"`.

### Task 2: `geometry::Camera`; measurements keep raw keypoints

**Files:**
- Create: `include/voyahslam/geometry/camera.h`, `src/geometry/camera.cpp`.
- Modify: `include/voyahslam/features/monocular_measurement.h`.
- Test: `tests/geometry/camera_test.cpp` (add to CMake) and `tests/features/monocular_measurement_test.cpp`.

**Interfaces:**
- **Consumes:** Task 1.
- **Produces:**
  - `geometry::Camera`, exactly as spec §1.2.
  - `MonocularMeasurement::distorted_points()`: `const TPoints2DF&`, the raw keypoints as passed to the constructor and compacted with the 85° drop.
  - `MonocularMeasurement::camera()`: `const geometry::Camera&`.
  - The measurement stores `camera_` (K and the distortion pointer it was constructed with).
- **Steps:**

- [ ] **Step 1: Write failing tests:**
  - `CameraTest.PinholeCameraProjects`: with a null distortion, `Project(Xc) == (K·Xc).hnormalized()`.
  - `CameraTest.ProjectUnprojectRoundTripKB`: with TUM-VI cam0, rays at 0°, 10°, …, 80°, 85° in 8 azimuths. `Unproject(Project(Xc))` must be parallel to Xc (angle < 1e-6 rad).
  - `CameraTest.ProjectJacobianMatchesFiniteDifferences`: KB and pinhole, 10 random Xc with z > 0 and angles up to 80°. Tolerance 1e-6 relative.
  - `MonocularMeasurementTest.DistortedPointsAndCamera`: `distorted_points()` equals the input keypoints; `camera().K` equals K; `camera().distortion` equals the pointer.
  - `MonocularMeasurementTest.DistortedPointsCompactedWithDrop`: a fisheye input where some features are beyond 85° and get dropped. `distorted_points()`, `points()` and `levels()` stay aligned, checked via `camera().Project(normalized_point(i))` ≈ `distorted_points().col(i)` (1e-6 px).
- [ ] **Step 2: Run** `--gtest_filter='Camera*:MonocularMeasurement*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the filter, the full suite, and the clang check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "geometry::Camera; measurements keep raw keypoints and expose camera()"`.

### Task 3: `OptimizePose` in raw pixels (and the Tracker call site)

**Files:**
- Modify: `src/optimization/projection.h`, `include/voyahslam/optimization/pose_optimization.h`, `src/optimization/pose_optimization.cpp`, `src/tracking/tracker.cpp`.
- Test: `tests/optimization/projection_test.cpp`, `tests/optimization/pose_optimization_test.cpp`; Tracker tests only if call sites change.

**Interfaces:**
- **Consumes:** `geometry::Camera` (Task 2).
- **Produces:**
  - `PoseObservation` (raw px, sigma px).
  - `StereoCameras{geometry::Camera right; RigidTransformation T_rl;}`.
  - `OptimizePose(initial_T_cw, const geometry::Camera& left, const std::optional<StereoCameras>& stereo, observations, options)`, exactly as spec §2.2.
  - `projection.h` helpers take a `Camera`:
    - `ProjectPixel(camera, T_cw, X)`;
    - `PixelJacobians(camera, T_cw, X, out_pose 2×6, out_point 2×3)`;
    - `ProjectPixelRight(right_camera, T_cw, T_rl, X)`;
    - `PixelJacobianRightPose(right_camera, T_cw, T_rl, X, out_pose)`.
  - The normalized-plane helpers are removed. `BundleAdjust` still uses them until Task 4, so keep them until Task 4 removes them, and mark them deprecated in a comment.
  - The Tracker builds observations from `distorted_points()` and passes `left().camera()`, plus `StereoCameras{right().camera(), rig().T_rl}` for binocular.
- **Steps:**

- [ ] **Step 1: Write failing tests:**
  - `ProjectionTest.PixelJacobiansMatchFiniteDifferences` and `ProjectionTest.PixelJacobianRightMatchesFiniteDifferences`: KB TUM-VI cameras, a 5° yaw rig, 10 random cases with points up to 80°, 1e-6 relative.
  - `PoseOptimizationTest.PinholeInvariance`: build a pinhole problem (0.5 px noise, 10 % outliers). Run the new API twice:
    - (a) the real K, pixel measurements and σ = 1.0 px;
    - (b) an identity-K camera, normalized measurements (K⁻¹·pixel) and σ = 1.0/f.
    Pose, per-observation χ² and inliers must be equal (1e-9). With identity K the formulation is exactly the old normalized-plane one, so this pins pinhole invariance with no copy of old code.
  - `PoseOptimizationTest.FisheyeMonoRecoversPose` / `FisheyeStereoRecoversPose`: points up to 80° off-axis projected through KB, noise-free, perturbed start. Recovery to 1e-6.
  - `PoseOptimizationTest.RightWithoutStereoCamerasFails` (Review Focus 3).
  - Update the existing `PoseOptimization*` tests to the new signature (pinhole: pixel measurements = K·normalized, σ in px). Their tolerances stay unchanged.
- [ ] **Step 2: Run** `--gtest_filter='Projection*:PoseOptimization*'`. Expected: compile failure.
- [ ] **Step 3: Implement,** including the Tracker call site.
- [ ] **Step 4: Run** the filter, `Tracker*:Slam*`, the full suite, the Release build, and the clang check. Expected: green. The Tracker synthetic accuracy tests (pinhole) must pass unchanged.
- [ ] **Step 5: Commit** `git commit -m "OptimizePose in raw pixels through geometry::Camera"`.

### Task 4: `BundleAdjust` in raw pixels (and the `MapInitializer` call site)

**Files:**
- Modify: `include/voyahslam/optimization/bundle_adjustment.h`, `src/optimization/bundle_adjustment.cpp`, `src/optimization/projection.h` (remove the normalized helpers), `src/mapping/map_initializer.cpp`.
- Test: `tests/optimization/bundle_adjustment_test.cpp`, `tests/mapping/map_initializer_test.cpp` (call sites only).

**Interfaces:**
- **Consumes:** Tasks 2–3 (`Camera`, `PixelJacobians`).
- **Produces:**
  - `BaProblem.keyframe_cameras` (`std::vector<geometry::Camera>`, size must equal `keyframe_poses`).
  - `BaObservation.measurement` in raw px; `sigma` in px.
  - `MapInitializer` builds observations from `distorted_points()` with each view's `camera()`, and sigma = `sigma_px · level_scale^level` (px).
- **Steps:**

- [ ] **Step 1: Write failing tests:**
  - `BundleAdjustTest.PinholeInvariance`: same pattern as Task 3 (real K + pixels + σ px vs identity K + normalized + σ/f; results equal to 1e-9).
  - `BundleAdjustTest.FisheyeRecoversPerturbedNoiseFree`: two views through KB TUM-VI, points up to 80°. Same tolerances as the existing noise-free test.
  - `BundleAdjustTest.CameraCountMismatchFails` (Review Focus 4).
  - Update the existing tests to the new fields.
- [ ] **Step 2: Run** `--gtest_filter='BundleAdjust*:MapInitializer*'`. Expected: compile failure.
- [ ] **Step 3: Implement.** Remove the normalized-plane projection helpers.
- [ ] **Step 4: Run** the filter, the full suite (`MapInitializerTest.BundleAdjustmentReachesOptimum` must still pass), and the clang check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "BundleAdjust in raw pixels; initialization BA uses camera()"`.

### Task 5: Matchers in raw pixels (projection, stereo gate, initialization)

**Files:**
- Modify: `include/voyahslam/features/projection_matcher.h`, `include/voyahslam/features/stereo_matcher.h`, `include/voyahslam/features/initialization_matcher.h`, `src/mapping/map_initializer.cpp` (`last_matched` and grid on distorted points).
- Test: `tests/features/projection_matcher_test.cpp`, `tests/features/stereo_matcher_test.cpp`, `tests/features/initialization_matcher_test.cpp`.

**Interfaces:**
- **Consumes:** `Camera`, `distorted_points()` (Task 2).
- **Produces** (spec §2.1, §2.4, §2.5):
  - **Projection matcher:** projects with `current.camera().Project`; bounding box and `FeatureGrid` on `distorted_points()`; same windows in raw px; public signatures unchanged.
  - **Stereo gate:** projects the triangulated point with each image's `camera()` and compares with `distorted_points()`, σ px.
  - **Initialization matcher:** grid on `current.distorted_points()`; `MapInitializer` starts `last_matched_` from the reference's `distorted_points()`.
- **Steps:**

- [ ] **Step 1: Write failing tests:**
  - `ProjectionMatcherTest.FisheyePeripheralPointsMatch`:
    - Build a KB TUM-VI scene with map points at 60–80°. The current frame's raw keypoints are within 2 raw px of the true projection.
    - Matched with the new code. The test also asserts that the old undistorted-space window (radius 2.5·th in undistorted px) would miss them: compute their undistorted offset > window, as a sanity check of the scene.
  - `StereoMatcherTest.FisheyePeripheralPairsAccepted`: KB stereo pairs at 60–80° with 0.5 px noise are accepted. The same pairs shifted 5 raw px in one image are rejected.
  - `InitializationMatcherTest.UsesRawPixels`: a fisheye pair where `out_last_matched` and the matches are in raw pixels (equal to `distorted_points()` of the matched features).
  - Existing pinhole tests of all three matchers pass unchanged. Update construction sites only if a test builds measurements without raw points; the constructor sets `distorted_points()` from the input either way.
- [ ] **Step 2: Run** `--gtest_filter='ProjectionMatcher*:StereoMatcher*:InitializationMatcher*:MapInitializer*'`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the full suite, the Release build, and the clang check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Matchers in raw pixels: projection, stereo gate, initialization"`.

### Task 6: ThreadSanitizer, smoke runs, docs

**Files:**
- Modify: `docs/DESIGN.md`, `docs/TODO.md`.

- [ ] **Step 1: ThreadSanitizer.** Scratch TSan tree from the Debug Conan toolchain (`-fsanitize=thread -O1 -g`, PRE_TEST discovery). Run `setarch $(uname -m) -R <build>/tests/voyahslam_tests --gtest_filter='Tracker*:Slam*:Mapper*'`. Expected: no warnings.
- [ ] **Step 2: clang-18 + libc++** check of all sources. Expected: clean.
- [ ] **Step 3: Smoke runs** (Release, corridor1 1024, apps log at debug).
  - **Mono, 400 frames:**
    - Record tracked frames per map, the total and re-initializations, against baseline 32 / 7 and the spike's KB variants (53 / 5; with gate 20, 108 / 5).
    - Success criterion: ≥ 53 tracked and ≤ 5 re-initializations.
    - Also run once with a temporary local-map gate of 20 (`TrackerOptions` via a temporary local change, reverted). Report it only.
  - **Stereo, 200 frames:** all tracked; record median inliers.
  - **Both:** init frame and point counts.
- [ ] **Step 4: Docs** per spec §3 "Docs" (decisions-log rows dated 2026-10-05, with the spikes' numbers).
- [ ] **Step 5: Commit** `git commit -m "Docs: camera model and raw-pixel residuals; smoke runs"`.
