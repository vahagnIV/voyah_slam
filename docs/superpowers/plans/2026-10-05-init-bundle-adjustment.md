# Bundle Adjustment After Monocular Initialization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refine the monocular two-view initial map with a g2o bundle adjustment. g2o is vendored pristine under `third_party/` and packaged by our own Conan recipe. The step runs before the median-depth rescale, as in ORB-SLAM3.

**Architecture:**
- **Packaging:** the pristine upstream g2o (`20241228_git`) plus a Conan 2 recipe, published locally as `g2o/20241228@voyah/stable`.
- **Optimizer:** a g2o-free public `optimization::BundleAdjust` API. Its only g2o user is `src/optimization/bundle_adjustment.cpp`, which uses a custom normalized-plane projection edge with analytic Jacobians from an internal helper.
- **Integration:** `MapInitializer` runs the adjustment, drops outliers, rescales, and enforces a minimum point count.

**Tech Stack:** C++20, Eigen 3.4, g2o 20241228 (Eigen solvers only), Conan 2, CMake, GoogleTest.

**Spec:** `docs/superpowers/specs/2026-10-05-init-bundle-adjustment-design.md`

## Global Constraints

- **Branch:** `feat/init-bundle-adjustment`. Commit per task. Never merge or push.
- **Naming (Google style):** `PascalCase` types and functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- **Private dependencies:** no g2o, OpenCV or Pangolin type or header in any public header (`include/`). g2o is linked PRIVATE.
- **Vendored source:** `third_party/g2o/src/` is pristine upstream `20241228_git` (commit `eec325a1`). Never edit files there; adapt in the recipe only.
- **Package reference:** `g2o/20241228@voyah/stable`, requiring `eigen/3.4.0`.
- **Pose convention:** poses are `T_cw` (`p_c = T_cw · p_w`). Measurements are undistorted normalized points (z = 1).
- **Build and test:**
  - `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 120`
  - Focused: `cmake-build-debug/build/Debug/tests/voyahslam_tests --gtest_filter='<Suite>*'`
  - All existing tests stay green (345 on master).
  - After changing `conanfile.txt`, re-run Conan for both trees:
    - `conan install . --output-folder=cmake-build-debug --build=missing -s build_type=Debug`
    - `conan install . --output-folder=build --build=missing -s build_type=Release`
- **Header check:** every new or changed header passes `clang++-18 -std=c++20 -stdlib=libc++ -fsyntax-only`, using the flags from `voyahslam_tests`' `flags.make`.

## Review Focus

1. **Bad input to `BundleAdjust`:** a NaN in a measurement, or a non-positive sigma. Expected: `success == false` and no crash or NaN propagation. Test in Task 3: `RejectsInvalidInput`.
2. **A point behind a camera after optimization.** Expected: its observations come back `inlier == false`, and the initializer drops the point. Test in Task 3: `PointBehindCameraIsOutlier`.
3. **A point observed only by the fixed keyframe** (one observation, unconstrained depth). Expected: it stays finite and nothing diverges. Test in Task 3: `SingleObservationPointStaysFinite`.
4. **Debug vs Release g2o packages.** Expected: both build trees link a matching g2o build type, with no mixed-runtime link errors. Verified in Task 2 (both trees build) and Task 5 (clean-machine run).
5. **A fresh machine without the local g2o package.** Expected: `conan install` fails with a clear "g2o/20241228@voyah/stable not found" message, and the README explains the one-time `conan create`. Verified in Task 5.

---

### Task 1: Vendor g2o and its Conan recipe

**Files:**
- Create: `third_party/g2o/src/` (pristine `20241228_git`, commit `eec325a1`, including `LICENSE`)
- Create: `third_party/g2o/conanfile.py`, `third_party/g2o/test_package/{conanfile.py,CMakeLists.txt,test_package.cpp}`, `third_party/README.md`

**Interfaces:**
- Produces: the Conan package `g2o/20241228@voyah/stable`.
  - CMake config name `g2o`.
  - Component targets, at least: `g2o::stuff`, `g2o::core`, `g2o::types_slam3d`, `g2o::types_sba`, `g2o::solver_eigen`.
  - The inter-component dependencies match upstream's library link order.

- [ ] **Step 1: Vendor the source.** Run `git clone --depth 1 --branch 20241228_git https://github.com/RainerKuemmerle/g2o /tmp/...` (in the scratchpad), check that HEAD is `eec325a1…`, then copy the tree without `.git` to `third_party/g2o/src/`. Record the size in the report.
- [ ] **Step 2: Write the recipe.** The spec §1 values apply verbatim.
  - Read `src/CMakeLists.txt` for the real option names; record any that differ from the spec in `third_party/README.md`.
  - `package_info` declares the components above, with `set_property("cmake_file_name", "g2o")` and per-component `cmake_target_name`, `libs`, and `requires` (`eigen::eigen` for core and stuff).
  - Upstream may name library files with a `g2o_` prefix and a build-type suffix (for example `g2o_core_d`). Read the installed lib names and map them.
- [ ] **Step 3: Write `test_package`.** A tiny program builds an `OptimizationAlgorithmLevenberg` with `BlockSolver_6_3` and `LinearSolverEigen`, adds one `VertexSE3Expmap` and one `VertexPointXYZ`, and calls `initializeOptimization()`. Link the targets above.
- [ ] **Step 4: Run** `conan create third_party/g2o --build=missing -s build_type=Debug`, then the same with `Release`. Expected: both succeed and `test_package` runs.
- [ ] **Step 5: Write `third_party/README.md`:** the upstream URL, tag and commit, license (BSD; note any LGPL parts that are not built), the recipe options, the build commands, and the update procedure (replace `src/`, bump `version`, re-run `conan create`).
- [ ] **Step 6: Commit** `git commit -m "third_party: vendor g2o 20241228 (pristine) with a Conan recipe"`. The source tree is large; commit it as-is.

### Task 2: Link g2o into the project and add the projection helper

**Files:**
- Modify: `conanfile.txt`, `src/CMakeLists.txt`, `tests/CMakeLists.txt` (`src/` as a private include directory for the test target)
- Create: `src/optimization/projection.h`
- Test: `tests/optimization/projection_test.cpp`

**Interfaces:**
- Consumes: the package from Task 1.
- Produces, in `src/optimization/projection.h` (namespace `voyahslam::optimization`, internal, no g2o):
  ```cpp
  [[nodiscard]] geometry::TPoint2DF ProjectNormalized(const geometry::RigidTransformation& T_cw,
                                                      const geometry::TPoint3DF& X);
  void ProjectionJacobians(const geometry::RigidTransformation& T_cw, const geometry::TPoint3DF& X,
                           Eigen::Matrix<precision_t, 2, 6>& out_pose,
                           Eigen::Matrix<precision_t, 2, 3>& out_point);
  ```
  - The pose Jacobian is taken with respect to a left perturbation `T ← exp(ξ)·T`, with ξ = (ω, υ): rotation first, then translation, matching g2o `SE3Expmap`.
  - `out_pose = J_π · [−[Xc]× | I]` and `out_point = J_π · R`, where `J_π = [[1/z, 0, −x/z²], [0, 1/z, −y/z²]]`.

- [ ] **Step 1: Write failing tests:**
  - `ProjectionTest.ProjectsToNormalizedPlane`: identity pose, X = (2, −1, 4) → (0.5, −0.25). A non-trivial pose gives a hand-computed value.
  - `ProjectionTest.JacobiansMatchFiniteDifferences`: 10 seeded random poses and points (depth 2–10). Compare with central differences, step 1e-6.
    - Pose: perturb with `exp(δ)·T`, using `Eigen::AngleAxis` for the rotation part and translation added after.
    - Point: perturb X directly.
    - The max absolute difference must be < 1e-6.
- [ ] **Step 2: Run** `--gtest_filter='Projection*'`. Expected: compile failure.
- [ ] **Step 3: Implement** the helper. In `conanfile.txt` add `g2o/20241228@voyah/stable` and re-run Conan for both trees (see Global Constraints). In `src/CMakeLists.txt` add `find_package(g2o REQUIRED)` and `target_link_libraries(voyahslam PRIVATE g2o::core g2o::stuff g2o::types_sba g2o::types_slam3d g2o::solver_eigen)`. The g2o link is unused until Task 3, but it proves both trees resolve the package.
- [ ] **Step 4: Run** the focused tests, then build **both** trees (`cmake --build build/build/Release -j8` too) and run the full Debug suite. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Link g2o (PRIVATE); normalized projection and analytic Jacobians"`.

### Task 3: `optimization::BundleAdjust`

**Files:**
- Create: `include/voyahslam/optimization/bundle_adjustment.h`, `src/optimization/bundle_adjustment.cpp` (add to `src/CMakeLists.txt`)
- Test: `tests/optimization/bundle_adjustment_test.cpp`

**Interfaces:**
- Consumes: `ProjectNormalized` and `ProjectionJacobians` (Task 2). Test scenes come from `tests/support/two_view_scene.h`:
  - `MakeGeneralScene(n, outlier_fraction, sigma, seed, T_21, half_fov_tan)` returns `TwoViewScene{correspondences{x1, x2, sigma}, T_21, points (camera 1), is_outlier}`.
  - `DefaultTransform()`, `NormalizedSigma(px, focal)`.
- Produces: the spec §2 types and function, verbatim: `BundleAdjustmentOptions{iterations = 20, robust = true, huber_chi2 = 5.991}`, `BaObservation`, `BaProblem`, `BaResult`, and `BundleAdjust(problem, options)`.
- Implementation per spec §2:
  - A custom edge `EdgeNormalizedProjection` (`BaseBinaryEdge<2, Eigen::Vector2d, VertexPointXYZ, VertexSE3Expmap>`) defined in the `.cpp` only.
  - `BlockSolver_6_3` + `LinearSolverEigen` + Levenberg. Points are marginalized.
  - Invalid input (any non-finite measurement or point, sigma ≤ 0, or an out-of-range index) → `success = false` without running.

- [ ] **Step 1: Write failing tests:**
  - `BundleAdjustTest.RecoversPerturbedNoiseFree`: `MakeGeneralScene(200, 0.0, 0.0, 1, DefaultTransform())`.
    - Keyframe 0 is identity and fixed. Keyframe 1 is `T_21` perturbed by 2° about a random axis, and its translation by 10 % of ‖t‖ in a random direction. The points are scaled by 1.05 and jittered by 2 %.
    - Observations: x1 (keyframe 0) and x2 (keyframe 1), with sigma `NormalizedSigma(1.0)`.
    - After `BundleAdjust` with `iterations = 50`, align the scale `s = ‖t_true‖ / ‖t_est‖`. Then: rotation error < 1e-6 rad, ‖s·t_est − t_true‖ < 1e-6, every ‖s·X_est − X_true‖ < 1e-6.
    - Keyframe 0 equals identity exactly. `success`, and `final_cost ≤ initial_cost`.
  - `BundleAdjustTest.FlagsOutliersAndStaysAccurate`: `MakeGeneralScene(300, 0.10, NormalizedSigma(0.5), 2, DefaultTransform())`, with the same perturbation. Use the scene's `is_outlier`: ≥ 95 % of the outlier points have at least one observation with `inlier == false`. Translation-direction error < 0.5°. `final_cost < initial_cost`.
  - `BundleAdjustTest.RejectsInvalidInput`: an empty problem → `success == false`. A NaN measurement → false. `sigma = 0` → false. An observation index out of range → false.
  - `BundleAdjustTest.PointBehindCameraIsOutlier`: add to a clean scene one point placed behind keyframe 1 (its observations are consistent projections of a point at negative depth for keyframe 1). After adjustment its keyframe-1 observation has `inlier == false`.
  - `BundleAdjustTest.SingleObservationPointStaysFinite`: one extra point observed only by the fixed keyframe → its result is finite and all other results are unaffected (same tolerances as the clean test, with a 1e-5 bound).
  - `BundleAdjustTest.RobustFlagChangesCost`: on the outlier scene, `robust = false` gives a `final_cost` different from `robust = true` (> 1e-6 relative).
- [ ] **Step 2: Run** `--gtest_filter='BundleAdjust*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the filter, the full suite, and the clang check of `bundle_adjustment.h`. Expected: green. Also grep `include/` for `g2o`; expected: no hits.
- [ ] **Step 5: Commit** `git commit -m "optimization::BundleAdjust: g2o bundle adjustment behind Eigen types"`.

### Task 4: Bundle adjustment in monocular initialization

**Files:**
- Modify: `include/voyahslam/mapping/map_initializer.h`, `src/mapping/map_initializer.cpp`
- Test: `tests/mapping/map_initializer_test.cpp`

**Interfaces:**
- Consumes: `BundleAdjust`, `BundleAdjustmentOptions` (Task 3).
- Produces: `MapInitializerOptions` gains `bool bundle_adjust = true`, `optimization::BundleAdjustmentOptions bundle_adjustment`, `double level_scale = 1.2`, and `size_t min_points_after_ba = 50`. The behaviour follows spec §3, steps 1–6, verbatim.
  - The two-view result gives the unscaled `T_21` and the triangulated points in camera 1.
  - For each triangulated correspondence c, the features are `first[c]` and `second[c]`; the existing code already has these.
  - Sigma uses each measurement's `levels()(feature)` and its own `K()(0,0)`.

- [ ] **Step 1: Write failing tests:**
  - `MapInitializerTest.BundleAdjustmentImprovesDirection`: for seeds 1–5 of `MakeSyntheticSequence({.seed = s})`, run two initializers on the same frames, one with `bundle_adjust = true` and one with `false`, until both succeed. For each seed:
    - `dir_err_ba ≤ dir_err_plain + 1e-3`°;
    - `dir_err_ba < 1.0`°;
    - median depth == 1 (1e-9);
    - points ≥ 50.

    Print both errors per seed.
  - `MapInitializerTest.TooFewPointsAfterBaFails`: `min_points_after_ba = 100000` → never initializes over the sequence.
  - The existing initializer, Mapper and Slam tests stay green with the default `bundle_adjust = true`.
- [ ] **Step 2: Run** `--gtest_filter='MapInitializer*'`. Expected: failure. The new options don't exist yet.
- [ ] **Step 3: Implement** spec §3.
  - The debug log line uses the existing `CLASS_LOGGER` in `MapInitializer`. If there is none, add one and match the Mapper's usage.
- [ ] **Step 4: Run** the full suite. Expected: green. Note the Debug runtime change of the initializer tests in the report.
- [ ] **Step 5: Commit** `git commit -m "MapInitializer: bundle adjustment after two-view reconstruction"`.

### Task 5: README, clean-machine build, ThreadSanitizer, smoke runs, docs

**Files:**
- Create or modify: `README.md`, `docs/DESIGN.md`, `docs/TODO.md`

- [ ] **Step 1: Write `README.md`** per spec §1 ("README.md"). Use the configure flags for OpenCV 5.1 and Pangolin 0.9.3 recorded in `docs/DESIGN.md` and `docs/TODO.md`. Where a flag isn't recorded, give the minimal set this project needs and say so.
- [ ] **Step 2: Clean-machine check.**
  - Set `CONAN_HOME=<scratchpad>/conan_home`, run `conan profile detect`, then follow the README verbatim: g2o Debug and Release `conan create`, the project's Conan install into fresh output folders under the scratchpad, the build, and `ctest`.
  - First, run the project's `conan install` *before* `conan create` once and record the error message (Review Focus 5).
  - Record each command and its result. Expected: green. Fix the README wherever it was wrong.
- [ ] **Step 3: ThreadSanitizer.**
  - Build a scratch TSan tree from the Debug Conan toolchain with `-fsanitize=thread -O1 -g` and `-DCMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE=PRE_TEST`.
  - Run `setarch $(uname -m) -R <build>/tests/voyahslam_tests --gtest_filter='Mapper*:Slam*'`.
  - Expected: no `WARNING: ThreadSanitizer`.
- [ ] **Step 4: clang-18 + libc++** syntax check of all sources. Expected: clean.
- [ ] **Step 5: Smoke runs** (Release, `/home/vahagn/Downloads/dataset-corridor1_1024_16`).
  - `monocular_full_slam … --rate=0 --max-frames=200`, with debug logging if the debug line is otherwise hidden. A temporary local change is fine; revert it.
  - Record: the init frame, points before and after the adjustment, initial and final cost, outliers removed, against the baseline (frame ~121, 73 points).
  - `binocular_full_slam … --max-frames=20`. Expected: unchanged (355 points).
- [ ] **Step 6: Docs** per spec §4 "Docs".
  - `DESIGN.md`: the optimization section, the bundle adjustment in initialization, the vendored g2o and recipe, and decisions-log rows dated 2026-10-05.
  - `TODO.md`:
    - remove the bundle-adjustment-after-init item;
    - add g2o with a clang/libc++ / NDK profile, and local/global bundle adjustment reusing `BundleAdjust`;
    - merge the pixel-space residual item;
    - add a smoke-run note.
- [ ] **Step 7: Commit** `git commit -m "README build instructions; docs for g2o bundle adjustment"`.
