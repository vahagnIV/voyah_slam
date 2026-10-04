# Stereo Map Initialization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a real, metric-scale map from one stereo frame. Left↔right stereo matching (ORB-SLAM3 fisheye style) runs in the Tracker thread, the stereo `MapInitializer` creates the map, and the Mapper publishes it like the mono map.

**Architecture:** The tasks, in order:
1. Fix the rig convention (`T_rl`) and give `BinocularMeasurement` a rig copy and match storage.
2. Add synthetic stereo test support.
3. Implement `MatchStereo`.
4. The Tracker matches on dequeue (new `TrackerOptions`).
5. Replace the stereo stub in `MapInitializer`, and give the Mapper a stereo observation storage.
6. Apps smoke run and docs.

**Tech Stack:** C++20, Eigen, GoogleTest, CMake/Conan 2.

**Spec:** `docs/superpowers/specs/2026-10-04-stereo-initialization-design.md`

## Global Constraints

- Branch `feat/stereo-initialization`. Commit per task; never merge or push.
- Google style:
  - `PascalCase` types and functions;
  - `snake_case` variables and `snake_case_` members;
  - `out_` prefix for non-const reference parameters.
- No OpenCV in public headers. `voyahslam` gains no dependency.
- Poses follow `T_ab`: p_a = T_ab · p_b. `StereoRig::T_rl` maps left → right. Keyframe poses are `T_cw`.
- Map point ids restart at 0 per map; frame ids are never reused.
- Build and test:
  - `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 120`
  - focused: `cmake-build-debug/build/Debug/tests/voyahslam_tests --gtest_filter='<Suite>*'`
  - All existing tests stay green (318 on master).
- Every new or changed header passes `clang++-18 -std=c++20 -stdlib=libc++ -fsyntax-only`, using the flags from `voyahslam_tests`' `flags.make`.

## Review Focus

1. **Empty left or right image.** A binocular measurement with 0 features on either side. Expected: `MatchStereo` returns correctly sized arrays (left count) with no matches and no crash. Test in Task 3: `EmptySideYieldsNoMatches`.
2. **Right image with exactly one feature (no second best).** Expected: second-best counts as +∞, so the ratio test passes and geometry decides. Test in Task 3: `SingleRightCandidateCanMatch`.
3. **A stereo map after a mono map in the same run** (via `TrackingLost`). Expected: map 2 is stereo, its point ids start at 0, and both observation storages are cleared. Test in Task 5: `StereoMapAfterMonoMap`.
4. **A binocular measurement that reaches the initializer unmatched** (`stereo_matched() == false`, e.g. a direct test call). Expected: no map in release, debug assert. Not tested; the assert is documented in Task 5.
5. **More than 32767 right features.** `right_feature_index` is `int16_t`. Expected: a debug assert, never silent wrap-around. Assert in Task 5; no test.

---

### Task 1: `StereoRig::T_rl`; `BinocularMeasurement` rig copy and match storage

**Files:**
- Modify: `include/voyahslam/features/stereo_rig.h`
- Modify: `include/voyahslam/features/binocular_measurement.h`
- Modify: `examples/dataset_reader/src/euroc_dataset_reader.cpp` (`BuildStereoRig`)
- Create: `include/voyahslam/features/stereo_matches.h`
- Modify fixtures: `tests/support/test_measurements.h`, `tests/map/frame_test.cpp`, `tests/features/binocular_measurement_test.cpp`, `tests/features/stereo_rig_test.cpp`, and any other `R_rl`/`t_rl` user (`grep -rn "R_rl\|t_rl" .`)
- Test: `tests/features/stereo_rig_test.cpp`, `tests/features/binocular_measurement_test.cpp`

**Interfaces:**
- Produces: `StereoRig` exactly as in spec §1.1.
  - `R_rl` and `t_rl` are removed.
  - `T_rl` is a `geometry::RigidTransformation` (left → right).
  - `baseline = ‖T_rl.translation()‖`, set by whoever builds the rig.
- Produces, in `features/stereo_matches.h`:
  ```cpp
  struct StereoMatches {
      Eigen::VectorXi right_indices;  // per left feature; −1 = none
      geometry::TRowVectorF depths;   // z in the left camera; −1 = none
      size_t num_matches = 0;
  };
  ```
- Produces, on `BinocularMeasurement`:
  - `const StereoRig& rig() const`
  - `void SetStereoMatches(StereoMatches&& matches)`: asserts sizes equal the left feature count, replaces `depths_` and `right_indices_`, sets `stereo_matched_`.
  - `bool stereo_matched() const`

- [ ] **Step 1: Write failing tests**
  - `StereoRigTest.TrlMapsLeftToRight`: a rig with right at +x, `T_rl = (I, (−0.12, 0, 0))`. The left point (1, 0, 5) maps to (0.88, 0, 5).
  - `BinocularMeasurementTest.StoresRig`: `rig().T_rl` round-trips (`ToMatrix34().isApprox`).
  - `BinocularMeasurementTest.SetStereoMatches`:
    - before the call: `stereo_matched()` is false and depths are all −1;
    - after setting a `StereoMatches` with 2 valid entries: `stereo_matched()` is true and `depths()` / `right_indices()` equal the given ones.
- [ ] **Step 2: Run** `--gtest_filter='StereoRig*:BinocularMeasurement*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
  - Reader: `T_rl = T_right_imu · T_left_imu⁻¹`, converted from the 4×4 matrix into `RigidTransformation(R, t)`.
  - Update the comment on the reader line that currently says "maps right-camera points into the left camera".
  - Fix the fixtures: right at +x → translation (−b, 0, 0).
- [ ] **Step 4: Run** the focused tests, then the full suite (the whole tree including examples must build). Expected: green.
- [ ] **Step 5: Commit** `git commit -m "StereoRig: T_rl (left to right); BinocularMeasurement stores rig and stereo matches"`.

### Task 2: Synthetic stereo test support

**Files:**
- Modify: `tests/support/synthetic_sequence.h`
- Test: `tests/support/synthetic_sequence_test.cpp`

**Interfaces:**
- Consumes: `StereoRig` (Task 1), the existing scene/descriptor helpers in `synthetic_sequence.h`.
- Produces:
  ```cpp
  struct SyntheticStereoOptions {
      size_t num_points = 600;
      double min_depth = 2.0, max_depth = 6.0;
      double baseline = 0.11;           // right camera at +x
      double pixel_noise = 0.5;         // σ px, independent per image
      int bit_flips = 3;                // left vs right descriptor
      size_t num_extra = 40;            // per image
      double timestamp = 0.0;
      uint32_t seed = 1;
  };
  struct SyntheticStereoFrame {
      features::TBinocularMeasurement<TestExtractor> measurement;
      geometry::TPoints3DF points_left;          // true 3D points (left camera frame)
      std::vector<Eigen::Index> right_of_left;   // true right index per left feature, −1 = extra / not visible
      std::vector<double> depth_of_left;         // true depth per left feature, −1 = extra
  };
  geometry::TMatrix33F SyntheticStereoIntrinsics();  // fx = fy = 380, cx = cy = 512 (1024² image)
  features::StereoRig SyntheticStereoRig(double baseline = 0.11);  // both K = SyntheticStereoIntrinsics(), T_rl = (I, (−b, 0, 0)), no distortion
  SyntheticStereoFrame MakeSyntheticStereoFrame(const SyntheticStereoOptions& options = {});
  ```
- **Generation:**
  - Each point is a uniform left pixel in [20, 1004]² back-projected to a uniform depth.
  - The point is projected into the right camera with `T_rl`. It is kept as a matched pair only if the right projection lies inside the image with z > 0; otherwise it appears only in the left image, with `right_of_left = −1`.
  - Noise is independent per image. Descriptor bits are flipped between left and right. Angles are consistent, with small noise. Levels are 0. Both images are shuffled independently, and the ground-truth vectors stay aligned with the final orders.
  - **Left feature count is ≥ 600** with the default options, so that it exceeds `stereo_min_features` (500).

- [ ] **Step 1: Write tests:**
  - `SyntheticStereoTest.GroundTruthConsistent`: with `pixel_noise = 0`, every pair with `right_of_left[i] = j` reprojects exactly. That is, left point i ≈ K·X/z and right point j ≈ K·T_rl·X/z (1e-9). Also `depth_of_left[i] == X.z()`.
  - `SyntheticStereoTest.EnoughFeaturesForInit`: left feature count ≥ 600; ≥ 400 true pairs.
- [ ] **Step 2: Run** `--gtest_filter='SyntheticStereo*'`. Expected: pass.
- [ ] **Step 3: Commit** `git commit -m "Test support: synthetic stereo frame"`.

### Task 3: `MatchStereo`

**Files:**
- Create: `include/voyahslam/features/stereo_matcher.h` (header-only template)
- Test: `tests/features/stereo_matcher_test.cpp` (add to `tests/CMakeLists.txt`)

**Interfaces:**
- Consumes: Task 1 (`StereoMatches`, `rig()`), Task 2 (synthetic frame), `geometry::Triangulate(pt1, pt2, P1, P2, out)` in `geometry/geometry.h`.
- Produces: `StereoMatcherOptions` and `MatchStereo<FE>(measurement, options) -> StereoMatches`, exactly as in spec §1.2 (defaults 0.7, 1.0, 1.2, 5.991, 0.99998).
- **Algorithm:** spec §1.2 steps 1–5, verbatim.
  - Use `P_l = [I | 0]` and `P_r = rig.T_rl.ToMatrix34()`.
  - Use the homogeneous normalized points from `left().normalized_points()` and `right().normalized_points()`.
  - The reprojection residual compares the 2D normalized projection with the measured normalized point. In image i, `σ_i = sigma_px · level_scale^level_i / K_i(0,0)`.
  - Parallax: `cos = (X − C_l)·(X − C_r) / (‖·‖‖·‖)`, with `C_l = 0` and `C_r = T_rl.Inverse().translation()`. Reject when `cos ≥ max_parallax_cos`.

- [ ] **Step 1: Write failing tests.** Hand-built cases use `SyntheticStereoRig()` and pixel coordinates computed from chosen 3D points.
  - `RatioTestRejectsAmbiguous`: two right candidates at distances 10 and 12 → no match (10 ≥ 0.7·12). At 10 and 20 → match.
  - `OneToOneKeepsCloserClaim`: two left features both best-match right j, at distances 5 and 9 → only the distance-5 left feature keeps j.
  - `RejectsPointBehindRightCamera`: the left and right rays only intersect behind the right camera → no match.
  - `RejectsNearParallelRays`: a point at 2000 m → rejected by parallax.
  - `RejectsGeometricallyInconsistentPair`: identical descriptors, but the right point is shifted 20 px vertically → rejected by the reprojection test.
  - `EmptySideYieldsNoMatches`:
    - 0 right features → `right_indices.size() == #left`, all −1, `num_matches == 0`;
    - 0 left features → empty arrays.
  - `SingleRightCandidateCanMatch`: exactly one right feature, consistent with the left point → matched.
  - `SyntheticNoiseFreeExact`: `MakeSyntheticStereoFrame({.pixel_noise = 0})` → every true pair is found. For each, |depth − truth| < 1e-6 and `right_indices[i] == right_of_left[i]`.
  - `SyntheticPrecisionAndDepth`: default noise → precision ≥ 0.98 and median relative depth error < 0.10. Recall is printed.
- [ ] **Step 2: Run** `--gtest_filter='StereoMatcher*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the filter, then the full suite, then the clang syntax check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "MatchStereo: ORB-SLAM3 fisheye-style stereo matching"`.

### Task 4: Tracker matches stereo on dequeue

**Files:**
- Modify: `include/voyahslam/tracking/tracker.h`, `src/tracking/tracker.cpp`, `include/voyahslam/slam/slam.h`, `src/slam/slam.cpp`
- Test: `tests/tracking/tracker_test.cpp`, `tests/slam/slam_test.cpp` (constructor wiring)

**Interfaces:**
- Consumes: `MatchStereo`, `StereoMatcherOptions` (Task 3), `SetStereoMatches` (Task 1).
- Produces:
  - `struct TrackerOptions { features::StereoMatcherOptions stereo_matcher; };` in `tracker.h`
  - `explicit Tracker(TrackerOptions options = {})`
  - `explicit Slam(mapping::MapperOptions mapper_options = {}, tracking::TrackerOptions tracker_options = {})`
- **Behaviour:** in `ProcessMeasurement` (the dequeue path, `tracker.cpp`), a binocular alternative gets `SetStereoMatches(MatchStereo(m, options_.stereo_matcher))` before anything else uses it, including `PublishInitializationMeasurement`. Mono is untouched.

- [ ] **Step 1: Write failing tests:**
  - `TrackerTest.StereoMatchedBeforeForwarding`: track `MakeSyntheticStereoFrame().measurement` while `WaitingForMap`. The forwarded `InitializationMeasurement` holds a binocular measurement with `stereo_matched() == true`, and its `depths()` equal a direct `MatchStereo` call on the same input.
  - `SlamTest.ForwardsTrackerOptions`: construct with a non-default `TrackerOptions`; it compiles and runs. Assert a field through a test accessor only if one already exists. Otherwise this is a construction smoke test.
- [ ] **Step 2: Run** `--gtest_filter='Tracker*:Slam*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the full suite. Expected: green. Existing tests built from random binocular measurements still pass here, because the stereo stub initializer is unchanged until Task 5.
- [ ] **Step 5: Commit** `git commit -m "Tracker: stereo-match binocular measurements on dequeue; TrackerOptions"`.

### Task 5: Stereo `MapInitializer` and Mapper stereo observations

**Files:**
- Modify: `include/voyahslam/mapping/map_initializer.h`, `src/mapping/map_initializer.cpp`, `include/voyahslam/mapping/mapper.h`, `src/mapping/mapper.cpp`
- Test: `tests/mapping/map_initializer_test.cpp`, `tests/mapping/mapper_test.cpp`, `tests/slam/slam_test.cpp`, `tests/tracking/tracker_test.cpp` (only where binocular inputs must now build a map)

**Interfaces:**
- Consumes: Tasks 1–4. Test inputs use `MakeSyntheticStereoFrame` with `MatchStereo` applied, or go through the Tracker.
- Produces:
  - `MapInitializerOptions::stereo_min_features = 500` and `stereo_min_points = 100`;
  - the binocular path of spec §2.1;
  - `InitialMapPoint::feature_in_second == −1` documented as "no second keyframe";
  - the Mapper member `map::ObservationStorage<map::ObservationType::Stereo> stereo_observations_`, used per spec §2.2.

- [ ] **Step 1: Write failing tests:**
  - `MapInitializerTest`:
    - `StereoInitializesFromOneFrame`: a matched synthetic frame →
      - 1 keyframe, `T_cw` identity;
      - point count = number of valid depths;
      - each point equals `depth · normalized_point(i)` (1e-12) and lies within 5 % of the true point;
      - `feature_in_second == −1`.
    - `StereoTooFewFeatures`: a matched frame with ≤ 500 left features → nullopt. Build it with `num_points` small and `num_extra = 0`.
    - `StereoTooFewPoints`: > 500 left features but < 100 valid depths → nullopt. Use mostly extras: `num_points = 50`, `num_extra = 600`.
    - `BinocularDropsMonoReference`: keep the existing test, with a matched synthetic stereo frame instead of `MakeBinocular`.
  - `MapperTest`:
    - `StereoPublishesPointsKeyFrameThenMap`:
      - order `MapPointsAdded`, `KeyFrameCreated`, `MapCreated`;
      - `count() == num_map_points() ≥ 100`;
      - the observations reference valid ids and features;
      - normals are unit length, and `normal · (X − C) > 0`.
    - `StereoMapAfterMonoMap`: mono init (existing synthetic mono path) → `TrackingLost(1)` → stereo frame → map 2, with `MapPointsAdded.start_id == 0` and `num_map_points()` equal to the stereo point count.
    - Update the existing stereo tests (`StereoInitializesFromOneMeasurement`, `TrackingLostStartsNewMap`, `IgnoresStaleTrackingLost`, `StateIsMappingWhenMapCreatedIsPublished`, `IgnoresInitializationMeasurementWhileMapping`, `StereoPublishesNoMapPoints` → rename to `StereoPublishesMapPoints`) so they use a matched synthetic stereo frame. The expectations change from "0 points" to "≥ 100 points" where relevant.
  - `SlamTest`: add `EndToEndStereoInitializeTrackLoseReinitialize`: binocular synthetic frames through the Tracker → `MapCreated(1)` with `MapPointsAdded` first → `PositionUpdated` → `MakeBinocular(0, …)` loss → a new stereo frame → `MapCreated(2)`. Increasing timestamps.
  - Tracker tests that use random binocular measurements: keep them where no map is expected. Switch them to synthetic stereo where a map is expected.
- [ ] **Step 2: Run** `--gtest_filter='MapInitializer*:Mapper*:Slam*:Tracker*'`. Expected: failures.
- [ ] **Step 3: Implement** spec §2.1–2.2.
  - Assert `right_indices[i] <= 32767` before the `int16_t` cast.
  - Assert `stereo_matched()` in the binocular path (debug only); release treats it as no depths.
- [ ] **Step 4: Run** the full suite. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Stereo MapInitializer and Mapper stereo observations"`.

### Task 6: ThreadSanitizer, apps smoke run, docs

**Files:**
- Modify: `docs/DESIGN.md`, `docs/TODO.md`

- [ ] **Step 1: ThreadSanitizer.**
  - Build in a scratch dir with `-fsanitize=thread -O1 -g`, `-DCMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE=PRE_TEST`, and the Debug Conan toolchain.
  - Run `setarch $(uname -m) -R <build>/tests/voyahslam_tests --gtest_filter='Slam*:Mapper*:Tracker*'`.
  - Expected: pass, no `WARNING: ThreadSanitizer`.
- [ ] **Step 2: clang-18 + libc++ syntax check** of all sources. Expected: clean.
- [ ] **Step 3: Apps smoke run** (Release, dataset `/home/vahagn/Downloads/dataset-corridor1_1024_16`).
  - Run `binocular_full_slam <dataset> --rate=0 --max-frames=20`, plus `--viewer` if a display is available.
  - Record: the init frame, the point count from the Mapper log line, the median map-point depth (add a temporary log line, or compute it from a test harness; do not commit debug code), and a viewer screenshot if possible.
  - Expected: init within the first few frames, hundreds of points, median depth roughly 1–5 m.
  - Also run `monocular_full_slam … --max-frames=200` to confirm mono is unchanged: init around frame 121, about 73 points.
- [ ] **Step 4: Docs** per spec §3 "Docs".
  - `DESIGN.md` sections and decisions-log rows dated 2026-10-04.
  - `TODO.md`: remove the stereo matching and real stereo initialization items; add epipolar-guided stereo search, stereo-aware tracking, and the smoke-run outcome as a note.
- [ ] **Step 5: Commit** `git commit -m "Docs: stereo initialization; smoke run"`.
