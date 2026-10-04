# Monocular Map Initialization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Mapper builds a real initial map from two monocular frames — ORB-SLAM3 initialization matching, two-view reconstruction, median-depth scaling — and publishes map points, keyframes with observations, and `MapCreated`.

**Architecture:** Prerequisites first (keypoint levels, `TPose` → `RigidTransformation`, `FeatureGrid`, a synthetic sequence for tests), then the matcher (header-only template), the monocular `MapInitializer` on top of the existing `TwoViewReconstructor`, and finally the Mapper storing and publishing the map. Stereo keeps its stub; tracking stays a stub.

**Tech Stack:** C++20, Eigen, spdlog, GoogleTest, Conan 2 / CMake; OpenCV 5.1 only inside `src/` (SIFT).

**Spec:** `docs/superpowers/specs/2026-10-04-mono-initialization-design.md`

## Global Constraints

- Branch `feat/mono-initialization`. Commit per task; never merge or push.
- Google style: `PascalCase` types/functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters. One event type per file.
- OpenCV must not appear in any public header (`include/`); it stays PRIVATE.
- Map point / frame ids are never reused within a map; frame ids are unique across maps; map point ids restart at 0 per map — (`map_id`, `MapPointId`) identifies a point.
- Keyframe pose = `T_cw` (world → camera), world = camera 1 of the map.
- Feature angles are radians for every extractor.
- Build/test: `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 60`; focused: `cmake-build-debug/build/Debug/tests/voyahslam_tests --gtest_filter='<Suite>*'`. All existing tests stay green (268 on master).
- Every new or changed header passes `clang++-18 -std=c++20 -stdlib=libc++ -fsyntax-only` (include/define flags from `cmake-build-debug/build/Debug/tests/CMakeFiles/voyahslam_tests.dir/flags.make`).
- New `src/*.cpp` files go into `src/CMakeLists.txt`; new tests into `tests/CMakeLists.txt`.

## Review Focus

1. A reference frame whose features are all above `max_reference_level` (e.g. SIFT with the default 0) — expected: no matches, no crash; the initializer simply never initializes. Test in Task 5 (`NoReferenceFeatureAtAllowedLevel`).
2. All grid points at one location (zero-extent bounding box) — expected: the grid still answers queries correctly, no division by zero. Test in Task 3 (`ZeroExtentBoundingBox`).
3. Pure rotation (no baseline) — expected: never initializes, no crash, no bogus map. Test in Task 6 (`PureRotationNeverInitializes`).
4. A binocular measurement arriving while a monocular reference is held — expected: the reference is dropped and the stereo stub initializes. Test in Task 6 (`BinocularDropsMonoReference`).
5. A frame with more than 65535 features (observation storage keeps `uint16_t` indices) — expected: a debug assertion, never a silently wrapped index. Assert in Task 7 (`assert(feature < 65536)` at each observation write); no test.

---

### Task 1: Keypoint pyramid levels

**Files:**
- Modify: `include/voyahslam/features/image_features.h`, `include/voyahslam/features/orb_feature_extractor.h`, `src/features/orb_feature_extractor.cpp`, `include/voyahslam/features/sift_feature_extractor.h`, `src/features/sift_feature_extractor.cpp`
- Modify (construction sites): `tests/support/test_measurements.h`, `tests/features/image_features_test.cpp`, `tests/features/monocular_measurement_test.cpp`, `tests/features/binocular_measurement_test.cpp`, `tests/map/frame_test.cpp`, any other `ImageFeatures(...)` call (`grep -rn "TFeatures(\|ImageFeatures<" src tests examples vizualization`)
- Test: `tests/features/image_features_test.cpp`, new `tests/features/feature_extractor_levels_test.cpp`

**Interfaces:**
- Produces: namespace-scope `features::TLevels = Eigen::Matrix<uint8_t, 1, Eigen::Dynamic>` in `image_features.h`; `ImageFeatures(points, strengths, angles, descriptors, TLevels&& levels)`; `const TLevels& levels() const`; `KeepFeatures` compacts levels.
- Produces: `double OrbFeatureExtractor::ScaleFactor(uint8_t level) const` (= `scale_factors_[level]`), `size_t OrbFeatureExtractor::num_levels() const`; `double SIFTFeatureExtractor::ScaleFactor(uint8_t level) const` (= `2^(level/3) / 2`), `static uint8_t SIFTFeatureExtractor::LevelFromOctave(int packed_octave)`.

- [ ] **Step 1: Write failing tests**
  - `ImageFeaturesTest.ConstructorAndGetters` also checks `levels()` round-trips.
  - `MonocularMeasurementTest.DropKeepsLevelsAligned`: features with levels `{0,1,2,3}` where feature 1 lies beyond 85° → `levels() == {0,2,3}` and still aligned with `points()`.
  - `OrbLevelsTest.LevelsInRangeAndScaleFactor`: `OrbFeatureExtractor` (defaults 1.2, 8) on a 512×512 seeded random-noise image smoothed with `images::GaussianBlur` (or a checkerboard), 1000 features → `levels().cols() == points().cols()`, every level < 8, at least two distinct levels, `ScaleFactor(0) == 1`, `ScaleFactor(3) == 1.2^3` (1e-12).
  - `SiftLevelsTest.LevelFromOctave`: packed octave = `(layer << 8) | (octave & 0xFF)`; octave −1, layer 1 → 1; octave 0, layer 2 → 5; octave 2, layer 3 → 12. `ScaleFactor(3) == 1.0`, `ScaleFactor(6) == 2.0`, `ScaleFactor(0) == 0.5`.
  - `SiftLevelsTest.DetectAndExtractRespectsCountAndRadians`: same image, `number_of_features = 50` → exactly `min(50, detected)` columns in points, angles, descriptors and levels; all angles in [0, 2π).

- [ ] **Step 2: Run** `voyahslam_tests --gtest_filter='ImageFeatures*:MonocularMeasurement*:OrbLevels*:SiftLevels*'` — expected: compile failure (no `levels()`).

- [ ] **Step 3: Implement.**
  - `ImageFeatures`: add `TLevels levels_` and the fifth constructor argument; `KeepFeatures` compacts it.
  - ORB: record each point's pyramid level in `ComputeKeyPointsOctTree`; add `ScaleFactor` and `num_levels`.
  - SIFT: decode `cv::KeyPoint::octave` with `LevelFromOctave`: octave = low byte as `int8_t`, layer = `(packed >> 8) & 0xFF`, level = (octave + 1) × 3 + layer. Fix two existing defects in the same function. Ruling: the spec requires radians, and the current code leaves columns uninitialized when more than `number_of_features` are detected.
    - Convert `angle` from degrees to radians.
    - Keep the `min(number_of_features, detected)` strongest keypoints, sorted by `response` descending. Size every array, descriptors included, to that count.
  - Update every construction site. Test helpers pass `TLevels::Zero(n)`.

- [ ] **Step 4: Run** the focused filter, then the full suite — expected: all pass.

- [ ] **Step 5: Commit** `git commit -m "Keypoint pyramid levels in ImageFeatures; per-extractor scale"`.

### Task 2: Migrate `TPose` to `RigidTransformation`

**Files:**
- Modify: `include/voyahslam/defs.h` (remove `TPose`), `include/voyahslam/map/frame.h`, `include/voyahslam/events/{key_frame_created,key_frame_pose_updated,position_updated,new_key_frame_candidate,map_created}.h`, `include/voyahslam/tracking/{pose_estimator,tracker}.h`, `src/tracking/pose_estimator.cpp`, `include/voyahslam/mapping/map_initializer.h`, `src/mapping/map_initializer.cpp`
- Modify tests: `tests/map/frame_test.cpp`, `tests/mapping/map_initializer_test.cpp`, `tests/tracking/{pose_estimator,tracker}_test.cpp`

**Interfaces:**
- Produces: every pose field and parameter is a `geometry::RigidTransformation` (`#include <voyahslam/geometry/rigid_transformation.h>`). Field names stay the same (`pose`), with one exception: `InitialKeyFrame::pose` becomes `T_cw`. `Frame::pose()` / `mutable_pose()` return `RigidTransformation`, and `Frame`'s class comment states "pose = T_cw (world → camera)".

- [ ] **Step 1: Change the types.** Event fields have no default reset, which is fine because producers set them on every publish. Tests compare poses with `a.ToMatrix34().isApprox(b.ToMatrix34())`. `NonIdentityPose()` helpers build `RigidTransformation(R, t)` with non-trivial R and t.
- [ ] **Step 2: Run** the full suite — expected: all pass with the same test count. `grep -rn TPose include src tests examples vizualization` — expected: no matches.
- [ ] **Step 3: Commit** `git commit -m "Migrate TPose to RigidTransformation; keyframe poses are T_cw"`.

### Task 3: `FeatureGrid`

**Files:**
- Create: `include/voyahslam/features/feature_grid.h`, `src/features/feature_grid.cpp`
- Test: `tests/features/feature_grid_test.cpp`

**Interfaces:**
- Consumes: `features::TLevels` (Task 1).
- Produces:
  ```cpp
  class FeatureGrid {
  public:
      FeatureGrid(const geometry::TPoints2DF& points, const TLevels& levels, int cols = 64, int rows = 48);
      [[nodiscard]] std::vector<Eigen::Index> FeaturesInArea(const geometry::TPoint2DF& center, double radius,
                                                             int min_level, int max_level) const;
  };
  ```
  Square window: `|dx| <= radius && |dy| <= radius`. Order of the returned indices is unspecified.

- [ ] **Step 1: Write failing tests** (compare as sorted vectors against a brute-force loop):
  - `FeatureGridTest.MatchesBruteForce`: 2000 seeded random points in [−200, 700]² (outside a 512² image too), levels random in [0, 7]; 50 random centres × radii {1, 10, 100}; levels [0, 7].
  - `FeatureGridTest.LevelFilter`: same data, window [2, 3] → only levels 2–3.
  - `FeatureGridTest.ZeroExtentBoundingBox`: 10 points all at (5, 5) → query (5, 5), r = 0 returns all 10; query (7, 7), r = 1 returns none.
  - `FeatureGridTest.EmptyInput`: 0 points → empty for any query.
- [ ] **Step 2: Run** `--gtest_filter='FeatureGrid*'` — expected: compile failure.
- [ ] **Step 3: Implement.** Uniform cells over the points' bounding box. Use a minimum cell size of 1e-6 so a zero extent works. Clamp the query's cell range to the grid. Check the exact distance and the level inside the cells.
- [ ] **Step 4: Run** — expected: pass; full suite green.
- [ ] **Step 5: Commit** `git commit -m "FeatureGrid: area queries over undistorted keypoints"`.

### Task 4: Synthetic monocular sequence (test support)

**Files:**
- Create: `tests/support/synthetic_sequence.h` (header-only, `namespace voyahslam::test`)
- Test: `tests/support/synthetic_sequence_test.cpp`

**Interfaces:**
- Consumes: `test::TestExtractor` (ORB), `features::TLevels`, `geometry::RigidTransformation`.
- Produces:
  ```cpp
  struct SyntheticSequenceOptions {
      size_t num_points = 300;
      size_t num_frames = 30;
      double step = 0.05;            // camera centre moves +x per frame (world units)
      double yaw_step_deg = 0.2;     // rotation about camera y per frame
      double min_depth = 4.0, max_depth = 8.0;
      double pixel_noise = 0.5;      // Gaussian σ, px
      int bit_flips = 3;             // per point per frame
      double angle_noise = 0.02;     // rad
      size_t num_extra = 20;         // unmatched random features per frame
      double frame_interval = 0.05;  // s
      uint32_t seed = 1;
  };
  struct SyntheticFrame {
      features::TMonocularMeasurement<TestExtractor> measurement;
      geometry::RigidTransformation T_cw;
      std::vector<Eigen::Index> point_of_feature;  // per feature: scene point index, −1 for extras
  };
  struct SyntheticSequence {
      geometry::TPoints3DF points_w;  // world = frame 0's camera
      std::vector<SyntheticFrame> frames;
  };
  geometry::TMatrix33F SyntheticIntrinsics();  // fx = fy = 190, cx = cy = 256 (512×512 image)
  SyntheticSequence MakeSyntheticSequence(const SyntheticSequenceOptions& options = {});
  ```
- Generation:
  - Each scene point is a uniform pixel in [20, 492]² back-projected from frame 0 at a uniform depth in [min_depth, max_depth].
  - Frame k: centre `C_k = (k·step, 0, 0)`, `R_cw = Ry(k·yaw_step)`, `t = −R_cw·C_k`.
  - A point is kept in frame k iff it has z > 0 and its noisy projection falls inside [0, 512)².
  - Each point has a base descriptor (random bytes) with `bit_flips` random bits flipped per frame, and a base angle (uniform in [0, 2π)) plus noise.
  - Extras get random pixels, random descriptors and random angles. All levels are 0 and all strengths 1.
  - Features are shuffled per frame (`std::mt19937`, seeded). Timestamp = `k·frame_interval`. Distortion model: `nullptr`.

- [ ] **Step 1: Write the generator and tests:**
  - `SyntheticSequenceTest.ProjectionsMatchPoses`: with `pixel_noise = 0`, every non-extra feature's point equals `K·T_cw·X` projected (1e-9).
  - `SyntheticSequenceTest.DescriptorsStayClose`: same point, frames 0 and 5 → ORB `Distance` ≤ 2·`bit_flips`. Random extras vs points → > 50 for ≥ 95 %.
  - `SyntheticSequenceTest.Deterministic`: two calls with the same seed give identical points.
- [ ] **Step 2: Run** `--gtest_filter='SyntheticSequence*'` — expected: pass.
- [ ] **Step 3: Commit** `git commit -m "Test support: synthetic monocular sequence"`.

### Task 5: `InitializationMatcher`

**Files:**
- Create: `include/voyahslam/features/initialization_matcher.h` (header-only template)
- Test: `tests/features/initialization_matcher_test.cpp`

**Interfaces:**
- Consumes: `FeatureGrid` (Task 3), `levels()`/`angles()` (Task 1), synthetic sequence (Task 4).
- Produces:
  ```cpp
  struct InitializationMatcherOptions {
      double window_px = 100.0;
      double max_distance = 50.0;   // spec says uint32 50; double so SIFT (float L2) can configure it
      double nn_ratio = 0.9;
      bool check_orientation = true;
      size_t histogram_bins = 30;
      int max_reference_level = 0;
  };
  template <typename TFeatureExtractor>
  [[nodiscard]] std::vector<Eigen::Index> MatchForInitialization(
      const TMonocularMeasurement<TFeatureExtractor>& reference,
      const TMonocularMeasurement<TFeatureExtractor>& current,
      const FeatureGrid& current_grid,
      std::vector<geometry::TPoint2DF>& out_last_matched,
      const InitializationMatcherOptions& options);
  ```
  The result has size = #reference features. Entry i is the matched current index, or −1. `out_last_matched` must have one entry per reference feature.
- Algorithm: spec §2, steps 1–5, verbatim. Rotation bin = `round(rot · bins / 2π) % bins`, with rot = `angle_ref − angle_cur` wrapped to [0, 2π). Three maxima as in ORB-SLAM3 `ComputeThreeMaxima`: drop the 2nd/3rd bins when their count is below 0.1 × the largest.

- [ ] **Step 1: Write failing tests.** Hand-built measurements use `SyntheticIntrinsics()`, `nullptr` distortion, chosen descriptors, levels and angles.
  - `WindowExcludesFarCandidates`: a perfect-descriptor candidate 150 px from `last_matched` → −1. At 50 px → matched.
  - `RatioTestRejectsAmbiguous`: two candidates at distances 10 and 11 → −1 (10 ≥ 0.9·11). At 10 and 20 → matched.
  - `RejectsAboveMaxDistance`: best distance 51 → −1.
  - `LaterBetterMatchReplacesClaim`: references A then B. Both best-match current c, with A at distance 20 and B at 5 → A = −1, B = c.
  - `OrientationFilterDropsInconsistent`: 40 matches rotated by 0.1 rad and 1 match rotated by π → the π match becomes −1. With `check_orientation = false` it is kept.
  - `MaxReferenceLevel`: a level-1 reference feature is skipped when `max_reference_level = 0` and matched when it is 1 (the current candidate is also at level 1). A level-1 reference never matches a level-0 candidate.
  - `NoReferenceFeatureAtAllowedLevel`: all reference levels 1 with `max_reference_level = 0` → all −1, `out_last_matched` unchanged.
  - `SyntheticPrecision`: sequence frames 0 and 3, `last_matched` = frame-0 points → precision ≥ 0.95, judged by `point_of_feature`. Print recall with `RecordProperty` or `std::cout`. `out_last_matched[i]` equals the current point for every match.
- [ ] **Step 2: Run** `--gtest_filter='InitializationMatcher*'` — expected: compile failure.
- [ ] **Step 3: Implement.** Distances come from `TFeatureExtractor::Distance`, cast to double.
- [ ] **Step 4: Run** — expected: pass; full suite green; clang-18/libc++ syntax check of the header.
- [ ] **Step 5: Commit** `git commit -m "InitializationMatcher: ORB-SLAM3 SearchForInitialization"`.

### Task 6: Monocular `MapInitializer`

**Files:**
- Modify: `include/voyahslam/mapping/map_initializer.h`, `src/mapping/map_initializer.cpp`, `include/voyahslam/mapping/mapper.h` (options), `src/mapping/mapper.cpp` (consume the new result)
- Modify tests: `tests/mapping/map_initializer_test.cpp`, `tests/mapping/mapper_test.cpp` (mono tests), `tests/slam/slam_test.cpp` (e2e)

**Interfaces:**
- Consumes: Tasks 1–5; `geometry::TwoViewReconstructor`, `TwoViewOptions`, `TwoViewCorrespondences`.
- Produces: the spec §3 types, verbatim — `MapInitializerOptions`, `InitialKeyFrame<FE>{T_cw, measurement}`, `InitialMapPoint`, `InitializationResult<FE>{keyframes, map_points}`. Also `explicit MapInitializer(MapInitializerOptions options = {})`, `std::optional<InitializationResult<FE>> TryInitialize(TMeasurement)` and `Reset()`.
- `MapperOptions{ MapInitializerOptions initializer; }`, replacing `min_mono_init_interval`. `kMinInitFeatures` is removed.
- Mapper (this task only): build keyframes from `result.keyframes`. Treat an engaged result with zero keyframes as failure (TODO item).

- [ ] **Step 1: Write failing tests** (`MapInitializerTest`, using the synthetic sequence unless stated):
  - `MonoInitializesOnceParallaxSuffices`: feed frames in order. Frame 1 → nullopt. Some frame ≤ 20 → result. Record the first success index k ≥ 2. The result has 2 keyframes; keyframe 0 has `T_cw` = identity. Rotation error vs ground truth `T_c_k,c_0` < 1°. Translation direction error < 10° (seeded; lock-in documented).
  - `MedianDepthIsOne`: median of the `map_points` z values == 1 (1e-9). Each point's `feature_in_first` / `feature_in_second` indexes into the respective measurement, and the two features share `point_of_feature`. At least 50 points.
  - `FewFeaturesResetsReference`: frame 0, then a 50-feature `MakeMonocular` frame, then frame 3 → nullopt (frame 3 became the reference). Then later frames (≤ 25) → success, with keyframe 0 = frame 3's measurement (same timestamp).
  - `FewMatchesReplacesReference`: frame 0, then frame 3 of a different-seed sequence (unrelated descriptors) → nullopt. Then frame 3 of the original sequence → nullopt, because the reference is now the unrelated frame.
  - `PureRotationNeverInitializes`: `step = 0`, `yaw_step_deg = 0.5`, 30 frames → never succeeds.
  - `BinocularDropsMonoReference`: frame 0, then `MakeBinocular(200, …)` → 1 keyframe, identity, 0 map points. Then frame 3 → nullopt (no reference left).
  - Keep the existing binocular stub test, updated for the new types.
  - Mapper and Slam tests: replace the mono tests that rely on `min_mono_init_interval` with synthetic-sequence frames.
    - `MonoInitializesAfterInterval` becomes `MonoInitializesWithParallax`: frames 0..k → `KeyFrameCreated` ×2 + `MapCreated`.
    - `FewFeaturesReplacesReference` drives the Mapper the same way as the initializer test.
    - `EndToEndInitializeTrackLoseReinitialize`: track frames 0–14 → map 1. Track 2 more frames → `PositionUpdated` on map 1. Track `MakeMonocular(0, …)` → lost. Track frames 15–29 → map 2. Timestamps increase throughout.
- [ ] **Step 2: Run** `--gtest_filter='MapInitializer*:Mapper*:Slam*'` — expected: compile failure.
- [ ] **Step 3: Implement** spec §3, steps 1–5.
  - Use the reference's `K()(0,0)` for σ.
  - Correspondences use `normalized_points().topRows<2>()` columns.
  - A non-finite or non-positive median depth counts as failure.
  - Scaling: `T_cw` of keyframe 2 = `RigidTransformation(R_21, t_21 / z̃)`.
  - The examples need no change (they use default `MapperOptions`). Confirm they build.
- [ ] **Step 4: Run** the full suite — expected: green.
- [ ] **Step 5: Commit** `git commit -m "Monocular MapInitializer: match, reconstruct, scale by median depth"`.

### Task 7: Mapper stores and publishes the initial map

**Files:**
- Modify: `include/voyahslam/events/key_frame_created.h`, `include/voyahslam/mapping/mapper.h`, `src/mapping/mapper.cpp`
- Test: `tests/mapping/mapper_test.cpp`, `tests/slam/slam_test.cpp`

**Interfaces:**
- Consumes: `InitializationResult` (Task 6), `map::MapPointStorage<FE>`, `map::ObservationStorage<map::ObservationType::Monocular>`, `events::MapPointsAdded<FE>`.
- Produces:
  - `KeyFrameCreated<FE>` fields `std::vector<uint32_t> observed_features` and `std::vector<map::MapPointId> observed_points` (pairs, same length), plus `void Reset()` that clears both and keeps capacity.
  - `Mapper::num_map_points()` (atomic, readable from any thread, like `num_keyframes()`).
  - Mapper members `map::MapPointStorage<FE> map_points_`, `map::ObservationStorage<map::ObservationType::Monocular> observations_` and `map_points_added_pool_{256}`.

- [ ] **Step 1: Write failing tests** (`MapperTest`):
  - `MonoPublishesPointsKeyFramesThenMap`: order `MapPointsAdded`, `KeyFrameCreated`, `KeyFrameCreated`, `MapCreated`, all on map 1.
    - `MapPointsAdded`: `start_id == 0`, `count() == num_map_points() > 0`.
    - Each `KeyFrameCreated` has `observed_features.size() == observed_points.size() == count()`. Every `observed_points` id is < `count()`. Every `observed_features` index is < that keyframe's feature count.
    - Normals have unit length. The first keyframe's descriptor of each observation equals the point's descriptor.
    - Extend `Published` / `TakeCaptured` to copy the payloads needed.
  - `MapCreatedCarriesSecondKeyFramePose`: the `MapCreated` pose equals the second `KeyFrameCreated` pose (`ToMatrix34` isApprox).
  - `PointIdsRestartAfterTrackingLost`: mono init → `TrackingLost(1)` → `num_map_points() == 0` → mono init again from later frames → `MapPointsAdded` with `map_id == 2` and `start_id == 0`.
  - `StereoPublishesNoMapPoints`: existing stereo test order is unchanged (`KeyFrameCreated`, `MapCreated`) and the observation vectors are empty.
  - `PooledKeyFrameCreatedIsReset`: create, recycle, re-create from the pool → vectors empty.
  - E2E (`slam_test`): `ProtocolRecorder` also records `MapPointsAdded` map ids. The test asserts one for map 1 before `MapCreated(1)` and one for map 2.
- [ ] **Step 2: Run** `--gtest_filter='Mapper*:Slam*'` — expected: failure.
- [ ] **Step 3: Implement** spec §4.
  - Fresh storages per map: assign `= {}` on success and on `TrackingLost`.
  - Normal = normalized sum of `normalize(X − C_k)` over both keyframes, where `C_k = T_cw.Inverse().translation()`.
  - Observations: `(point, frame, feature)` for both keyframes, with `assert(feature < 65536)`.
  - Publish order follows the spec: `MapPointsAdded` (skipped when there are 0 points, i.e. stereo), then `KeyFrameCreated` with observations, then `MapCreated`.
  - Log line: `"{} -> {} (map {}, {} keyframes, {} points)"`.
- [ ] **Step 4: Run** the full suite — expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper: store initial map points and observations; publish MapPointsAdded"`.

### Task 8: Docs, sanitizers, apps smoke run

**Files:**
- Modify: `docs/DESIGN.md`, `docs/TODO.md`

- [ ] **Step 1: ThreadSanitizer.**
  - Scratch build with `-DCMAKE_CXX_FLAGS="-fsanitize=thread -O1 -g" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread" -DCMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE=PRE_TEST` (Debug Conan toolchain).
  - Run `setarch $(uname -m) -R <build>/tests/voyahslam_tests --gtest_filter='Slam*:Mapper*:Tracker*'`.
  - Expected: pass, no `WARNING: ThreadSanitizer`.
- [ ] **Step 2: clang-18 + libc++ syntax check** of all sources — expected: clean.
- [ ] **Step 3: Apps smoke run.**
  - Release build. Run `monocular_full_slam` on `/home/vahagn/Downloads/dataset-corridor1_1024_16` with `--rate=0 --max-frames=200`, and `binocular_full_slam` with `--max-frames=20`.
  - Expected: no crash. Record whether and at which frame `Initializing -> Mapping (map 1, 2 keyframes, N points)` appears. No assertion: tuning is deferred.
- [ ] **Step 4: Docs.**
  - `DESIGN.md`: the initialization pipeline (matcher → MapInitializer → scaling → publication order), the `T_cw` convention, observations on `KeyFrameCreated`, per-map point ids, keypoint levels and per-extractor scales, and the SIFT angles-in-radians fix. Add decisions-log rows dated 2026-10-04.
  - `TODO.md`:
    - Remove "Keep each keypoint's pyramid level", "Migrate TPose users", "Real MapInitializer (mono part)" and the "engaged-but-empty" note.
    - Add: Pangolin viewer (next, marked as before tuning); tuning on TUM-VI 512 plus a per-frame "dropped by 85°" statistic; BA after initialization; real stereo initialization; `OrbFeatureExtractor::Distance` returns `uint8_t`, so a 256-bit difference wraps to 0 (widen the return type); and the smoke-run outcome.
- [ ] **Step 5: Commit** `git commit -m "Docs: monocular initialization design and TODO"`.
