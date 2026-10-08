# Relocalization and Reference-Frame Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A short tracking loss no longer costs the map. The Tracker tries BoW reference-Frame tracking, then relocalizes against a BoW Frame database (P3P RANSAC on rays), and starts a new map only after a 5 s timeout. A motion-model inlier gate fixes the stereo drift found by the spike.

**Architecture:**
- **algs** gains `bow::FrameDatabase`, `features::SearchByBoW` (frame vs Frame) and `geometry::SolvePnPRansac` (Lambda Twist P3P).
- **Events:** the Mapper publishes each Frame's BoW data in `MapTransaction`, and the Tracker's replica keeps the BoW data plus the database.
- **Tracker:** gains the `RecentlyLost` state and `TrackingLost::final`.

**Tech Stack:** C++20, Eigen, GoogleTest, CMake/Conan.

**Spec:** `docs/superpowers/specs/2026-10-07-relocalization-design.md` (amended with the stereo-drift spike; its notes are in `.superpowers/spikes/2026-10-07-stereo-drift.md`).

## Global Constraints

- **Branch:** `feat/relocalization`. Commit per task. Never merge or push.
- **Naming:** Google style. `PascalCase` types and functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- **Layers:** common ← events ← algs ← slam. No OpenCV or g2o in public headers.
- **Concurrency:** the Tracker never mutates map data. Threads talk only via events. The vocabulary is immutable and shared read-only through `std::shared_ptr<const bow::Vocabulary<FE>>`.
- **No vocabulary:** behaviour is exactly as today. No RecentlyLost; a failure goes straight to Lost, which publishes `TrackingLost(final = true)`.
- **Residuals:** raw pixels with σ = sigma_px·1.2^level, and χ² 5.991 for mono.
- **Spec values:**
  - reference-Frame tracking: ratio 0.7, ≥ 15 matches, ≥ 10 inliers;
  - relocalization: ratio 0.75, ≥ 15 matches per candidate;
  - RANSAC: 300 iterations, confidence 0.99, ≥ 10 inliers, 5 iterations per candidate per round;
  - verification: success at ≥ 50 inliers; projection windows 10 / 3 with distance 100 / 64; the second pass runs when 30 < n < 50;
  - database query: `common ≥ 0.8 · max`; candidate groups: top 10 covisible Frames, kept at ≥ 0.75 · best;
  - `recently_lost_seconds` = 5.0, `min_map_frames_to_relocalize` = 10;
  - `min_motion_model_inliers` = 30.
- **Build and test:**
  - Debug tree: `cmake-build-debug/build/Debug`.
  - Fast loop: `--target check`. End of each task: `--target check_all`, the apps, and Release (`build/build/Release`).
  - cmake binary: `/home/vahagn/Programs/clion/clion-2025.1.3/bin/cmake/linux/x64/bin/cmake`.
  - Current total: 533 tests.
- **Header check:** clang-18 with libc++ `-fsyntax-only` on changed public headers, using only their library's and lower libraries' roots.
- **Docs:** DESIGN, TODO and ROADMAP change only in Task 8.
- **Smoke inputs:** dataset `/home/vahagn/Downloads/dataset-corridor1_1024_16`; vocabulary `/tmp/claude-1000/-home-vahagn-git-voyah-slam/5fb6bfcd-a778-4bfc-81f0-1361ffd0496e/scratchpad/ORBvoc.bin`.

## Review Focus

1. **Perceptual aliasing:** a measurement from an unmapped place must not relocalize into a wrong pose.
   - Expected: no candidate reaches 50 inliers; stay RecentlyLost.
   - Test: Task 7, `RelocalizationTest.UnmappedPlaceDoesNotRelocalize`.
2. **A candidate Frame removed by a transaction while RecentlyLost.**
   - Expected: the database no longer returns it and verification never touches a removed Frame.
   - Test: Task 3, `FrameDatabaseTest.RemovedFrameNeverReturned`; Task 7, `RelocalizationTest.SkipsFrameRemovedMidRecovery`.
3. **`TrackingLost(final = false)` reaching the Mapper or viewer in any state** (Initializing, Mapping, stale map).
   - Expected: the Mapper never clears on non-final; the viewer shows "recently lost" without dropping the scene.
   - Test: Task 6, `MapperTest.IgnoresNonFinalTrackingLost`, `SceneStateTest.NonFinalLostKeepsScene`.
4. **Empty or featureless measurements while RecentlyLost.**
   - Expected: an empty BoW vector, no candidates, no crash; the timer still advances by timestamp.
   - Test: Task 7, `RelocalizationTest.EmptyMeasurementsAdvanceTimeout`.
5. **PnP on degenerate input:** < 4 points, collinear points, points behind the camera, all outliers.
   - Expected: `success = false`, never NaN.
   - Test: Task 5, `PnpTest.DegenerateInputFailsCleanly`.

---

### Task 1: Motion-model inlier gate (spec §0)

**Files:**
- Modify: `libs/slam/include/voyahslam/tracking/tracker.h` (`TrackingOptions`), `libs/slam/src/tracking/tracker.cpp`
- Test: `libs/slam/tests/tracking/tracker_test.cpp`

**Interfaces:**
- **Produces:** `TrackingOptions::min_motion_model_inliers = 30`. A motion-model pose with fewer final inliers counts as failed, and the zero-motion fallback runs next (its minimum stays at `min_motion_inliers`, 10).

- [ ] **Step 1: Failing test** `TrackerTest.WeakMotionModelFallsBackToZeroMotion`. Use a synthetic sequence where the motion model gets 10–29 inliers, for example a sudden velocity change. The frame must be tracked with stage `ZeroMotion` (or later), not `MotionModel`.
- [ ] **Step 2: Run.** Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`. Then smoke-run (Release, `--vocabulary`) stereo 1000 twice and mono 400 once.
  - Expected: stereo ATE ≤ 0.04 m with one map in both runs; mono one map with ATE about 0.032 m.
  - Put the numbers in the report.
- [ ] **Step 5: Commit** `git commit -m "Tracker: motion-model inlier gate (30) falls back to zero motion"`.

### Task 2: BoW in `MapTransaction`; vocabulary to the Tracker

**Files:**
- Modify:
  - events: `libs/events/include/voyahslam/events/map_transaction.h`
  - Mapper: `libs/slam/src/mapping/{mapper.cpp,mapper_quality.cpp}`
  - replica: `libs/slam/include/voyahslam/tracking/map_replica.h`
  - Tracker: `libs/slam/include/voyahslam/tracking/tracker.h`, `libs/slam/src/tracking/tracker.cpp`
  - Slam: `libs/slam/include/voyahslam/slam/slam.h`, `libs/slam/src/slam/slam.cpp`
- Test: `libs/slam/tests/tracking/map_replica_test.cpp`, `libs/slam/tests/mapping/mapper_test.cpp`

**Interfaces:**
- **Produces:**
  - `NewFrame<FE>` gains `bow::BowVector bow; bow::FeatureVector features;`. They are filled by the Mapper when it has a vocabulary, empty otherwise, and cleared by `Reset()`.
  - `ReplicaKeyFrame<FE>` gains the same two fields, copied on apply.
  - `Tracker(TrackerOptions, std::shared_ptr<const bow::Vocabulary<FE>> vocabulary = nullptr)`.
  - `Slam` passes its vocabulary to both the Mapper and the Tracker.

- [ ] **Step 1: Failing tests:**
  - `MapperBowTest.TransactionsCarryBow`: with a tiny vocabulary, every `added_frames` entry has a non-empty `bow`, equal to the Mapper's own.
  - `MapReplicaTest.StoresFrameBow`
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "MapTransaction carries Frame BoW; Tracker gets the vocabulary"`.

### Task 3: `bow::FrameDatabase` and its replica integration

**Files:**
- Create: `libs/algs/include/voyahslam/bow/frame_database.h`, `libs/algs/tests/bow/frame_database_test.cpp`
- Modify: `libs/slam/include/voyahslam/tracking/map_replica.h` (owns a `FrameDatabase`, updated on added and removed Frames, cleared on a new map)
- Test: also `libs/slam/tests/tracking/map_replica_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  struct DatabaseHit { map::FrameId frame; uint32_t common_words; double score; };
  class FrameDatabase {
  public:
      void Add(map::FrameId frame, const BowVector& bow);
      void Remove(map::FrameId frame);
      void Clear();
      // Frames sharing ≥ 1 word; keep common ≥ min_common_ratio · max common; score = Vocabulary::Score; sorted by score desc.
      [[nodiscard]] std::vector<DatabaseHit> Query(const BowVector& bow, double min_common_ratio = 0.8) const;
  };
  // MapReplica: const bow::FrameDatabase& database() const;
  ```
  - `FrameDatabase` keeps each Frame's `BowVector`, so it can score and remove.
  - `map::FrameId` comes from common, so algs including common is legal.

- [ ] **Step 1: Failing tests:**
  - `FrameDatabaseTest.QueryRanksByScore`
  - `FrameDatabaseTest.CommonWordThreshold`: with max common 10, a Frame with 7 shared words is excluded and one with 8 is kept.
  - `FrameDatabaseTest.RemovedFrameNeverReturned` (Review Focus 2)
  - `FrameDatabaseTest.EmptyQueryReturnsNothing`
  - `MapReplicaTest.DatabaseFollowsTransactions`: added and removed Frames are reflected, and `new_map` clears.
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "bow::FrameDatabase (inverted index); replica keeps it in sync"`.

### Task 4: `features::SearchByBoW` (frame vs Frame)

**Files:**
- Create: `libs/algs/include/voyahslam/features/bow_matcher.h`, `libs/algs/tests/features/bow_matcher_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  struct BowMatchOptions { int th_low = 50; double nn_ratio = 0.75; bool check_orientation = true; size_t histogram_bins = 12; };
  // frame_points[i]: the live MapPointId observed by the Frame's feature i, or nullopt.
  // Returns per current feature: matched MapPointId or nullopt. One-to-one over Frame features.
  template <typename FE> std::vector<std::optional<map::MapPointId>> SearchByBoW(
      const features::TMonocularMeasurement<FE>& frame_measurement, const bow::FeatureVector& frame_features,
      const std::vector<std::optional<map::MapPointId>>& frame_points,
      const features::TMonocularMeasurement<FE>& current, const bow::FeatureVector& current_features,
      const BowMatchOptions& options);
  ```
  - It walks the two sorted node lists side by side.
  - For each Frame feature that has a point, it finds the best and second-best current feature under the same node.
  - It reuses `RotationHistogram`, keeping the top 3 bins.

- [ ] **Step 1: Failing tests:**
  - `BowMatcherTest.MatchesOnlySharedNodes`
  - `BowMatcherTest.RatioTestRejectsAmbiguous`
  - `BowMatcherTest.OneToOne`
  - `BowMatcherTest.SkipsFrameFeaturesWithoutPoints`
  - `BowMatcherTest.RotationHistogramDropsInconsistent`
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "features::SearchByBoW: frame-to-Frame BoW matching"`.

### Task 5: `geometry::SolvePnPRansac` (Lambda Twist P3P)

**Files:**
- Create: `libs/algs/include/voyahslam/geometry/pnp.h`, `libs/algs/src/geometry/pnp.cpp`, `libs/algs/tests/geometry/pnp_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  // Up to 4 poses T_cw mapping the 3 world points onto the 3 unit bearings (camera frame).
  std::vector<RigidTransformation> SolveP3P(const std::array<TPoint3DF,3>& world,
                                            const std::array<TPoint3DF,3>& bearings);
  struct PnpOptions { int max_iterations = 300; double confidence = 0.99; size_t min_inliers = 10; double chi2 = 5.991;
                      uint64_t seed = 0; };
  struct PnpResult { bool success = false; RigidTransformation T_cw; std::vector<bool> inlier; size_t num_inliers = 0; int iterations = 0; };
  // observations: raw pixels + sigma (px) per point; bearings computed via camera.Unproject.
  PnpResult SolvePnPRansac(const Camera& camera, const std::vector<TPoint3DF>& world,
                           const std::vector<TPoint2DF>& pixels, const std::vector<double>& sigmas,
                           const PnpOptions& options);
  ```
  - **Lambda Twist** follows Persson & Nordberg 2018, ECCV ("Lambda Twist: An Accurate Fast Robust Perspective Three Point (P3P) Solver"), as in their reference implementation.
  - **Sampling:** each RANSAC sample takes 3 points to generate candidates and a 4th to choose among them.
  - **Scoring:** the raw-pixel χ² through `camera.Project`. Depth ≤ 0 is an outlier.
  - **Iteration cap:** adaptive, `log(1−conf) / log(1−w³)` capped at `max_iterations`.
  - **Determinism:** a seeded `std::mt19937_64`.
  - **Final pose:** the best hypothesis's pose. Refinement is the caller's job, with `OptimizePose`.

- [ ] **Step 1: Failing tests:**
  - `PnpTest.P3PRecoversNoiseFreePose`: 3 points; one of the returned poses equals the truth within 1e-9.
  - `PnpTest.P3PAllSolutionsReproject`: every returned pose maps the 3 points onto the 3 bearings within 1e-9.
  - `PnpTest.RansacFisheyeNoiseFree`: KB TUM-VI camera, rays up to 80°, 100 points; the pose within 1e-6.
  - `PnpTest.RansacHalfOutliers`: 100 points, 50 % outliers, 1 px noise; within 1 cm and 0.5°, and the inlier mask is ≥ 90 % correct.
  - `PnpTest.DegenerateInputFailsCleanly` (Review Focus 5): 3 points, collinear points, all points behind the camera, all outliers. Each gives `success == false`, with a finite pose or the identity.
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "geometry::SolvePnPRansac: Lambda Twist P3P on rays, raw-pixel inliers"`.

### Task 6: Tracker states, `TrackingLost::final`, reference-Frame tracking

**Files:**
- Modify:
  - events: `libs/events/include/voyahslam/events/{tracking_lost.h,frame_tracked.h}` (`final`; `TrackingStage` values)
  - Tracker: `libs/slam/include/voyahslam/tracking/tracker.h`, `libs/slam/src/tracking/tracker.cpp`
  - Mapper: `libs/slam/src/mapping/mapper.cpp` (only `final` clears)
  - viewer: `vizualization/{include/voyahviz/scene_state.h,src/scene_state.cpp}` (a `RecentlyLost` status)
- Test: `libs/slam/tests/tracking/tracker_test.cpp`, `libs/slam/tests/mapping/mapper_test.cpp`, `vizualization/tests/scene_state_test.cpp`

**Interfaces:**
- **Consumes:** Task 2's vocabulary in the Tracker, Task 4's `SearchByBoW`, `OptimizePose`.
- **Produces:**
  - `TrackerState::RecentlyLost`; `TrackingLost::final`; `TrackingStage::{ReferenceFrame, Relocalization}`;
  - `TrackerOptions::relocalization { double recently_lost_seconds = 5.0; size_t min_map_frames_to_relocalize = 10; }`;
  - `SceneState` status `RecentlyLost`.
- **Flow:**
  - The motion model, zero-motion and reference-Frame tracking (spec §3) run in that order. Reference-Frame tracking is used only with a vocabulary.
  - On failure:
    - with a vocabulary and more than 10 Frames: RecentlyLost, publishing `TrackingLost(final = false)` and recording the timestamp;
    - otherwise: Lost, publishing `final = true`.
  - In RecentlyLost, each measurement calls `TryRelocalize(measurement)`. In this task it is a stub that returns failure; Task 7 implements it.
  - After `recently_lost_seconds` of measurement timestamps: Lost, publishing `final = true`.
  - No `FrameTracked` or `InitializationMeasurement` is published while RecentlyLost.

- [ ] **Step 1: Failing tests:**
  - `TrackerTest.ReferenceFrameRecoversJump`: with a vocabulary, a synthetic jump that defeats the motion model and zero-motion is tracked with stage `ReferenceFrame`.
  - `TrackerTest.FailureEntersRecentlyLostWithVocabulary`: `final == false`; nothing is published while RecentlyLost.
  - `TrackerTest.RecentlyLostTimesOutToLost`: after 5 s of timestamps, `final == true`.
  - `TrackerTest.SmallMapGoesStraightToLost`
  - `TrackerTest.NoVocabularyBehavesAsBefore`
  - `MapperTest.IgnoresNonFinalTrackingLost` (Review Focus 3): in Mapping and in Initializing, and for a stale map.
  - `SceneStateTest.NonFinalLostKeepsScene` (Review Focus 3)
- [ ] **Step 2: Run.** Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`, the apps, Release and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Tracker: RecentlyLost state, TrackingLost::final, reference-Frame tracking"`.

### Task 7: Relocalization

**Files:**
- Create: `libs/slam/include/voyahslam/tracking/relocalizer.h` (and a .cpp if needed), `libs/slam/tests/tracking/relocalization_test.cpp`
- Modify: `libs/slam/src/tracking/tracker.cpp` (`TryRelocalize` calls it)

**Interfaces:**
- **Consumes:** Task 3's `replica.database()`, the replica `CovisibilityIndex`, Task 4's `SearchByBoW`, Task 5's `SolvePnPRansac`, `OptimizePose`, and `SearchFromLocalMap` with a point source restricted to one Frame's points.
- **Produces:**
  ```cpp
  struct RelocalizationResult { bool success = false; RigidTransformation T_cw; map::FrameId frame; std::vector<int64_t> matches; size_t inliers = 0; };
  template <typename FE> RelocalizationResult Relocalize(const MapReplica<FE>& replica, const TMeasurement<FE>& measurement,
      const bow::Vocabulary<FE>& vocabulary, const RelocalizationOptions& options);
  ```
  Implement spec §4.2–4.5 exactly. On success the Tracker runs its local-map step and resumes Tracking, with the reference set to `frame`, the velocity reset and stage `Relocalization`.

- [ ] **Step 1: Failing tests:**
  - `RelocalizationTest.RelocalizesIntoSameMap`: build a map with a synthetic sequence, then black out with 10 empty measurements, then resume in the mapped area. Expect `Relocalization`, the same `map_id`, and a pose within 1 cm / 0.5° of the truth.
  - `RelocalizationTest.UnmappedPlaceDoesNotRelocalize` (Review Focus 1): measurements from an unseen synthetic scene stay RecentlyLost.
  - `RelocalizationTest.SkipsFrameRemovedMidRecovery` (Review Focus 2)
  - `RelocalizationTest.EmptyMeasurementsAdvanceTimeout` (Review Focus 4)
  - `RelocalizationTest.CandidateGroupingPrefersCovisibleCluster`: two candidate clusters; the one with the higher accumulated score is tried first.
- [ ] **Step 2: Run.** Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`, the apps and Release. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Tracker: BoW relocalization (database, SearchByBoW, P3P RANSAC, verification)"`.

### Task 8: Blackout smoke, integration and docs

**Files:**
- Modify:
  - apps: `examples/full_slam/{monocular_full_slam.cpp,binocular_full_slam.cpp,full_slam_common.h}` (`--drop_frames a:b`)
  - tests: `libs/slam/tests/integration/slam_test.cpp`
  - docs: `docs/DESIGN.md`, `docs/TODO.md`, `docs/ROADMAP.md`

**Interfaces:** `--drop_frames a:b` replaces frames a through b (inclusive, 0-based) with measurements that have no features, keeping their timestamps.

- [ ] **Step 1: Integration tests:** `SlamTest.{Mono,Stereo}RelocalizesAfterBlackout`.
  - A 1 s blackout in the mapped area ends in the same map.
  - The pose error is within a bound set at 1.5 × the maximum of 3 measured runs. Record the measurements in the commit message.
- [ ] **Step 2: ThreadSanitizer** on `voyah_slam_tests` and `voyah_integration_tests` (scratch TSan tree, `setarch -R`). Expected: no warnings.
- [ ] **Step 3: Smoke runs** (Release, `--vocabulary`, 2 runs each):

  | Run | Must | Report |
  |---|---|---|
  | Mono 400, `--drop_frames 250:270` | Relocalizes into the same map | recovery frame, ATE, re-initializations |
  | Stereo 1000, `--drop_frames 500:520` | Relocalizes into the same map, no other loss | same |
  | Mono 400 and stereo 1000, no drop | One map each; ATE ≤ 0.04 m stereo, about 0.032 m mono | Frames, ATE, relocalization and reference-Frame counts |

  A miss is DONE_WITH_CONCERNS with a diagnosis. Nothing is tuned.
- [ ] **Step 4: Docs** (spec §6):
  - **`DESIGN.md`:** the state machine, `TrackingLost::final`, BoW in `MapTransaction`, `FrameDatabase`, relocalization and PnP, the motion-model gate and the spike result, the smoke table, and decisions-log entries dated 2026-10-08.
  - **`TODO.md`:** relocalization into other maps (step 5), MLPnP as an alternative, and the remove the stale stereo-drift item.
  - **`ROADMAP.md`:** step 3 done.
- [ ] **Step 5: Run** `check_all`, the apps and Release. Expected: green.
- [ ] **Step 6: Commit** `git commit -m "Relocalization: blackout smoke, integration tests, docs"`.
