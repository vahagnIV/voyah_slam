# Map Quality Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Mapper keeps the map healthy as it grows: fusion, recent-point culling, Frame culling and interruptible local BA. Every Mapper change is published as one atomic `MapTransaction`.

**Architecture:**
- `MapTransaction<FE>` replaces every Mapper event. All consumers (replica, viewer, Tracker) apply it in a fixed order.
- The new algorithms are pure functions in `voyah_algs` that return decisions: `FuseMatches`, the culling rules, and `BundleAdjust` extensions.
- The Mapper (`voyah_slam`) applies the decisions and publishes two transactions per created Frame.
- An urgent `FrameTracked` stops local BA through a generic priority counter in the event inbox.

**Tech Stack:** C++20, Eigen, g2o (PRIVATE in algs), GoogleTest, CMake/Conan; Python 3 + numpy for `tools/ate.py`.

**Spec:** `docs/superpowers/specs/2026-10-07-map-quality-design.md`

## Global Constraints

- **Branch:** `feat/map-quality`. Commit per task. Never merge or push.
- **Naming:** Google style. `PascalCase` types and functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- **Layers:** common ← events ← algs ← slam. A lower layer never includes an upper one, and the separate include roots enforce this. No g2o or OpenCV in public headers.
- **Concurrency:** no locks on map data. The Mapper is the only writer, and threads communicate only via events. The urgent counter is event infrastructure, not map data.
- **Ids:** never reused. Frame ids are unique per run; point ids are append-only per map.
- **Residuals:**
  - Raw pixels, σ = `sigma_px · 1.2^level`.
  - χ² 5.991 for mono observations and 7.815 for stereo.
- **Events:** pooled and ref-counted. `Reset()` clears every part and keeps capacity. Producers set `map_id`.
- **Build and test:**
  - Debug tree: `cmake-build-debug/build/Debug`.
  - Fast loop: `cmake --build cmake-build-debug/build/Debug --target check` (unit tests).
  - End of each task: `--target check_all`, and also build the apps (`cmake --build cmake-build-debug/build/Debug`).
  - Release tree: `build/build/Release` must build.
  - cmake binary: `/home/vahagn/Programs/clion/clion-2025.1.3/bin/cmake/linux/x64/bin/cmake`.
  - Starting total: 459 tests.
- **Header check:** clang-18 with libc++, `-std=c++20 -fsyntax-only`, on changed public headers, using only the include roots of that library and the libraries below it.
- **Docs:** `docs/DESIGN.md`, `docs/TODO.md` and `docs/ROADMAP.md` change only in Task 8.

## Review Focus

1. **A transaction removes a point that the Tracker's `last_frame_` or local map still references.**
   - Expected: no access to dead data. Replaced points are remapped; removed ones are dropped from the matches.
   - Test: Task 1, `TrackerTest.RemovedAndReplacedPointsInLastFrame`.
2. **Frame culling removes the Frame that holds a point's reference observation, or the Tracker's reference Frame.**
   - Expected: the reference moves to the smallest remaining Frame id on both sides, and `FrameTracked::reference_frame_id` never names a removed Frame.
   - Tests: Task 1, `MapReplicaTest.ReferenceReassignedOnFrameRemoval`; Task 7, `MapperQualityTest.CullsFrameHoldingReferences`.
3. **The same feature matched to two different points within one fusion pass** (a chain A→B, B→C).
   - Expected: each merge resolves to the final winner, with no dangling ids in the transaction.
   - Test: Task 4, `FuseMatchesTest.ChainedDuplicatesResolveToOneWinner`; Task 6, `MapperFusionTest.ChainedMergesPublishFinalWinner`.
4. **A local BA problem with no free Frame,** for example when every covisible Frame is fixed or the map has only its first Frame.
   - Expected: BA is skipped, not run on a degenerate problem, and transaction 2 still publishes.
   - Test: Task 7, `MapperQualityTest.SkipsBaWithoutFreeFrames`.
5. **The urgent flag set while no BA is running, or several urgent events queued.**
   - Expected: the counter never goes negative and BA is not pre-stopped by stale counts.
   - Test: Task 2, `AsyncObserverTest.PriorityCountTracksPushPop`.

---

### Task 1: `MapTransaction` — one Mapper event, applied by every consumer

**Files:**
- Create: `libs/events/include/voyahslam/events/map_transaction.h`
- Delete: `libs/events/include/voyahslam/events/{key_frame_created,map_created,map_points_updated,map_points_removed,key_frame_pose_updated}.h`
- Modify:
  - events: `event_types.h`, `events.h`
  - slam: `libs/slam/include/voyahslam/tracking/{map_replica.h,tracker.h}`, `libs/slam/src/tracking/tracker.cpp`, `libs/slam/{include/voyahslam,src}/mapping/mapper.*`
  - common: `libs/common/include/voyahslam/map/covisibility_index.h`, `libs/common/src/map/covisibility_index.cpp`
  - viewer: `vizualization/{include/voyahviz/scene_state.h,src/scene_state.cpp}`
  - every test that names the deleted events
- Test: `libs/slam/tests/tracking/map_replica_test.cpp`, `libs/slam/tests/tracking/tracker_test.cpp`, `libs/slam/tests/mapping/mapper_test.cpp`, `libs/common/tests/map/covisibility_index_test.cpp`, `vizualization/tests/scene_state_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  // events/map_transaction.h
  struct TransactionObservation { map::FrameId frame; uint32_t feature; map::MapPointId point;
                                  precision_t depth = -1; int16_t right_index = -1; };
  struct RemovedObservation { map::FrameId frame; map::MapPointId point; };
  struct ReplacedPoint { map::MapPointId loser; map::MapPointId winner; };
  template <typename FE> struct NewFrame { map::FrameId id; geometry::RigidTransformation T_cw;
      features::TMeasurement<FE> measurement; std::vector<uint32_t> observed_features;
      std::vector<map::MapPointId> observed_points; };
  template <typename FE> struct MapTransaction : Event, memory::Recyclable<...> {
      bool new_map = false;  std::optional<map::FrameId> start_frame;
      std::vector<NewFrame<FE>> added_frames;
      map::MapPointId new_points_start_id{};  ColumnBuffer<precision_t,3> new_positions, new_normals;
      ColumnBuffer<desc> new_descriptors;  void ResizeNewPoints(uint32_t);  uint32_t num_new_points() const;
      std::vector<TransactionObservation> added_observations;
      std::vector<map::MapPointId> updated_ids;  ColumnBuffer<precision_t,3> updated_positions, updated_normals;
      ColumnBuffer<desc> updated_descriptors;  void ResizeUpdatedPoints(uint32_t);
      std::vector<RemovedObservation> removed_observations;
      std::vector<ReplacedPoint> replaced_points;
      std::vector<map::MapPointId> removed_points;
      std::vector<std::pair<map::FrameId, geometry::RigidTransformation>> updated_poses;
      std::vector<map::FrameId> removed_frames;
      void Reset();
  };
  // EventType: + MapTransaction; − KeyFrameCreated, MapCreated, MapPointsUpdated, MapPointsRemoved,
  //            KeyFramePoseUpdated. TrackingLost stays the last enumerator.
  // CovisibilityIndex gains:
  void Remove(FrameId frame, MapPointId point);   // no-op if absent
  void RemovePoint(MapPointId point);             // drops all its observations
  void RemoveFrame(FrameId frame);                // drops all its observations
  // MapReplica gains:
  [[nodiscard]] bool CheckConsistency() const;    // spec §2 invariant; for tests
  ```
- **Apply order** (spec §2): new map, then add Frames, add points, add observations, update points, update poses, remove observations, remove points, remove Frames.
  - **Replaced points:** the loser is removed in the "remove points" step.
  - **References:** when a point's reference observation is removed, its reference moves to the remaining observation with the smallest Frame id. If none remains, the point is removed.
- **Mapper (behaviour-preserving):**
  - Initialization publishes one transaction: `new_map`, `start_frame` = the last initial Frame, all Frames and all points.
  - `CreateFrame` publishes one transaction: the Frame, new points, added observations and updated points.
- **Tracker:**
  - `new_map` replaces the `MapCreated` handling.
  - Every transaction's `replaced_points` remap `last_frame_.matches` from loser to winner.
  - `removed_points` set those matches to −1.
  - The "own Frame merge" (matching timestamp) now reads `added_frames`.

- [ ] **Step 1: Write failing tests:**
  - `CovisibilityIndexTest.RemoveVariants`: `Remove`, `RemovePoint` and `RemoveFrame` update both directions; `Covisible` drops counts accordingly.
  - `MapReplicaTest.AppliesEveryTransactionPart`: one transaction exercising all parts. Afterwards `CheckConsistency()` holds, removed points are not alive, removed Frames are absent, and updated poses and points have their new values.
  - `MapReplicaTest.ReferenceReassignedOnFrameRemoval` (Review Focus 2): a point referenced in Frame 3 and also observed by Frames 5 and 7. Removing Frame 3 moves its reference to Frame 5, with the level and distance recomputed from Frame 5's feature and pose.
  - `MapReplicaTest.PointWithoutObservationsIsRemoved`
  - `TrackerTest.RemovedAndReplacedPointsInLastFrame` (Review Focus 1): after a transaction that replaces 4→9 and removes 5, the matches of `last_frame_` hold 9 where they held 4, and −1 where they held 5. The next frame tracks without failing.
  - Migrate `MapperTest`, `TrackerTest` and `SlamTest` assertions to transactions. Mono init gives exactly one transaction with `new_map`, 2 Frames and all points; stereo init gives one transaction with 1 Frame.
  - `SceneStateTest.AppliesTransactionRemovals`: points, Frames and poses.
- [ ] **Step 2: Run** `check`. Expected: compile failures (new type).
- [ ] **Step 3: Implement.** Migrate every producer and consumer, then delete the five old event headers.
- [ ] **Step 4: Run** `check_all`, build the apps, build Release, run the header check. Expected: green. The test total is 459 plus the new tests.
- [ ] **Step 5: Commit** `git commit -m "MapTransaction: one atomic Mapper event applied by replica, Tracker and viewer"`.

### Task 2: Urgent `FrameTracked` and the inbox priority counter

**Files:**
- Modify: `libs/events/include/voyahslam/events/{event_types.h,async_observer.h}`, `libs/events/include/voyahslam/events/frame_tracked.h`, `libs/slam/include/voyahslam/tracking/tracker.h`, `libs/slam/src/tracking/tracker.cpp`
- Test: `libs/events/tests/events/event_system_test.cpp`, `libs/slam/tests/tracking/tracker_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  // Event base: bool priority = false;  (producers set it per publish; pooled events are not reset by the base)
  // AsyncObserver:
  [[nodiscard]] uint32_t pending_priority() const;   // atomic; +1 on push of a priority event, −1 when popped
  [[nodiscard]] const std::atomic<uint32_t>& pending_priority_counter() const;
  // FrameTracked: bool urgent (mirrors Event::priority)
  // TrackingOptions: double urgent_inlier_factor = 1.5;
  ```
- The Tracker sets `urgent` and `priority` when `num_inliers < urgent_inlier_factor × min_local_map_inliers`, that is below 45 with the defaults.

- [ ] **Step 1: Write failing tests:**
  - `AsyncObserverTest.PriorityCountTracksPushPop` (Review Focus 5): push 3 events (2 priority), pop all → the counter goes 2 → 0 and never below. Pushing a non-priority event leaves it unchanged. A pooled event reused with `priority` reset to false is not counted.
  - `TrackerTest.UrgentWhenInliersNearGate`: a frame with 40 inliers is urgent; one with 60 is not.
- [ ] **Step 2: Run** the filter. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Urgent FrameTracked; inbox priority counter"`.

### Task 3: `BundleAdjust` — stereo edges and interruption

**Files:**
- Modify: `libs/algs/include/voyahslam/optimization/bundle_adjustment.h`, `libs/algs/src/optimization/bundle_adjustment.cpp`, `libs/algs/src/optimization/projection.h` (if a stereo helper is needed)
- Test: `libs/algs/tests/optimization/bundle_adjustment_test.cpp`

**Interfaces:**
- **Consumes:** `optimization::StereoCameras` and the right-image projection helpers already used by `OptimizePose` (`ProjectPixelRight`, `PixelJacobianRightPose`).
- **Produces:**
  ```cpp
  struct BaObservation { …; std::optional<double> right_x; };              // raw px in the right image
  struct BaProblem { …; std::vector<std::optional<StereoCameras>> keyframe_stereo; }; // empty = all mono
  struct BundleAdjustmentOptions { …; double stereo_chi2 = 7.815; const std::atomic<bool>* stop = nullptr; };
  struct BaResult { …; bool interrupted = false; int stage_reached = 2; };    // 0 robust, 1 plain, 2 finished
  ```
- **Stereo edge:** the residual is (u_l, v_l, u_r) in raw pixels. `chi2` and `inlier` use `stereo_chi2` for stereo edges.
- **Stopping:** the stop flag is checked after every LM iteration (g2o `setForceStopFlag` with a per-iteration action, or `optimize(1)` in a loop). Stage-1 outlier deactivation runs only if stage 0 completed.

- [ ] **Step 1: Write failing tests:**
  - `BundleAdjustmentTest.StereoEdgeJacobianMatchesFiniteDifference` (1e-6 relative).
  - `BundleAdjustmentTest.StereoRecoversPerturbedNoiseFree`: a stereo rig, perturbed poses and points; recovered within 1e-6.
  - `BundleAdjustmentTest.PreSetStopFlagInterruptsAtStageZero`: `interrupted`, `stage_reached == 0`, poses equal to the input within 1e-12, `success` true.
  - `BundleAdjustmentTest.StopFlagSetMidRunStopsWithinOneIteration`: the flag is set from a per-iteration callback after iteration 2, and `iterations ≤ 3`.
  - The existing init BA tests are unchanged: `PinholeInvariance`, `FisheyeRecoversPerturbedNoiseFree`, `MapInitializerTest.BundleAdjustmentReachesOptimum`.
- [ ] **Step 2: Run** `--gtest_filter='BundleAdjust*'` on `voyah_algs_tests`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "BundleAdjust: stereo edges, stop flag, stage reporting"`.

### Task 4: `FuseMatches`

**Files:**
- Create: `libs/algs/include/voyahslam/features/fuse_matcher.h`, `libs/algs/tests/features/fuse_matcher_test.cpp`

**Interfaces:**
- **Consumes:** `features::detail::ProjectionGate`, `features::FeatureGrid`, `geometry::Camera`, `map::PointReference`.
- **Produces:**
  ```cpp
  struct FuseOptions { double radius_factor = 3.0; double chi2_mono = 5.991; double chi2_stereo = 7.815;
                       int th_low = 50; double scale_slack = 0.2; double min_view_cos = 0.5;
                       double max_ray_angle_deg = 85.0; double sigma_px = 1.0; double level_scale = 1.2; int max_level = 7; };
  struct FuseCandidatePoint { map::MapPointId id; geometry::TPoint3DF position; geometry::TPoint3DF normal;
                              typename FE::TDescriptor descriptor; map::PointReference reference; };  // template on FE
  struct FuseDecision { map::MapPointId point; uint32_t feature;
                        std::optional<map::MapPointId> existing; };   // existing = point the feature already has
  template <typename FE> std::vector<FuseDecision> FuseMatches(
      const features::TMonocularMeasurement<FE>& frame_left, const geometry::RigidTransformation& T_cw,
      const std::optional<features::StereoView>& stereo,          // right distorted points + right indices + rig; nullopt for mono
      const std::vector<std::optional<map::MapPointId>>& feature_points,  // what each feature observes now
      const std::vector<FuseCandidatePoint<FE>>& points, const FuseOptions& options);
  ```
  - At most one decision per point.
  - Points already observed by the frame (`feature_points` contains the id) are skipped.
  - The gate follows spec §3.
  - If a `StereoView` type doesn't exist, define it minimally in the same header: `{const TPoints2DF* right_points; const Eigen::VectorXi* right_indices; StereoCameras cameras;}`.

- [ ] **Step 1: Write failing tests** (`FuseMatchesTest.*`, synthetic pinhole and KB fisheye):
  - `AddsObservationForFreeFeature`
  - `ReportsExistingForDuplicate` (the decision's `existing` is set)
  - `RejectsBehindOutsideAngleScaleChi2Descriptor`: one case per gate.
  - `SkipsPointsAlreadyObserved`
  - `ChainedDuplicatesResolveToOneWinner` (Review Focus 3, at the algorithm level): two candidate points that would claim the same feature give exactly one decision for that feature, the closer descriptor winning.
  - `StereoUsesRightResidual`: a stereo match off by 5 px in the right image only is rejected.
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "FuseMatches: projection-gated duplicate and observation search"`.

### Task 5: Culling rules as pure functions

**Files:**
- Create: `libs/algs/include/voyahslam/mapping/map_quality.h`, `libs/algs/src/mapping/map_quality.cpp`, `libs/algs/tests/mapping/map_quality_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  struct MapQualityOptions {
      size_t fuse_neighbours_mono = 30, fuse_neighbours_stereo = 10, fuse_second_order = 20;
      size_t recent_window = 3; double min_found_ratio = 0.25;
      size_t min_obs_mono = 2, min_obs_stereo = 3;          // removed if weighted obs <= this after 2 Frames
      double redundancy_ratio = 0.9; size_t redundancy_observers = 3;
      size_t ba_min_shared = 15, ba_max_optimized = 20; int ba_robust_iterations = 5, ba_iterations = 10;
  };
  struct RecentPointStats { size_t frames_since_creation; uint32_t visible; uint32_t found; size_t weighted_obs; bool stereo; };
  enum class RecentPointVerdict { Keep, Remove, Graduate };
  RecentPointVerdict JudgeRecentPoint(const RecentPointStats& s, const MapQualityOptions& o);
  struct FramePointStats { int level; bool counted; size_t other_observers_at_level; };  // counted=false: stereo far point
  bool IsRedundantFrame(const std::vector<FramePointStats>& points, const MapQualityOptions& o);
  ```
- **`JudgeRecentPoint` order:**
  1. `visible > 0 && found / visible < 0.25` → Remove.
  2. `frames_since_creation >= 2` and the weighted observation count is ≤ the threshold → Remove.
  3. `frames_since_creation >= recent_window` → Graduate.
  4. Otherwise → Keep.
- **`IsRedundantFrame`:** among the counted points, the fraction with `other_observers_at_level >= 3` is ≥ 0.9. A Frame with no counted points is not redundant.

- [ ] **Step 1: Write failing tests:**
  - `JudgeRecentPointTest.Boundaries`: found/visible 1/4 → Keep and 1/5 → Remove; observation counts at 2 (mono) and 3 (stereo) → Remove, at 3 and 4 → Keep; graduation at 3 Frames.
  - `IsRedundantFrameTest.Boundaries`: 9/10 → true; 89/100 → false; no counted points → false.
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Map-quality rules: recent-point and Frame-redundancy verdicts"`.

### Task 6: Mapper — counters, recent-point culling, fusion, transaction 1

**Files:**
- Modify: `libs/slam/include/voyahslam/mapping/mapper.h`, `libs/slam/src/mapping/mapper.cpp`. If mapper.cpp passes ~900 lines, split it: create `libs/slam/src/mapping/mapper_quality.cpp` for fusion and culling.
- Test: `libs/slam/tests/mapping/mapper_test.cpp` (new suites `MapperFusionTest` and `MapperCullingTest`)

**Interfaces:**
- **Consumes:** Task 1 `MapTransaction`; Task 4 `FuseMatches`; Task 5 `JudgeRecentPoint` and `MapQualityOptions`.
- **Produces:**
  - `MapperOptions::map_quality` (`MapQualityOptions`) and `MapperOptions::fuse` (`FuseOptions`).
  - Per-point `visible` and `found` counters, updated from every `FrameTracked` of the current map, skipped events included.
  - The recent-point list.
  - Transaction 1 per created Frame: step 1 (recent culling) runs before creation, then creation, then fusion. Its parts are the new Frame, new points, added and removed observations, updated, replaced and removed points.
- **Fusion targets:** the first-order Frames from `CovisibilityIndex::Covisible(new, 30|10)`, plus the second-order Frames (each target's top 20), unique and excluding the new Frame. Forward and backward passes follow spec §3.
- **Merging:**
  - The winner has the larger weighted observation count; on a tie, the lower id wins.
  - The loser's observations move to the winner, except in Frames that already observe the winner.
  - Chains resolve within a pass: if C replaced B and B replaced A, every record points to the final winner.

- [ ] **Step 1: Write failing tests:**
  - `MapperFusionTest.MergesDuplicatePoints`: in a synthetic scene with planted duplicate pairs, after the next Frame each pair has become one point. The transaction lists the loser in `removed_points` and `replaced_points`, and the winner has the union of the observations.
  - `MapperFusionTest.FusionRaisesObservationCounts`: in a mono sequence of 6 Frames, at least 50 % of the points have ≥ 3 observations. Without fusion they would all have 2.
  - `MapperFusionTest.ChainedMergesPublishFinalWinner` (Review Focus 3).
  - `MapperCullingTest.CountersIncludeSkippedFrameTracked`: 3 events queued and only one evaluated; the counters still increased 3 times.
  - `MapperCullingTest.RemovesPoorRecentPoint`: a point with found/visible = 1/5 is removed in the next transaction 1.
  - `MapperFusionTest.ReplicaStaysConsistent`: every transaction applied to a `MapReplica` keeps `CheckConsistency()` true, and covisibility matches the Mapper's.
- [ ] **Step 2: Run** `--gtest_filter='Mapper*'` on `voyah_slam_tests`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`, build the apps and Release. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper: visible/found counters, recent-point culling, fusion (transaction 1)"`.

### Task 7: Mapper — local BA, Frame culling, transaction 2

**Files:**
- Modify: `libs/slam/src/mapping/mapper.cpp` (or `mapper_quality.cpp`), `libs/slam/include/voyahslam/mapping/mapper.h`
- Test: `libs/slam/tests/mapping/mapper_test.cpp` (suite `MapperQualityTest`)

**Interfaces:**
- **Consumes:** Task 2 `pending_priority_counter()`; Task 3 `BundleAdjust` with stereo and stop; Task 5 `IsRedundantFrame`.
- **Produces:** transaction 2 per created Frame, holding updated poses, updated points, removed observations (BA outliers and culled Frames), removed points and removed Frames.
- **Local BA problem** (spec §5):
  - Optimized Frames: the new Frame plus covisible Frames with ≥ 15 shared points, capped at 20.
  - Fixed Frames: the other observers. The map's first Frame is always fixed.
  - Skip BA when no Frame would be free (Review Focus 4).
- **Stop flag:** a member `std::atomic<bool> ba_stop_`. Before BA it is set if `inbox().pending_priority() > 0`. The per-iteration hook re-checks the counter.
- **Applying BA:**
  - Poses and points are always applied.
  - Outliers are removed only if `stage_reached >= 1`.
  - Points with no observations left are removed.
- **Frame culling** (spec §4.2): uses `IsRedundantFrame`, never removes the first or the new Frame, and reassigns references by the smallest-Frame-id rule.
- **Debug log:** one line per pass, with ms per step, BA iterations, interrupted yes/no, and counts.

- [ ] **Step 1: Write failing tests:**
  - `MapperQualityTest.PublishesTwoTransactionsPerFrame`
  - `MapperQualityTest.LocalBaReducesReprojectionError`: on a noisy synthetic sequence, the mean reprojection error of the local points after transaction 2 is lower than before.
  - `MapperQualityTest.UrgentEventStopsBa`: a test hook enqueues an urgent `FrameTracked` before BA. The result is `interrupted` with stage 0, and no outliers are removed.
  - `MapperQualityTest.CullsRedundantFrame`: a sequence with near-duplicate Frames; one is removed; the first Frame never is.
  - `MapperQualityTest.CullsFrameHoldingReferences` (Review Focus 2): a removed Frame that held point references; the references are reassigned, and replica and Mapper agree.
  - `MapperQualityTest.SkipsBaWithoutFreeFrames` (Review Focus 4)
- [ ] **Step 2: Run.** Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`, build the apps and Release, and run the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper: interruptible local BA and Frame culling (transaction 2)"`.

### Task 8: Trajectory output, integration, ThreadSanitizer, smoke, docs

**Files:**
- Modify: `examples/full_slam/{monocular_full_slam.cpp,binocular_full_slam.cpp,full_slam_common.h}`, `libs/slam/tests/integration/slam_test.cpp`, `docs/DESIGN.md`, `docs/TODO.md`, `docs/ROADMAP.md`
- Create: `tools/ate.py`

**Interfaces:**
- **Trajectory output:** `--trajectory_out <file>` writes one TUM line per tracked frame (`timestamp tx ty tz qx qy qz qw`, camera-to-world). On re-initialization it starts a new section, marked by a `# map <id>` comment line.
- **`tools/ate.py <trajectory> <groundtruth.csv> --align sim3|se3`:**
  - reads TUM-VI mocap (`mav0/mocap0/data.csv`);
  - associates timestamps within 20 ms;
  - Umeyama alignment;
  - prints RMSE and the number of matched poses, using the largest map section.

- [ ] **Step 1: Integration tests:**
  - `SlamTest.DuplicatesFusedInLongSequence`: after the run, observation counts are ≥ 3 for ≥ 50 % of the points, and no two alive points lie within 1 mm with descriptors within 10 bits.
  - Tighten the bounds of `SlamTest.{Mono,Stereo}MapGrowsWhileLeavingInitialView`: measure 3 runs; the new bound is the maximum × 1.5. Record the measurements in the commit message.
- [ ] **Step 2: ThreadSanitizer** on `voyah_slam_tests` and `voyah_integration_tests`, in a scratch TSan tree from the Debug toolchain. Expected: no warnings.
- [ ] **Step 3: Smoke runs** (Release, `/home/vahagn/Downloads/dataset-corridor1_1024_16`):

  | Run | Must | Report |
  |---|---|---|
  | Mono 400 | One map from the first initialization to the end; Frames created throughout | Frames created/culled, points created/culled/fused, BA ms per Frame and interrupts, Mapper ms per pass, ATE (Sim3) |
  | Stereo 1000 | No tracking loss, one map | same, ATE (SE3) |

  A miss is DONE_WITH_CONCERNS with a diagnosis from the debug logs. Nothing is tuned.
- [ ] **Step 4: Docs** (spec §8):
  - `DESIGN.md`: the pipeline, `MapTransaction` and its apply order, fusion, culling, local BA, the urgent interruption, the smoke table, and decisions-log entries dated 2026-10-07.
  - `TODO.md`: global BA in a separate thread (step 4).
  - `ROADMAP.md`: step 1 done, with its numbers.
- [ ] **Step 5: Run** `check_all`, build the apps and Release. Expected: green.
- [ ] **Step 6: Commit** `git commit -m "Map quality: trajectory output, ATE tool, integration, smoke, docs"`.
