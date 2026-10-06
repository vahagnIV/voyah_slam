# Keyframes and Map Growth Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The map grows while tracking. The Tracker reports every tracked measurement in `FrameTracked`; the Mapper decides when a measurement becomes a `Frame`, adds observations, creates new points (stereo close points, triangulation against neighbour Frames), and publishes one atomic `KeyFrameCreated`. The Tracker tracks against a covisibility-based local map.

**Architecture:**
- Events first: `FrameTracked` replaces `PositionUpdated`, and `KeyFrameCreated` becomes a complete transaction. `MapPointsAdded` and `NewKeyFrameCandidate` are deleted, with behaviour unchanged.
- Shared building blocks:
  - `map::CovisibilityIndex`, used by both the Mapper and the replica;
  - point statistics;
  - the pure `ShouldCreateFrame` rule;
  - triangulation matching.
- The Mapper's decision and creation steps are wired in one at a time.
- The Tracker's local map and reference Frame come next, then the viewer, then end-to-end checks.

**Tech Stack:** C++20, Eigen, g2o (PRIVATE), GoogleTest, Conan 2/CMake, Pangolin (viewer, PRIVATE).

**Spec:** `docs/superpowers/specs/2026-10-06-keyframes-design.md`

## Global Constraints

- **Branch:** `feat/keyframes`. Commit per task. Never merge or push.
- **Naming (Google style):** `PascalCase` types/functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- **Public headers:** no g2o/OpenCV/Pangolin types in any public header (`include/`).
- **Threading:** no locks on map data. Threads communicate only via events. The Mapper is the only map writer, and the Tracker never mutates map objects.
- **Ids:**
  - Frame ids come only from the Mapper's `next_frame_id_`, never reused within a run.
  - Map point ids are append-only within a map and restart per map.
  - Every point id the Mapper receives is checked with `IsValid` before use.
- **Terminology:** `Frame` is a keyframe. Event names keep "KeyFrame".
- **Residual space:** reprojection tests are in raw pixels through `geometry::Camera`, with σ = `sigma_px · level_scale^level` (defaults 1.0, 1.2) and χ² 5.991 (2 dof). Rays stay rays.
- **Events:** pooled, ref-counted. `Reset()` clears contents but keeps capacity. Producers set `map_id` on every publish.
- **Build/test:**
  - Full: `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 120`.
  - Focused: `cmake-build-debug/build/Debug/tests/voyahslam_tests --gtest_filter='<Suite>*'`.
  - Viewer tests: `cmake-build-debug/build/Debug/vizualization/tests/voyahviz_tests`.
  - The Release tree `build/build/Release` must build.
  - Master has 405 tests; all stay green, modulo the call-site migrations this plan names.
  - cmake binary: `/home/vahagn/Programs/clion/clion-2025.1.3/bin/cmake/linux/x64/bin/cmake`.
- **Header check:** every new or changed public header passes `clang++-18 -std=c++20 -stdlib=libc++ -fsyntax-only` with the include flags from `voyahslam_tests`' `flags.make`.
- **Docs:** `docs/DESIGN.md` and `docs/TODO.md` are changed only in Task 8.

## Review Focus

1. **Stationary camera (zero baseline) between the new Frame and a neighbour.**
   - Expected: the neighbour is skipped, no points are created, and there is no NaN.
   - Test: Task 3, `TriangulationTest.ZeroBaselineCreatesNothing`.
2. **`FrameTracked` naming dead, out-of-range or duplicate point ids** (the Tracker's replica lags, and two features could name one point).
   - Expected: the Mapper skips them; one observation per (Frame, point).
   - Test: Task 4, `MapperFrameCreationTest.SkipsInvalidAndDuplicatePoints`.
3. **A binocular measurement with no stereo matches at all.**
   - Expected: `need_close` is false, no close points are created, and nothing crashes.
   - Test: Task 5, `MapperFrameCreationTest.StereoWithoutMatchesCreatesNoClosePoints`.
4. **`FrameTracked` for a map the Mapper has already left** (after `TrackingLost`, while `Initializing`).
   - Expected: ignored, and no Frame is created.
   - Test: Task 4, `MapperInboxTest.IgnoresFrameTrackedOfOldMap`.
5. **Tracker matches whose observing Frames are not (yet) in its replica.**
   - Expected: the local map is built from the Frames it knows. With no known Frame, there is no reference Frame and the local map falls back to all points.
   - Test: Task 6, `LocalMapTest.UnknownFramesFallBackToAllPoints`.

---

### Task 1: Event migration — `FrameTracked`, combined `KeyFrameCreated`, `CovisibilityIndex`

**Files:**
- Create: `include/voyahslam/events/frame_tracked.h`, `include/voyahslam/map/covisibility_index.h`, `tests/map/covisibility_index_test.cpp`
- Modify: `include/voyahslam/events/{event_types.h, events.h, key_frame_created.h}`, `include/voyahslam/tracking/{map_replica.h, tracker.h}`, `src/tracking/tracker.cpp`, `include/voyahslam/mapping/mapper.h`, `src/mapping/mapper.cpp`, `vizualization/include/voyahviz/scene_state.h`, `vizualization/src/scene_state.cpp`, plus every test that names the removed events (`tests/events/*`, `tests/tracking/*`, `tests/mapping/mapper_test.cpp`, `tests/features/projection_matcher_test.cpp`, `tests/slam/slam_test.cpp`, `vizualization/tests/{scene_state_test,map_viewer_test}.cpp`), `tests/CMakeLists.txt`
- Delete: `include/voyahslam/events/{position_updated.h, map_points_added.h, new_key_frame_candidate.h}`

**Interfaces:**
- **Produces:**
  ```cpp
  // events/event_types.h: EventType loses PositionUpdated, MapPointsAdded, NewKeyFrameCandidate,
  // gains FrameTracked; TrackingLost stays the last enumerator.
  // events/frame_tracked.h
  enum class TrackingStage : uint8_t { MotionModel, ZeroMotion, LocalMap };
  template <typename TFeatureExtractor> struct FrameTracked : Event, memory::Recyclable<...> {
      double timestamp = 0.0;
      geometry::RigidTransformation pose;                       // T_cw
      features::TMeasurement<TFeatureExtractor> measurement;
      std::vector<uint32_t> observed_features;                  // inliers only, paired with
      std::vector<map::MapPointId> observed_points;
      std::vector<map::MapPointId> visible_points;
      std::optional<map::FrameId> reference_frame_id;
      std::vector<map::FrameId> local_frames;
      TrackingStage stage = TrackingStage::MotionModel;
      uint32_t num_matches = 0, num_inliers = 0;
      void Reset();
  };
  // events/key_frame_created.h — added members (existing Frame members unchanged)
  struct AddedObservation { map::FrameId frame_id; uint32_t feature; map::MapPointId point;
                            precision_t depth = -1; int16_t right_index = -1; };
  map::MapPointId new_points_start_id{};
  memory::ColumnBuffer<precision_t, 3> new_positions, new_normals;
  memory::ColumnBuffer<TDescriptorScalarType, DescriptorSize> new_descriptors;
  void ResizeNewPoints(uint32_t count);  uint32_t num_new_points() const;
  std::vector<AddedObservation> added_observations;
  std::vector<map::MapPointId> updated_ids;
  memory::ColumnBuffer<precision_t, 3> updated_positions, updated_normals;
  memory::ColumnBuffer<TDescriptorScalarType, DescriptorSize> updated_descriptors;
  void ResizeUpdatedPoints(uint32_t count);
  // map/covisibility_index.h
  struct PointObservation { FrameId frame; uint32_t feature; };
  class CovisibilityIndex {
  public:
      void Add(FrameId frame, uint32_t feature, MapPointId point);   // ignores an existing (frame, point)
      void Clear();
      const std::vector<PointObservation>& Observations(MapPointId point) const;  // empty if unknown
      size_t NumObservations(MapPointId point) const;
      const std::vector<MapPointId>& Points(FrameId frame) const;                 // empty if unknown
      bool Contains(FrameId frame, MapPointId point) const;
      // Frames sharing points with `frame`, by shared count desc, ties by FrameId asc; excludes `frame`.
      std::vector<std::pair<FrameId, uint32_t>> Covisible(FrameId frame, size_t max_count) const;
  };
  // tracking/map_replica.h — added
  const map::CovisibilityIndex& covisibility() const;
  ```
- **Behaviour:** behaviour-preserving.
  - The Tracker publishes `FrameTracked` where it published `PositionUpdated`.
    - `observed_*` are the final inlier matches; `stage` and the counts come from the final step.
    - `reference_frame_id` = the current map's initial Frame (`map_frame_id_`).
    - `visible_points` and `local_frames` stay empty until Task 6.
  - The Mapper's initialization publishes the combined event:
    - mono: event 1 carries Frame 1, all new points and Frame 1's observations; event 2 carries Frame 2;
    - stereo: one event;
    - then `MapCreated`.
  - The replica applies the parts in the order new points → updated points → `added_observations` → Frame, and feeds `CovisibilityIndex`.
  - `SceneState` takes pose and trajectory from `FrameTracked`, and points and Frames from `KeyFrameCreated`.

- [ ] **Step 1: Write failing tests:**
  - `CovisibilityIndexTest.AddIgnoresDuplicates`
  - `CovisibilityIndexTest.CovisibleOrdersBySharedCountThenId`: Frames A, B and C. B shares 3 points with A and C shares 3; D shares 1. `Covisible(A, 10)` = [(B,3),(C,3),(D,1)] when B < C; `Covisible(A, 2)` has size 2.
  - `CovisibilityIndexTest.UnknownIdsAreEmpty`
  - `MapReplicaTest.AppliesCombinedKeyFrameCreatedAtomically`:
    - One event with 3 new points, 1 updated point, 1 added observation on an existing Frame, and the Frame.
    - After `Apply`, every observation's point is alive and every Frame exists.
    - `covisibility().Observations(p)` matches the per-Frame lists in both directions.
  - `MapReplicaTest.UpdatedPointsChangeNormalAndDescriptor`
  - `MapperTest`: the existing initialization tests, migrated to assert mono = 2 `KeyFrameCreated` (the first with `num_new_points()` equal to the map size) and stereo = 1, then `MapCreated`.
  - `TrackerTest`: the existing tracking tests, migrated to `FrameTracked`; also assert `observed_points.size() == num_inliers` and `stage == TrackingStage::LocalMap` on a successful frame.
  - `SceneStateTest`: the `PositionUpdated` tests are renamed to `FrameTracked`; the `MapPointsAdded` tests become `KeyFrameCreated` with new points.
- [ ] **Step 2: Run** the full build. Expected: compile failures where the deleted events and the new ones are referenced.
- [ ] **Step 3: Implement** the Interfaces above and migrate every producer and consumer.
- [ ] **Step 4: Run** the full suite, viewer tests, Release build and header check. Expected: green, with the test count ≥ 405 + new tests.
- [ ] **Step 5: Commit** `git commit -m "Events: FrameTracked replaces PositionUpdated; KeyFrameCreated is one atomic transaction"`.

### Task 2: `ShouldCreateFrame` and point statistics

**Files:**
- Create: `include/voyahslam/mapping/keyframe_decision.h`, `src/mapping/keyframe_decision.cpp`, `include/voyahslam/map/point_statistics.h`, `tests/mapping/keyframe_decision_test.cpp`, `tests/map/point_statistics_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  // mapping/keyframe_decision.h
  struct KeyFrameOptions {
      double max_interval = 1.0;  double ratio_mono = 0.9;  double ratio_stereo = 0.75;
      double ratio_stereo_early = 0.4;  double weak_ratio = 0.25;  double close_depth_baselines = 40.0;
      size_t close_tracked_max = 100;  size_t close_untracked_min = 70;  size_t min_inliers = 15;
  };
  struct KeyFrameSignals {
      bool stereo = false;  size_t num_frames = 0;  size_t ref_points = 0;  double since_last = 0.0;
      size_t backlog = 0;  size_t inliers = 0;  size_t close_tracked = 0;  size_t close_untracked = 0;
  };
  [[nodiscard]] bool ShouldCreateFrame(const KeyFrameSignals& signals, const KeyFrameOptions& options);
  // map/point_statistics.h
  geometry::TPoint3DF MeanViewingDirection(const geometry::TPoint3DF& point,
                                           const std::vector<geometry::TPoint3DF>& camera_centres);  // unit; mean of unit (X − C)
  template <typename TFeatureExtractor>
  typename TFeatureExtractor::TDescriptor DistinctiveDescriptor(
      const std::vector<typename TFeatureExtractor::TDescriptor>& descriptors);  // least median distance to the others; ties → lowest index
  ```
- The rule is spec §3.3 verbatim. `need_close` = `stereo && close_tracked < close_tracked_max && close_untracked > close_untracked_min`.

- [ ] **Step 1: Write failing tests** (`ShouldCreateFrameTest.*`). Baseline signals: mono, `num_frames` 3, `ref_points` 100, `since_last` 0.1, `backlog` 1, `inliers` 50.
  - `NoTriggerNoFrame`: baseline → false (c1 is false).
  - `IdleAndWeakCreates`: backlog 0, inliers 89 → true; inliers 90 → false (ratio 0.9).
  - `MaxIntervalCreates`: since_last 1.0, backlog 1, inliers 89 → true.
  - `MinInliersFloor`: backlog 0, inliers 15 → false; 16 → true.
  - `StereoWeakTriggersWhileBusy`: stereo, backlog 1, inliers 24, ref_points 100 → true (c1c, and c2 at 0.75).
  - `StereoNeedClose`: stereo, inliers 90, ref_points 100, close_tracked 99, close_untracked 71, backlog 1 → true; close_untracked 70 → false.
  - `StereoEarlyRatio`: stereo, num_frames 1, backlog 0, ref_points 100: inliers 39 → true, 40 → false.
  - `PointStatisticsTest.MeanViewingDirectionIsUnitMean`: two centres symmetric about the z-axis → (0,0,1).
  - `PointStatisticsTest.DistinctiveDescriptorPicksMedoid`: three descriptors, where one differs by 1 bit from the other two and they differ by 10 from each other → the 1-bit one. All equal → index 0.
- [ ] **Step 2: Run** `--gtest_filter='ShouldCreateFrame*:PointStatistics*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the filter, full suite and header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "ShouldCreateFrame rule and point statistics"`.

### Task 3: Triangulation matching and acceptance

**Files:**
- Create: `include/voyahslam/mapping/triangulation.h`, `tests/mapping/triangulation_test.cpp`

**Interfaces:**
- **Consumes:** `geometry::Camera` (`Project`, `ProjectJacobian`, `Unproject`), `geometry::Triangulate`, `features::RotationHistogram`, `TFeatureExtractor::Distance`, `MonocularMeasurement::{normalized_points, distorted_points, levels, descriptors, camera}`.
- **Produces:**
  ```cpp
  struct TriangulationOptions {
      int th_low = 50;  double nn_ratio = 0.6;  double min_parallax_cos = 0.9998;  double chi2 = 5.991;
      double sigma_px = 1.0;  double level_scale = 1.2;  double scale_consistency = 1.5;
      size_t histogram_bins = 12;  double epipolar_sigmas = 3.0;
  };
  template <typename TFeatureExtractor> struct TriangulationView {
      const features::TMonocularMeasurement<TFeatureExtractor>* measurement;  // left image
      geometry::RigidTransformation T_cw;
      std::vector<bool> has_point;          // per feature: already observes a map point
      std::vector<precision_t> stereo_depth; // per feature, < 0 if none; empty for mono
      double baseline = 0.0;                 // rig baseline, 0 for mono
  };
  struct TriangulatedPoint { geometry::TPoint3DF position; uint32_t feature_a; uint32_t feature_b; };
  template <typename FE> std::vector<std::pair<uint32_t, uint32_t>> SearchForTriangulation(
      const TriangulationView<FE>& a, const TriangulationView<FE>& b, const TriangulationOptions& options);
  template <typename FE> std::vector<TriangulatedPoint> TriangulateMatches(
      const TriangulationView<FE>& a, const TriangulationView<FE>& b,
      const std::vector<std::pair<uint32_t, uint32_t>>& matches, const TriangulationOptions& options);
  ```
- **Matching** (spec §4.4): brute force over pairs where neither feature `has_point`.
  - Distance ≤ `th_low`, and best < `nn_ratio` · second.
  - **Epipolar test on rays:** with `n = normalize(t_ba × (R_ba · ray_a))`, accept if `asin(|n · ray_b_unit|) ≤ epipolar_sigmas · σ_level(b) / ‖ProjectJacobian_b‖₂`. The pixel-to-angle factor is the spectral norm of the 2×3 Jacobian at `ray_b`, at depth 1.
  - Rotation histogram, keeping the top 3 bins.
  - One-to-one.
- **Acceptance:** DLT on rays.
  - Parallax: cos < `min_parallax_cos`.
  - **Stereo fallback:** if a feature has `stereo_depth ≥ 0` and its stereo parallax cos, `cos(2·atan2(baseline/2, depth))`, is smaller than the two-view cos, place the point from that stereo depth instead (the larger stereo parallax wins).
  - z > 0 in both views.
  - Raw-px χ² < `chi2` in both, with σ = `sigma_px·level_scale^level`.
  - Scale consistency: `dist_a/dist_b` lies within `ratio_level · scale_consistency·level_scale` and `ratio_level / (scale_consistency·level_scale)`, where `ratio_level = level_scale^(level_a − level_b)`.

- [ ] **Step 1: Write failing tests** (`TriangulationTest.*`, synthetic scenes as in `tests/support`):
  - `PinholeRecoversPoints`: two views with a 0.3 m baseline and 200 points. All `has_point` are false and descriptors are exact. ≥ 95 % are matched and triangulated, every position within 1e-6 of ground truth, and there are no wrong pairs.
  - `FisheyePeripheralPointsRecovered`: the TUM-VI KB camera with rays from 60° to 80° and 0.5 px noise. ≥ 80 % are recovered, within 2 % of depth.
  - `SkipsFeaturesWithPoints`: a feature with `has_point` is never in a match.
  - `RejectsLowParallax`: a 1 mm baseline at 5 m depth → no points.
  - `RejectsReprojectionOutlier`: one view's keypoint is shifted 5 px perpendicular to the epipolar curve; the match passes the descriptor test but no point is created for it.
  - `ZeroBaselineCreatesNothing` (Review Focus 1): identical poses → empty result, no NaN.
  - `StereoFallbackUsesDepth`: a 2 mm baseline between views, `stereo_depth` set and baseline 0.11 → the point equals the stereo back-projection (1e-9).
- [ ] **Step 2: Run** `--gtest_filter='Triangulation*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the filter, full suite and header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Triangulation matching and acceptance on rays, raw-pixel gate"`.

### Task 4: Mapper decides and creates Frames from tracked observations

**Files:**
- Modify: `include/voyahslam/mapping/mapper.h`, `src/mapping/mapper.cpp`
- Test: `tests/mapping/mapper_test.cpp` (new suites `MapperInboxTest`, `MapperFrameCreationTest`)

**Interfaces:**
- **Consumes:**
  - Task 1: `FrameTracked`, combined `KeyFrameCreated`, `CovisibilityIndex`.
  - Task 2: `ShouldCreateFrame`, `KeyFrameOptions`, `MeanViewingDirection`, `DistinctiveDescriptor`.
- **Produces:**
  ```cpp
  struct MapperOptions {
      MapInitializerOptions initializer;
      KeyFrameOptions keyframe;
      TriangulationOptions triangulation;      // carries sigma_px, level_scale (spec §4 "MapperOptions gains")
      size_t neighbours_mono = 30;  size_t neighbours_stereo = 10;
      size_t min_close_points = 100;  double min_baseline_depth_ratio = 0.01;
  };
  // Mapper: readable from any thread
  [[nodiscard]] size_t num_frame_evaluations() const;  // FrameTracked events evaluated (not skipped)
  [[nodiscard]] size_t backlog_skipped() const;        // total FrameTracked skipped by drain-to-latest
  ```
- **Behaviour:**
  - **Inbox:** `ProcessPending` drains a batch. A `FrameTracked` is evaluated only if it is in `Mapping`, it is for the current map, and no later `FrameTracked` of the same map is in the batch. `backlog` = skipped since the last evaluation.
  - **Signals** (spec §3.2):
    - `ref_points` uses `CovisibilityIndex::NumObservations` with `min_obs` 2 (≤ 2 Frames) or 3, on `reference_frame_id`. If that Frame is unknown or absent, the newest Frame is used.
    - `since_last` is measured from the last Frame's timestamp.
    - The close counts use `close_depth = close_depth_baselines × rig baseline`.
  - **When `ShouldCreateFrame` is true:** spec §4 steps 1, 2 and 5.
    - Duplicate points are kept once, invalid ids are skipped, and the Mapper's index is updated.
    - Normals and descriptors of the touched points are recomputed into `updated_*`.
    - A combined event is published with no new points.
  - Every evaluation is logged at debug level with its signals and the result.

- [ ] **Step 1: Write failing tests:**
  - `MapperInboxTest.DrainsToLatest`: after mono init, three `FrameTracked` events are queued before one `ProcessPending`. `num_frame_evaluations()` grows by 1 and `backlog_skipped()` by 2.
  - `MapperInboxTest.TrackingLostAfterFrameTrackedCreatesNoFrame`
  - `MapperInboxTest.IgnoresFrameTrackedOfOldMap` (Review Focus 4): the event carries map_id n after the map was reset → no `KeyFrameCreated`.
  - `MapperFrameCreationTest.CreatesFrameWhenWeak`: mono, an idle Mapper and inliers at 50 % of the map → one `KeyFrameCreated` with the new Frame id = previous + 1. Its observations equal the event's pairs, and `updated_ids` equals those points.
  - `MapperFrameCreationTest.NoFrameWhenStrong`: inliers = ref_points → none.
  - `MapperFrameCreationTest.SkipsInvalidAndDuplicatePoints` (Review Focus 2): pairs that include an id ≥ `num_ids()` and the same point twice → skipped / kept once.
  - `MapperFrameCreationTest.RefPointsUsesMinObs`: after a third Frame, points observed by only 2 Frames no longer count.
- [ ] **Step 2: Run** `--gtest_filter='Mapper*'`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `Mapper*:Slam*:Tracker*`, the full suite, Release build and header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper decides on FrameTracked and creates Frames with tracked observations"`.

### Task 5: Mapper grows the map — stereo close points and triangulation

**Files:**
- Modify: `src/mapping/mapper.cpp`, `include/voyahslam/mapping/mapper.h`
- Test: `tests/mapping/mapper_test.cpp`

**Interfaces:**
- **Consumes:**
  - Task 3: `SearchForTriangulation`, `TriangulateMatches`, `TriangulationView`.
  - Task 4: `CreateFrame`.
- **Behaviour** (spec §4 steps 3–4):
  - **Close points.** Binocular only. Back-project the same way stereo initialization places its points. Take every point with depth < `close_depth`, and at least the `min_close_points` closest.
  - **Neighbours.** `CovisibilityIndex::Covisible(new, neighbours_mono|stereo)`, skipping any neighbour where:
    - mono: `baseline / median depth of the neighbour's points < min_baseline_depth_ratio`;
    - stereo: `baseline < rig baseline`.
  - A feature already used by an earlier neighbour's new point is marked `has_point` for the next neighbour.
  - New points:
    - **References:** (new Frame, feature).
    - **Normals:** computed from their observations.
    - **Descriptors:** the new Frame's feature descriptor.
  - Each new point's neighbour observation goes into `added_observations`, with depth and right index for a binocular neighbour.

- [ ] **Step 1: Write failing tests:**
  - `MapperFrameCreationTest.StereoClosePointsFromDepth`: a synthetic stereo frame, noise-free. The new points' positions match ground truth within 1e-9, and the count = max(#depth < close_depth, min(100, #matched untracked)).
  - `MapperFrameCreationTest.StereoWithoutMatchesCreatesNoClosePoints` (Review Focus 3).
  - `MapperFrameCreationTest.MonoTriangulatesAgainstNeighbours`: a synthetic mono sequence with 4 % parallax. After the third Frame, new points exist; each has one observation in the new Frame and one in `added_observations`; positions are within 1e-6 of ground truth.
  - `MapperFrameCreationTest.ReplicaStaysConsistent`: the published events are applied to a `MapReplica`. Its covisibility equals the Mapper's for every Frame.
- [ ] **Step 2: Run** `--gtest_filter='MapperFrameCreation*'`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the full suite, Release build and header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper grows the map: stereo close points and triangulation against neighbours"`.

### Task 6: Tracker — covisibility local map, reference Frame, visible points

**Files:**
- Create: `include/voyahslam/tracking/local_map.h`, `tests/tracking/local_map_test.cpp`
- Modify: `include/voyahslam/tracking/tracker.h`, `src/tracking/tracker.cpp`, `include/voyahslam/features/projection_matcher.h`
- Test: `tests/tracking/tracker_test.cpp`, `tests/features/projection_matcher_test.cpp`

**Interfaces:**
- **Consumes:** `MapReplica::covisibility()` (Task 1).
- **Produces:**
  ```cpp
  // tracking/local_map.h
  struct LocalMap { std::vector<map::FrameId> frames; std::vector<bool> point_mask; bool all_points = false; };
  template <typename FE> LocalMap BuildLocalMap(const MapReplica<FE>& replica, const std::vector<int64_t>& matches,
                                               size_t local_neighbours, size_t max_local_frames);
  template <typename FE> std::optional<map::FrameId> ReferenceFrame(const MapReplica<FE>& replica,
                                                                    const std::vector<int64_t>& inlier_matches);
  // TrackingOptions gains: size_t local_neighbours = 10; size_t max_local_frames = 80;
  // SearchFromLocalMap gains a trailing parameter: std::vector<map::MapPointId>* out_visible = nullptr
  //   (ids passing the projection gate, appended; existing callers unchanged)
  ```
- **Behaviour:**
  - **K1:** Frames observing any match, ordered by match count descending (ties by id).
  - **K2:** each K1 Frame's top `local_neighbours` covisible Frames.
  - **Cap:** `max_local_frames`.
  - **Mask:** the alive points of those Frames.
  - **No known Frame:** `all_points = true` (Review Focus 5).
  - `SearchFromLocalMap` runs through a point-source adapter whose `IsAlive` = replica alive && mask.
  - `ReferenceFrame` returns the K1 Frame with the most inliers.
  - `FrameTracked` fills `reference_frame_id`, `local_frames` and `visible_points`. `map_frame_id_` is removed.
  - **`last_frame_` merge:** on `KeyFrameCreated` whose measurement timestamp equals the timestamp of `last_frame_.measurement`, set `last_frame_.matches[feature] = point` for its observations.

- [ ] **Step 1: Write failing tests:**
  - `LocalMapTest.K1AndK2`: a hand-built replica with Frames F1–F5. The matches touch only F1. F1 is covisible with F2 (5 shared) and F3 (2); F4 is covisible only with F3; F5 shares nothing. `local_neighbours` 1 → frames {F1, F2}, and the mask equals the points of F1 ∪ F2.
  - `LocalMapTest.CapLimitsFrames`: 100 Frames each sharing a matched point, cap 80 → 80 frames.
  - `LocalMapTest.UnknownFramesFallBackToAllPoints` (Review Focus 5).
  - `LocalMapTest.ReferenceFrameHasMostInliers`
  - `ProjectionMatcherTest.ReportsVisiblePoints`: `out_visible` ⊇ matched ids, and points behind the camera are excluded.
  - `TrackerTest.FrameTrackedCarriesLocalMap`: after mono init, `local_frames` = both Frames, `reference_frame_id` is set, and `visible_points` ⊇ `observed_points`.
  - `TrackerTest.MergesOwnFrameObservationsIntoLastFrame`
- [ ] **Step 2: Run** `--gtest_filter='LocalMap*:ProjectionMatcher*:Tracker*'`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the full suite, Release build and header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Tracker: covisibility local map, reference Frame, visible points"`.

### Task 7: Viewer draws tracked and visible points

**Files:**
- Modify: `vizualization/include/voyahviz/scene_state.h`, `vizualization/src/scene_state.cpp`, `vizualization/src/pangolin_renderer.{h,cpp}`
- Test: `vizualization/tests/scene_state_test.cpp`

**Interfaces:**
- **Produces:** `SceneState` accessors.
  ```cpp
  const std::vector<voyahslam::map::MapPointId>& tracked_points() const;   // last FrameTracked's observed_points
  const std::vector<voyahslam::map::MapPointId>& visible_points() const;   // last FrameTracked's visible_points
  const std::vector<voyahslam::map::FrameId>& local_frames() const;
  ```
  All three are cleared on a map switch and on `TrackingLost`.
- **Rendering colours:**
  - tracked: green (0, 0.9, 0);
  - visible and unmatched: blue (0.2, 0.4, 1);
  - other alive points: grey (0.5);
  - local-map frustums: orange (1, 0.6, 0);
  - other Frames: the existing colour.

- [ ] **Step 1: Write failing tests:**
  - `SceneStateTest.FrameTrackedSetsTrackedAndVisible`
  - `SceneStateTest.TrackingLostClearsTrackedSets`
  - `SceneStateTest.KeyFrameCreatedAppliesUpdatedPoints`
- [ ] **Step 2: Run** the viewer tests. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the viewer tests, the full suite and the Release build. Expected: green. Also run the mono app once with the viewer to confirm visually, and note it in the report.
- [ ] **Step 5: Commit** `git commit -m "Viewer: tracked, visible points and local-map Frames"`.

### Task 8: End-to-end, ThreadSanitizer, smoke runs, docs

**Files:**
- Modify: `tests/support/synthetic_sequence.h`, `tests/slam/slam_test.cpp`, `docs/DESIGN.md`, `docs/TODO.md`

**Interfaces:**
- `SyntheticSequenceOptions` gains two options; existing tests are unchanged by default:
  - `double turn_deg = 0.0`: total yaw over the sequence, linear;
  - `bool surround_scene = false`: points on the walls of a 10 m room around the path instead of a frontal slab.

- [ ] **Step 1: Write failing tests:**
  - `SlamTest.MonoMapGrowsWhileLeavingInitialView`: `turn_deg` 90, `surround_scene`, 120 frames. Expected:
    - `TrackingLost` is never published;
    - Mapper Frames > 2 and points > the initial count;
    - the trajectory error (`trajectory_error.h`, Sim3) ≤ 0.05 × path length.
  - `SlamTest.StereoMapGrowsWhileLeavingInitialView`: the same, with SE3 alignment and ≤ 0.02 m.
- [ ] **Step 2: Run** `--gtest_filter='Slam*'`. Expected: the new tests fail on a clean tree only if Tasks 1–6 have a gap; otherwise they pass. If they pass at once, record that in the report: the test then guards against regressions.
- [ ] **Step 3: ThreadSanitizer.** Build a scratch TSan tree from the Debug toolchain (`-fsanitize=thread -O1 -g`). Run `setarch $(uname -m) -R <tsan>/tests/voyahslam_tests --gtest_filter='Tracker*:Mapper*:Slam*'`. Expected: no warnings.
- [ ] **Step 4: Smoke runs** (Release, corridor1 1024, debug logs, headless):

  | Run | Must | Report |
  |---|---|---|
  | Mono, 400 frames | The first map survives ≥ 150 consecutive tracked frames after its initialization; the median map lifetime is ≥ 50 frames | Frames, points, re-initializations, Frame-creation decisions (count and reasons), ATE vs mocap (Sim3) when available |
  | Stereo, 1000 frames | No `TrackingLost`; one map | Frames created, points, median inliers, ATE |

  A miss → DONE_WITH_CONCERNS with a diagnosis from the decision logs. Nothing is tuned.
- [ ] **Step 5: Docs** per spec §8. Add a DESIGN decisions-log entry dated 2026-10-06 for each listed decision, update the Tracker, Mapper and events-table sections, and add the smoke table. Add the TODO.md items from spec §8.
- [ ] **Step 6: Run** the full suite, viewer tests, Release build and the clang check of all sources. Expected: green.
- [ ] **Step 7: Commit** `git commit -m "Keyframes: end-to-end tests, smoke runs, docs"`.
