# Loop Closing (Detection and Correction) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When the camera returns to a mapped place, the LoopCloser detects it (ORB-SLAM3 `NewDetectCommonRegions`), and the Mapper corrects the map (ORB-SLAM3 `CorrectLoop` + `OptimizeEssentialGraph`). The correction goes out as one transaction, and the Tracker follows it.

**Architecture:**
- Map graph: a spanning tree, loop edges and a correction epoch, all carried in `MapTransaction`.
- algs gains Sim3 RANSAC, Sim3 optimization, Sim3 projection matching and the essential graph.
- The LoopCloser gets its own replica and publishes `LoopDetected`.
- The Mapper corrects; the Tracker shifts `last_frame_`.

**Tech Stack:** C++20, Eigen, g2o (PRIVATE in algs, Sim3 types from g2o `types_sim3`), GoogleTest, CMake/Conan.

**Spec:** `docs/superpowers/specs/2026-10-09-loop-closing-design.md`

## Global Constraints

- **Branch:** `feat/loop-closing`. Commit per task. Never merge or push.
- **Naming:** Google style. `PascalCase` types and functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- **Layers:** common ← events ← algs ← slam. No g2o or OpenCV in public headers.
- **Concurrency:**
  - No locks on map data.
  - The Mapper is the only writer.
  - The LoopCloser only reads its own replica and publishes `LoopDetected`.
  - The vocabulary is immutable and shared read-only.
- **No vocabulary means no loop closing.** The LoopCloser does nothing, and every other behaviour is unchanged.
- **Fidelity:** follow ORB-SLAM3's `LoopClosing` and `Optimizer` logic. Check each threshold against https://github.com/UZ-SLAMLab/ORB_SLAM3 (`LoopClosing.cc`, `KeyFrameDatabase.cc`, `Sim3Solver.cc`, `Optimizer.cc`), and record any difference in the report.
- **Spec values:**
  - Detection:
    - `min_map_frames` 12; `num_candidates` 3; `window` 5;
    - `min_bow_matches` 20, `min_sim3_inliers` 20, `min_optimized` 15;
    - `min_projection` 100, `confirmation_projection` 50;
    - `required_coincidences` 3, `max_unconfirmed_queries` 3;
    - RANSAC 300 iterations, confidence 0.99, χ² 9.21.
  - Correction: essential graph `min_essential_weight` 100, 20 iterations; fuse radius 4.
- **Residuals:** raw pixels, with σ = sigma_px·1.2^level.
- **Build and test:**
  - Debug tree: `cmake-build-debug/build/Debug`.
  - Fast loop: `--target check`. End of each task: `--target check_all`, the apps, and Release (`build/build/Release`).
  - cmake binary: `/home/vahagn/Programs/clion/clion-2025.1.3/bin/cmake/linux/x64/bin/cmake`.
  - Current total: 583 tests.
- **Header check:** clang-18 with libc++ `-fsyntax-only` on changed public headers.
- **Docs:** DESIGN, TODO and ROADMAP change only in Task 8.
- **Smoke inputs:**
  - dataset `/home/vahagn/Downloads/dataset-corridor1_1024_16`;
  - vocabulary `/home/vahagn/data/vocabulary/ORBvoc.bin`;
  - `tools/ate.py`.

## Review Focus

1. **A Frame in the loop window, or the query or match Frame, is culled between detection and correction.**
   - Expected: the correction is rejected, or the culled Frame is skipped, with no dangling ids in the transaction.
   - Test: Task 6, `MapperLoopTest.RejectsLoopWithCulledFrames`.
2. **A duplicate `LoopDetected` for an already-closed loop,** for example the next query confirming the same place.
   - Expected: no second correction; the existing loop edge is detected.
   - Test: Task 6, `MapperLoopTest.IgnoresDuplicateLoop`.
3. **A final `TrackingLost` (map cleared) while a `LoopDetected` is in flight.**
   - Expected: ignored by `map_id`. The LoopCloser's replica clears on `new_map`.
   - Test: Task 5, `LoopCloserTest.ClearsOnNewMap`; Task 6, `MapperLoopTest.IgnoresLoopOfOldMap`.
4. **A degenerate Sim3:** scale ≤ 0 or non-finite, or a near-identity loop between covisible Frames.
   - Expected: rejected before correction.
   - Test: Task 2, `Sim3Test.DegenerateFailsCleanly`; Task 5, `LoopCloserTest.CovisibleRevisitIsNotALoop`.
5. **The Tracker is RecentlyLost when a correction arrives.**
   - Expected: no `last_frame_` shift; relocalization uses the corrected replica.
   - Test: Task 7, `TrackerLoopTest.CorrectionWhileRecentlyLost`.

---

### Task 1: Map graph — spanning tree, loop edges, epoch

**Files:**
- Modify:
  - events: `libs/events/include/voyahslam/events/{map_transaction.h,frame_tracked.h}`
  - Mapper: `libs/slam/{include/voyahslam,src}/mapping/mapper*`
  - replica: `libs/slam/include/voyahslam/tracking/map_replica.h`
- Test: `libs/slam/tests/mapping/mapper_test.cpp`, `libs/slam/tests/tracking/map_replica_test.cpp`

**Interfaces:**
- **Produces:**
  - `NewFrame::parent` (`std::optional<map::FrameId>`);
  - `MapTransaction::parent_changes` (`std::vector<std::pair<FrameId,FrameId>>`), `added_loop_edges` (`std::vector<std::pair<FrameId,FrameId>>`), `map_epoch` (`uint32_t`);
  - `FrameTracked::map_epoch`;
  - replica accessors: `parent(FrameId)`, `children(FrameId)`, `loop_edges(FrameId)`, `map_epoch()`;
  - Mapper:
    - the parent at Frame creation is the Frame sharing the most points;
    - re-parenting on cull follows ORB-SLAM3 `SetBadFlag`;
    - Frames with loop edges are not culled;
    - `map_epoch` starts at 0 per map.

- [ ] **Step 1: Failing tests:**
  - `MapperGraphTest.ParentIsMostCovisibleAtCreation`
  - `MapperGraphTest.CulledFrameChildrenAreReparented`: no orphan remains, and every child's parent is alive.
  - `MapperGraphTest.LoopEdgeFramesAreNotCulled`
  - `MapReplicaTest.StoresParentsAndLoopEdges`: covers new Frames, `parent_changes` and loop edges, including after removals.
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.** Re-parenting: repeatedly attach the child that shares the most points with any candidate parent, starting from the culled Frame's parent, to that candidate. Remaining children go to the culled Frame's parent.
- [ ] **Step 4: Run** `check_all`, the apps, Release and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Map graph: spanning tree, loop edges, map epoch in transactions"`.

### Task 2: `geometry::SolveSim3Ransac`

**Files:**
- Create: `libs/algs/include/voyahslam/geometry/sim3.h`, `libs/algs/src/geometry/sim3.cpp`, `libs/algs/tests/geometry/sim3_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  struct Sim3 { double scale = 1.0; TMatrix33F R = I; TPoint3DF t = 0; /* x' = s R x + t */ ... };
  Sim3 Inverse(const Sim3&); Sim3 Compose(const Sim3& a, const Sim3& b);
  std::optional<Sim3> AlignHorn(const std::vector<TPoint3DF>& from, const std::vector<TPoint3DF>& to, bool fix_scale);
  struct Sim3RansacOptions { int max_iterations = 300; double confidence = 0.99; size_t min_inliers = 20; double chi2 = 9.21; bool fix_scale = false; uint64_t seed = 0; };
  struct Sim3RansacResult { bool success = false; Sim3 S_12; std::vector<bool> inlier; size_t num_inliers = 0; };
  // Pair i: X1 in frame-1 camera coords observed at pixel1 (camera1), X2 likewise; inlier if both reprojections (S_12 maps 2→1 and its inverse) have chi2 < chi2.
  Sim3RansacResult SolveSim3Ransac(const Camera& camera1, const Camera& camera2,
      const std::vector<TPoint3DF>& X1, const std::vector<TPoint2DF>& pixels1, const std::vector<double>& sigmas1,
      const std::vector<TPoint3DF>& X2, const std::vector<TPoint2DF>& pixels2, const std::vector<double>& sigmas2,
      const Sim3RansacOptions& options);
  ```
  It follows ORB-SLAM3 `Sim3Solver`: Horn's closed form, 3-point minimal samples, the reprojection inlier test in both images, and adaptive iterations.

- [ ] **Step 1: Failing tests:**
  - `Sim3Test.HornRecoversNoiseFree`: with and without `fix_scale`, to 1e-9.
  - `Sim3Test.RansacHalfOutliers`: 200 seeds. Success, and an inlier mask at least 90 % correct, in ≥ 95 % of them.
  - `Sim3Test.FixScaleKeepsUnitScale`
  - `Sim3Test.DegenerateFailsCleanly` (Review Focus 4): fewer than 3 pairs, collinear points, all outliers, and a scale ≤ 0 all give `success == false` and a finite pose.
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "geometry::SolveSim3Ransac (Horn, ORB-SLAM3 Sim3Solver)"`.

### Task 3: `OptimizeSim3` and `SearchBySim3Projection`

**Files:**
- Create: `libs/algs/include/voyahslam/optimization/sim3_optimization.h`, `libs/algs/src/optimization/sim3_optimization.cpp`, `libs/algs/tests/optimization/sim3_optimization_test.cpp`
- Modify: `libs/algs/include/voyahslam/features/projection_matcher.h`
- Test: `libs/algs/tests/features/projection_matcher_test.cpp`

**Interfaces:**
- **Consumes:** Task 2 `Sim3`.
- **Produces:**
  ```cpp
  struct Sim3Match { TPoint3DF X1; TPoint2DF pixel1; double sigma1; TPoint3DF X2; TPoint2DF pixel2; double sigma2; };
  struct Sim3OptimizationResult { bool success; Sim3 S_12; std::vector<bool> inlier; size_t num_inliers; };
  Sim3OptimizationResult OptimizeSim3(const Camera& camera1, const Camera& camera2, const std::vector<Sim3Match>& matches,
                                      const Sim3& initial_S_12, bool fix_scale, double huber_chi2 = 10.0);  // ORB-SLAM3 OptimizeSim3: 5 robust iterations, outliers removed, 5 more (10 if outliers were removed).
  // features/projection_matcher.h
  // ORB-SLAM3 SearchBySim3: project frame-2 points into 1 and frame-1 points into 2 through S_12; keep mutual matches.
  template <typename FE> std::vector<std::pair<uint32_t, uint32_t>> SearchBySim3Projection(
      const TMonocularMeasurement<FE>& frame1, const RigidTransformation& T1w, const std::vector<std::optional<MapPointId>>& points1, const std::vector<TPoint3DF>& positions1,
      const TMonocularMeasurement<FE>& frame2, const RigidTransformation& T2w, const std::vector<std::optional<MapPointId>>& points2, const std::vector<TPoint3DF>& positions2,
      const Sim3& S_12, double radius, int max_distance);
  ```
  The radius is the ORB-exact pixel radius `th·1.2^level`, as in `SearchFromFrame`.

- [ ] **Step 1: Failing tests:**
  - `Sim3OptimizationTest.JacobianMatchesFiniteDifference`
  - `Sim3OptimizationTest.RecoversPerturbedSim3`
  - `Sim3OptimizationTest.FixScaleKeepsScale`
  - `Sim3OptimizationTest.RejectsOutliers`
  - `ProjectionMatcherTest.SearchBySim3MutualMatches`: radius pinned at 9 px vs 11 px; only mutual matches are kept.
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.** Use g2o `types_sim3`: add it to the algs PRIVATE links and the Conan g2o components if it is missing.
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "OptimizeSim3 and SearchBySim3Projection (ORB-SLAM3)"`.

### Task 4: `OptimizeEssentialGraph`

**Files:**
- Create: `libs/algs/include/voyahslam/optimization/essential_graph.h`, `libs/algs/src/optimization/essential_graph.cpp`, `libs/algs/tests/optimization/essential_graph_test.cpp`

**Interfaces:**
- **Produces:**
  ```cpp
  struct EssentialGraphProblem {
      std::vector<map::FrameId> frames; std::vector<RigidTransformation> T_cw;     // current (uncorrected) poses
      std::vector<std::optional<Sim3>> corrected_S_cw;                             // set for the loop window
      std::vector<std::pair<size_t,size_t>> edges;                                 // tree + loop + strong covisibility (indices)
      size_t fixed;  bool fix_scale;  int iterations = 20;
  };
  struct EssentialGraphResult { bool success; std::vector<Sim3> S_cw; };           // optimized; caller converts to SE3 (scale folded)
  EssentialGraphResult OptimizeEssentialGraph(const EssentialGraphProblem&);
  ```
  - Following ORB-SLAM3: each edge's measurement is the relative Sim3 between its two vertices' **non-corrected** poses (`Sji = Sjw·Swi`), except edges inside the corrected window, which use the corrected values.
  - Every edge has identity information.
  - LM, with the fixed vertex held constant.

- [ ] **Step 1: Failing tests:**
  - `EssentialGraphTest.RingWithDriftIsCorrected`: 20 Frames in a ring, drift injected, one loop edge. After optimization the end-to-start error is ≤ 10 % of the injected drift.
  - `EssentialGraphTest.FixedVertexStays`
  - `EssentialGraphTest.StereoKeepsScale`
- [ ] **Step 2: Run.** Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all` and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "OptimizeEssentialGraph (ORB-SLAM3 pose graph)"`.

### Task 5: LoopCloser detection

**Files:**
- Modify:
  - LoopCloser: `libs/slam/include/voyahslam/loop_closing/loop_closer.h`, `libs/slam/src/loop_closing/loop_closer.cpp`
  - events: `libs/events/include/voyahslam/events/loop_detected.h`
  - wiring: `libs/slam/{include/voyahslam,src}/slam/slam.*`
- Test: `libs/slam/tests/loop_closing/loop_closer_test.cpp`

**Interfaces:**
- **Consumes:** Task 1 replica graph; Task 2 `SolveSim3Ransac`; Task 3 `OptimizeSim3` and `SearchBySim3Projection`; `SearchByBoW`; `FrameDatabase`.
- **Produces:**
  - `LoopDetected { map::MapId query_map_id, match_map_id; FrameId query_frame, match_frame; Sim3 S_query_to_match; std::vector<std::pair<MapPointId,MapPointId>> matched_points; }`. The LoopCloser sets the Event base `map_id` = `query_map_id`.
  - `LoopCloser(LoopCloserOptions, std::shared_ptr<const bow::Vocabulary<FE>>)` subscribes to the Mapper. It becomes a template on FE like the Tracker.
  - `LoopCloserOptions` holds the spec §4 values.
  - The detection logic is a reusable function, `DetectCommonRegion(replica, query_frame, options)`, so step 5 can reuse it.

- [ ] **Step 1: Failing tests:**
  - `LoopCloserTest.DetectsLoopAfterThreeConfirmations`: a synthetic sequence returns to its start. `LoopDetected` is published only after the 3rd confirmation, with S close to the true drift.
  - `LoopCloserTest.CovisibleRevisitIsNotALoop` (Review Focus 4)
  - `LoopCloserTest.UnmappedLookAlikeRejected`
  - `LoopCloserTest.ClearsOnNewMap` (Review Focus 3)
  - `LoopCloserTest.NoVocabularyDoesNothing`
- [ ] **Step 2: Run.** Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`, the apps, Release and the header check. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "LoopCloser: replica, ORB-SLAM3 common-region detection, LoopDetected"`.

### Task 6: Mapper correction

**Files:**
- Create: `libs/slam/src/mapping/mapper_loop.cpp`
- Modify: `libs/slam/include/voyahslam/mapping/mapper.h`, `libs/slam/src/mapping/mapper.cpp`
- Test: `libs/slam/tests/mapping/mapper_test.cpp` (suite `MapperLoopTest`)

**Interfaces:**
- **Consumes:** Task 1; Task 4 `OptimizeEssentialGraph`; Task 5 `LoopDetected`; the existing fusion code.
- **Produces:**
  - Spec §3 steps 1–8. Steps 3–6 are reusable private functions (`CorrectWindow`, `FuseLoopPoints`, `OptimizeGraph`).
  - `MapperOptions::loop`.
  - `map_epoch` increments on each correction.
  - A stale-epoch `FrameTracked` never creates a Frame.

- [ ] **Step 1: Failing tests:**
  - `MapperLoopTest.CorrectsWindowPointsAndGraph`: synthetic drift plus `LoopDetected`. After correction the error at the loop is within a bound and the replica stays consistent.
  - `MapperLoopTest.FusesLoopPoints`
  - `MapperLoopTest.PublishesOneTransactionWithEpoch`
  - `MapperLoopTest.StaleEpochCreatesNoFrame`
  - `MapperLoopTest.RejectsLoopWithCulledFrames` (Review Focus 1)
  - `MapperLoopTest.IgnoresDuplicateLoop` (Review Focus 2)
  - `MapperLoopTest.IgnoresLoopOfOldMap` (Review Focus 3)
- [ ] **Step 2: Run.** Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`, the apps and Release. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper: loop correction (CorrectLoop, fusion, essential graph, epoch)"`.

### Task 7: Tracker follows the correction

**Files:**
- Modify: `libs/slam/include/voyahslam/tracking/tracker.h`, `libs/slam/src/tracking/tracker.cpp`
- Test: `libs/slam/tests/tracking/tracker_test.cpp` (suite `TrackerLoopTest`)

**Interfaces:**
- **Consumes:** Task 1's `map_epoch` and `FrameTracked::map_epoch`.
- **Produces:**
  - On a transaction with a new epoch, while Tracking: `T_last ← T_last · T_ref_old⁻¹ · T_ref_new`. Use the reference Frame's pose before and after the apply; mono applies the same SE3, since the scale is already folded.
  - `FrameTracked` publishes the replica's epoch.

- [ ] **Step 1: Failing tests:**
  - `TrackerLoopTest.LastFrameFollowsReferenceAfterCorrection`: the next frame is tracked, with no loss.
  - `TrackerLoopTest.FrameTrackedCarriesEpoch`
  - `TrackerLoopTest.CorrectionWhileRecentlyLost` (Review Focus 5)
- [ ] **Step 2: Run.** Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Tracker: follow loop corrections; FrameTracked carries map epoch"`.

### Task 8: End to end, smoke, docs

**Files:**
- Modify:
  - apps: `examples/full_slam/*` (`--no_loop_closing`)
  - tests: `libs/slam/tests/integration/slam_test.cpp`
  - docs: `docs/DESIGN.md`, `docs/TODO.md`, `docs/ROADMAP.md`

- [ ] **Step 1: Integration tests:** `SlamTest.{Mono,Stereo}ClosesLoop`.
  - A synthetic loop with drift, where mono drift includes scale.
  - The end-of-run error is ≤ 1/3 of the same run without loop closing.
  - Measure 3 runs and put the numbers in the commit message.
- [ ] **Step 2: ThreadSanitizer** on `voyah_slam_tests` and `voyah_integration_tests`. Expected: no warnings.
- [ ] **Step 3: Smoke runs** (Release, `--vocabulary`, full corridor1, 2 runs each, with and without `--no_loop_closing`):

  | Run | Must | Report |
  |---|---|---|
  | Mono full | Loop detected on return to the room and corrected; one map; ATE (Sim3) below the no-loop run | loop frame, correction ms, ATE both ways, Frames, points |
  | Stereo full | Same; no tracking loss across the correction | same, ATE (SE3) |

  A miss is DONE_WITH_CONCERNS with a diagnosis. Nothing is tuned.
- [ ] **Step 4: Docs** (spec §6):
  - `DESIGN.md`: the LoopCloser replica and detection; the spanning tree and loop edges; correction, the epoch and the Tracker shift; the essential graph; the smoke table; decisions-log entries dated 2026-10-09.
  - `TODO.md`: global BA (4c), map merging (step 5), scale without IMU.
  - `ROADMAP.md`: 4a+4b done.
- [ ] **Step 5: Run** `check_all`, the apps and Release. Expected: green.
- [ ] **Step 6: Commit** `git commit -m "Loop closing: end-to-end tests, smoke runs, docs"`.
