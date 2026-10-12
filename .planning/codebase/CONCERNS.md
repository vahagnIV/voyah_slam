---
last_mapped_commit: b0e957ce59ec197bb96ec9ccbcc97cf037513c8e
last_mapped_at: 2026-10-12
---
# Codebase Concerns

**Analysis Date:** 2026-10-12

## Tech Debt

### ASan Compatibility Issue with g2o

**Issue:** ASan runs abort in g2o with "attempting free on address which was not malloc()-ed" from `g2o::Solver::~Solver()` (via `OptimizePose`).

**Root Cause:** ODR (One Definition Rule) violation. When `__SANITIZE_ADDRESS__` is set, Eigen 3.4 disables `EIGEN_GLIBC_MALLOC_ALREADY_ALIGNED`, causing the ASan-compiled code to use Eigen's `handmade_aligned_malloc` while non-ASan g2o frees with its own `aligned_free` (plain `free`), or vice versa. This creates memory allocator mismatches.

**Files:** `libs/algs/tests/optimization/bundle_adjustment_test.cpp` and all code using `OptimizePose` or `BundleAdjust` via g2o.

**Impact:** ASan (Address Sanitizer) testing cannot run with the current build setup. The library is suitable only for Release builds and basic Debug testing without ASan.

**Fix Approach:** 
- Option A: Build g2o with `-fsanitize=address` for ASan runs
- Option B: Define `EIGEN_MALLOC_ALREADY_ALIGNED=0` consistently across both Eigen and g2o in ASan configuration
- **Recommended:** Option A (ensure g2o build inherits ASan flags)

---

### Event Pool Lifetime and Ownership

**Issue:** Components own pools of events they publish, but consumers can hold references to pooled events. Without care, on exception unwinding during shutdown, producer and consumer destructors may conflict over freed pool memory.

**Files:** `libs/events/include/voyahslam/events/object_pool.h`, `libs/slam/include/voyahslam/slam.h`, `apps/full_slam/` (guards in place)

**Impact:** If a consumer (e.g., external visualizer or JNI bridge) outlives its producer, `Notify` may push into freed memory. Library users must manually guard with `slam.Stop(); viewer.Stop()` on all exit paths.

**Current Mitigation:** Applications (not the library) guard against this. The library has no built-in lifetime management.

**Fix Approach:** Design pools that outlive all producers and consumers (global/static lifetime, or heap-allocated with explicit lifecycle management).

---

### Subscriber Lifetime and Disconnection

**Issue:** Subscribers (async and sync observers) are stored as raw pointers with no disconnect mechanism. A subscriber dying before its publisher leaves `Notify` pushing into freed memory.

**Files:** 
- `libs/events/include/voyahslam/events/observable.h` - stores `AsyncObserver*`
- `libs/events/include/voyahslam/events/wake_signal.h` - stores `WakeSignal*`

**Impact:** External observers (e.g., visualizer, logging, JNI bridge) that attach via `AddObserver` must outlive their observable, or corruption occurs.

**Current Mitigation:** None. Documented only implicitly in app guards.

**Fix Approach:** 
- Implement RAII connection handles that disconnect on destruction
- Use `shared_ptr` for observers to allow out-of-order destruction
- Document explicitly in `Observable` class comments

---

### Pool Overflow Recovery

**Issue (Fixed in 0.3.0):** Event pools previously aborted on overflow; now they grow dynamically (logged and counted). However, this creates unbounded memory growth if an async observer stalls (e.g., a stuck visualizer thread).

**Files:** `libs/events/include/voyahslam/events/object_pool.h`, commit `0301690`

**Impact:** A stuck observer can cause Tracker and Mapper pools to grow without bound, consuming available memory.

**Partial Mitigation:** Since 0.3.0, pools grow instead of aborting (less catastrophic, but still a concern).

**Fix Approach:** Implement **lossy delivery for display-only observers**: add a delivery mode (e.g., `AddObserver(o, Delivery::Lossy)`) with a bounded queue that drops and releases the oldest events, for viewers and logging. Never apply this to `MapTransaction` replicas or the Mapper's `FrameTracked` input (those need all events).

---

## Known Bugs

### Loop Closing False Positives on Repetitive Scenes

**Issue:** Loop closing corrects false loops on TUM-VI corridor1 where the corridor pattern repeats with the same heading 7–30 m apart. These false corrections make accuracy worse than without loop closing.

**Files:** `libs/slam/src/loop_closing/`, `docs/TODO.md` line 650

**Symptoms:** 
- Same-heading look-alikes (frames 2215–2236 vs 692–807 are 14.9 m apart, normals agree)
- False loops have exit coordinates 9.2 m / 176° apart, but Sim3 claims 0.2 m / 6°
- After correction: mono ATE 0.679 / 0.669 m (vs 0.341 / 0.500 m without loop closing), stereo 1.295 / 18.16 m (vs 0.098 / 0.138 m)

**Trigger:** Normal-consistency gate (added 2026-10-10) rejects opposite-facing false loops but **not** same-heading ones. ORB-SLAM3 visual gates fail identically on this dataset.

**Workaround:** Loop closing is opt-in and off by default (user decision 2026-10-10) until this is fixed.

**Fix Approach (deferred to roadmap step 6):** 
1. **Drift-bound gate:** Reject loops whose query and match lie farther apart in the map than drift could explain (bound per metre travelled; mono needs scale-aware bounds)
2. **Verify-before-publish:** Check reprojection error after essential-graph correction or next local BA; drop corrections that break the map
3. **Both together** (recommended)

---

### Flaky Integration Tests

**Issue:** Three pre-existing integration tests fail intermittently under parallel load or with timing-dependent variations.

**Files:** `libs/slam/tests/integration/slam_test.cpp`

**Test failures (100-run stress test on master and branch):**

1. **`SlamTest.StereoMapGrowsWhileLeavingInitialView`**
   - Failure rate: 2/60 runs under parallel load on master
   - Failed once in `check_all` on the loop-closing branch
   - Symptoms: Tracking loss or point count mismatch
   - Root cause: Thread timing-dependent (Mapper pace changes the observed map state)

2. **`SlamTest.DuplicatesFusedInLongSequence`**
   - Failure rate: 2/100 runs (master and branch consistent)
   - Symptoms: Either a `TrackingLost` during the run with mismatched alive-point count, or one 1 mm duplicate left unfused
   - Root cause: Thread timing; fusion gates leave some near-duplicate pairs (centimetre-apart with noise)

3. **`SlamTest.MonoRelocalizesAfterBlackout`**
   - Failure rate: 3/100 runs on master (2 hangs), 4/100 on branch (2 hangs)
   - Symptoms: 
     - Second non-final loss `tracking_lost` {1, 1}
     - Or 120 s hang: Tracker stays in `RecentlyLost` at input end
   - Root cause: Harness waits for final `TrackingLost` or last frame publish; neither arrives if Tracker stays in `RecentlyLost`

**Impact:** Tests pass locally and in `check_all` but fail in parallel stress runs. Not blocking releases but indicate race conditions or timing-dependent logic.

**Fix Approach:**
- Make harness end on "input done and Tracker idle" (not "final event published")
- Use `RunSetup::lock_step` (used by loop tests) to make runs deterministic
- Widen error bounds on load-sensitive tests (e.g., `StereoRelocalizesAfterBlackout` margin 1.5×)

---

## Fragile Areas

### Loop Closing with Global BA Memory Overhead

**Issue:** Global BA adds another full replica plus the g2o problem, doubling memory usage.

**Files:** `libs/slam/src/global_ba/`

**Measurements (2026-10-10 smoke run):**
- **Stereo corridor1:** 2.28–2.33 GB without global BA (1649–1709 Frames, ~101–106k points) → **4.23 GB with it** (1758 Frames, 108k points)
- **Mono:** 1.51–1.47 GB without → 2.24–2.38 GB with
- **After shared measurements fix (2026-10-10):** stereo peak dropped to **1.21 GB** (1794 Frames, 114k points; 1.7 GB before, a 30% reduction)

**Impact:** Large-scale runs (>2000 stereo frames) approach or exceed typical 4–8 GB memory budgets.

**Investigation Needed:** Determine what dominates (measurements, BoW, per-replica copies, BA problem) before trimming.

---

### Global BA Execution Time

**Issue:** Global BA iterations are expensive and can be interrupted mid-run, losing work.

**Files:** `libs/slam/src/global_ba/global_bundle_adjuster.cpp`

**Measurements (2026-10-10 corridor1 release):**
- Mono: 3.3–28.2 s per run (10 iterations)
- Stereo: up to 82.5–85.2 s per run (1465–1654 Frames, 92–103k points)
- 3 of 4 mono runs were interrupted by the next loop correction after 15–35 s (work lost)
- Interruption landing point: only at next LM iteration (up to ~6 s on largest maps)
- `Slam::Stop()` waits for current LM iteration: 6–8 s on full corridor1 stereo map

**Impact:** Large maps stall tracking for >80 s. Interruptions waste work. Stop operations block for seconds.

**No cap in ORB-SLAM3 either.** 

**Fix Approach:** 
- Fewer iterations on large maps (dynamic iteration cap based on Frame count)
- Time budget per run
- Polling inside an iteration (allow interruption finer-grained)

---

### Merge Weld Performance

**Issue:** `WeldWindows` (merge step 5) is non-interruptible, blocking the Mapper and any pending `FrameTracked`.

**Files:** `libs/slam/src/mapping/mapper_merge.cpp`, line "WeldWindows"

**Measurements (2026-10-11 corridor1):**
- 202–247 ms per merge on windows of 25–30 and 26 Frames
- Cannot be interrupted (ORB-SLAM3 passes a stop flag never set; ours does not use one)

**Impact:** While merging, the Tracker stalls waiting for a `MapTransaction` and input blocking can occur.

**Fix Approach:** Make the weld interruptible for merges on larger windows, or cap the weld window size.

---

### ORB Extraction Performance on arm64

**Issue:** NEON FAST path (`fast_neon.cpp`) has not been tested on real arm64 hardware, only on x86-64 via SIMDe emulation (and cross-compiled for Android NDK).

**Files:** `libs/algs/src/features/fast/fast_neon.cpp`, line 12

**Tests Needed:**
- Build and run on arm64 Linux or Android device
- Compare with `cv::FAST` on the same device
- Verify bit-exactness with golden digests on arm64

**Impact:** NEON path may be incorrect or suboptimal on real arm64, falling back to C++ implementation in timing-critical feature extraction.

---

### Raw-Pixel Residual Camera Model Edge Cases

**Issue:** The 85° ray-angle cutoff assumes distorted radius grows monotonically with ray angle; Brown-Conrady violates this (non-radially symmetric).

**Files:** `libs/algs/src/tracking/monocular_measurement.cpp`, method `DropFeaturesBeyondMaxRayAngle`

**Symptoms:** With some Brown-Conrady coefficients (k1 < 0, small k2), Distort(tan 85°) can fold back, causing the angle test to keep or drop the wrong features.

**Impact:** Harmless on EuRoC (no features near 85°); could break on very wide-FOV optics.

**Fix Approach:** Define a per-model "valid region" rather than assuming monotonicity.

---

## Performance Bottlenecks

### LoopCloser Detection Time

**Issue:** Loop detection can be slow on large maps, potentially dropping queries if the backlog exceeds `max_query_backlog` (8).

**Files:** `libs/slam/src/loop_closing/`, `query_stats`

**Measurements (2026-10-10):**
- Synthetic loop scenes: 4.5–9.3 ms mean, 26 ms max (Debug -O2)
- Corridor1 release mono: 55–133 ms (mean/max under load)
- Corridor1 release stereo: 43.5–96.5 ms
- None dropped on corridor1

**Impact:** Queries beyond the backlog are silently dropped; detection work is wasted.

---

### Map Point ID Scanning

**Issue:** Tracker scans all point IDs per frame, even culled ones. Point IDs are append-only (removal only marks dead), so storage grows with every point ever created.

**Files:** `libs/slam/src/tracking/tracker.cpp`, loop over map point IDs

**Measurements (5000 stereo frames estimate):**
- ~140 MB memory growth from fusion and culling (ids ~6× live points)
- ~540k ids scanned per frame

**Impact:** Long runs accumulate dead IDs, increasing per-frame tracking time and memory.

**Fix Approach:** 
- Compact point storage with id remap event
- Index live points separately (skip dead ones in scanning)

---

### Stereo Tracking Test Margin

**Issue:** Stereo tracking test has thin error margin: 1.79 cm error against 2 cm bound.

**Files:** `libs/slam/tests/tracking/`, margin assertion

**Impact:** Margin leaves little room for benign variations; test may flake under different compiler/hardware.

---

### Relocalization Candidate Verification

**Issue:** Relocalization verifies every candidate group at ≥0.75× best score, costing up to C × 60 verifications (C candidates × 60 rounds, 1–4 ms per round with 3× `OptimizePose` and 2× `SearchFromFrame`).

**Files:** `libs/slam/src/tracking/relocalizer.cpp`

**Impact:** High CPU cost for failed relocalization attempts; can take seconds per measurement.

**Fix Approach:** Cap relocalization candidates (e.g., top 10 groups) or bound time per measurement.

---

## Scaling Limits

### Point ID Storage Growth

**Issue:** `MapPointStorage` ids are append-only (removal marks dead only), so storage size grows with every point ever created in a map. Culling and fusion leave empty slots.

**Files:** `libs/slam/include/voyahslam/map/`, `MapPointStorage`

**Problem:** Long runs create thousands of IDs; IDs never removed, only marked dead. Slots indexed by id stay allocated.

**Impact:** A 5000-frame mono run accumulates ~140 MB from fusion/culling (ids ~6× live points, ~540k ids scanned per frame).

**Fix Approach:**
- Compact storage with id remap event (requires distributed remap to all replicas)
- Switch to a new map when slot utilization drops below threshold
- Index live points separately

---

### Culled Frames Array Fragmentation

**Issue:** Culled Frames leave empty slots in the Mapper's Frame array. IDs are consecutive per map (lookup by id is O(1)), but array grows without bound.

**Files:** `libs/slam/src/mapping/mapper.cpp`, `Frames_` array

**Impact:** Long runs with aggressive Frame culling (synthetic mono unit: 3 Frames remain from ~30) accumulate stale slots.

---

### Global BA Memory on Large Maps

**Issue:** Global BA peaks at 4+ GB on corridor1 stereo. The memory profile (measurements, BoW, per-replica copies, BA problem) is not fully characterized.

**Files:** `libs/slam/src/global_ba/`

**Current Status:** After shared measurements (2026-10-10), stereo corridor1 dropped from 1.7 GB to 1.21 GB (30% win), but still allocates full replicas and BA problems per correction.

---

## Thread Safety and Concurrency

### Map ID Correctness Under Replica Lag

**Issue:** Replicas apply transactions from a lagging state. Stale map IDs in events from lagging replicas cannot always be detected if they are newer than the replica's current state.

**Files:** `libs/slam/`, replica rule §6

**Mitigation:** ID non-reuse within a run (once a map is deactivated, its id is never reused). Stale old IDs are detectable; newer IDs arriving early cannot be rejected.

---

### TrackerOptions and ProjectionMatcher Level Synchronization

**Issue:** `TrackerOptions::level_scale` and `ProjectionMatcherOptions::level_scale`/`max_level` are kept in sync manually; any change to the extractor requires updates in three places.

**Files:** 
- `libs/slam/include/voyahslam/tracking/tracker.h` line 91 TODO
- `libs/algs/include/voyahslam/features/projection_matcher.h`
- Feature extractor implementation

**Impact:** Manual synchronization is error-prone; a change to extractor scale factors can silently break matchers.

**Fix Approach:** Derive `level_scale` and `max_level` from the feature extractor class at compile time.

---

### Slam Member Declaration Order Dependency

**Issue:** Tracker copies vocabulary pointer before Mapper's constructor moves it; correctness depends on declaration order (`tracker_` before `mapper_`).

**Files:** `libs/slam/include/voyahslam/slam.h`

**Symptoms:** If `mapper_` is declared before `tracker_`, the move happens before the copy, leaving `tracker_` with a dangling pointer.

**Fix Approach:** Pass vocabulary by copy to both, or add a static assertion or explicit comment.

---

## Undefined Behavior Risks

### RANSAC Min/Max Iteration UB

**Issue:** `ransac.h` `std::clamp` is UB when `min_iterations > max_iterations` (not guarded).

**Files:** `libs/algs/src/estimators/ransac.h`

**Fix Approach:** Compute bounds once with `std::min`/`std::max` at initialization.

---

### Essential Estimator NaN Reprojection Error

**Issue:** `SelectPose` counts a NaN reprojection error (z exactly 0, undefined depth) as good; no guard.

**Files:** `libs/algs/src/estimators/essential_estimator.cpp`

**Fix Approach:** Use `!(err <= max)` guard to reject NaN.

---

### g2o Behind-Camera Projection

**Issue:** g2o edges call `Camera::Project` before checking depth (z > 0 precondition), allowing z ≤ 0 → NaN mirrored values. Also, `Camera::Project` heap-allocates per call (dynamic 3×1 for `Distort`) in the hot path.

**Files:** `libs/algs/src/optimization/`, calls to `Camera::Project`

**Impact:** NaN propagates into g2o problems; allocation overhead in loop-heavy code.

**Fix Approach:**
- Check depth first before `Project`
- Add allocation-free fast path for `Project` and `Unproject`

---

### Camera Intrinsics Validation

**Issue:** `OptimizePose`, `BundleAdjust`, and `Camera` do not validate K beyond finiteness (fx, fy > 0).

**Files:** 
- `libs/algs/src/optimization/optimize_pose.cpp`
- `libs/algs/src/optimization/bundle_adjust.cpp`
- `libs/common/include/voyahslam/geometry/camera.h`

**Risk:** Invalid focal lengths or NaN can propagate silently into optimization.

**Fix Approach:** Add explicit validation: fx, fy > 0, cx/cy finite, K determinant > 0.

---

### Backwards Timestamps in RecentlyLost

**Issue:** `RecentlyLost` timeout never fires for backwards timestamps; elapsed time is negative.

**Files:** `libs/slam/src/tracking/relocalizer.cpp`

**Risk:** Tracker can stay in `RecentlyLost` indefinitely if timestamps jump backwards.

**Fix Approach:** Treat backwards jump as timeout or reset.

---

## Test Coverage Gaps

### Loop Closing Verification

**Untested areas:**
- `SearchBySim3Projection` depth-range and behind-camera gates
- Radius pinning only at level 0 and in x
- `OptimizeSim3` with rejects-outliers on gross outliers only (no medium-strength outliers)
- SolveSim3Ransac mirror-degenerate scenes (peak 11/20 inliers)
- `OptimizeEssentialGraph` with fix-scale-off and non-unit corrected loop edge
- Covisible-confirmation path of the normal gate (candidate and refinement paths tested)

**Files:** `libs/slam/tests/loop_closing/loop_closer_test.cpp`

**Impact:** Loop closing code could have silent bugs in edge cases not covered by synthetic scenes.

---

### Global BA Snapshot Contract

**Issue:** `ChangesDuringRunDoNotAffectResult` only catches when data it reads changed. The snapshot contract is documented but not enforced.

**Files:** `libs/slam/tests/global_ba/global_bundle_adjuster_test.cpp`

**Untested:** Returns nullopt on non-finite result, robust=false keeping outliers, single intrinsics in problem tests.

---

### Map Replica Covisibility

**Issue (TaskMapReplicaTest findings):** 
- Tracker's `observed_points` count compared only; actual points not checked
- Atomic-apply test does not check updated-before-Frame order or point 3's distance
- No test helpers creating featureless carrier Frames (currently used)

**Files:** `libs/slam/tests/tracking/map_replica_test.cpp`

**Impact:** Replica consistency bugs could hide in light test coverage.

---

### Mono Tracker Translation Accuracy

**Issue:** Mono translation accuracy at default initialization is about 10% at 1° parallax (needs tuning).

**Files:** `libs/slam/tests/integration/slam_test.cpp`, mono initialization tests

**Status:** Tuning deferred; no test pinning the accuracy bound.

---

### Triangulation Filter Tests

**Untested:**
- Rotation histogram
- One-to-one drop losing valid second candidate
- `th_low` threshold
- Ratio test and positive depth (no test fails if z > 0 check removed)
- Stereo fallback with depth on side b
- Neighbour-skip rules
- At most one new point per new-Frame feature across neighbours
- Fisheye test cannot distinguish Ruling F band from old one

**Files:** `libs/slam/tests/mapping/mapper_test.cpp` (map growth section)

**Impact:** Triangulation code could silently regress in edge cases.

---

### Bag of Words Vocabulary Tests

**Gaps:**
- No leaf at depth between L − levels_up and L
- `RotatedOffsetsAreRounded` is partly self-referential (only 5,9 case pins)
- `SearchByBoW` does not pin ratio test among shared-node candidates
- No stereo BoW test; FeatureVector and `levels_up` asserted indirectly

**Files:** `libs/algs/tests/bow/vocabulary_test.cpp`

---

### Relocalization Test Margins

**Issues:**
- `SlamTest.StereoRelocalizesAfterBlackout` is load-sensitive (margin 1.5×0.0217 m can fail 1/20 runs under load)
- `SmallMapGoesStraightToLostByDefault` uses synthetic maps (3–5 Frames) that do not exercise default 10-Frame rule
- Motion-model gate test forces 100000 (no 10–29 band / exactly-30 case)

**Files:** `libs/slam/tests/tracking/tracker_test.cpp`

---

## Missing Critical Features

### Bearings Beyond 85° FOV

**Issue:** Current implementation uses z=1 normalized points with 85° cutoff. Full fisheye support requires bearing vectors.

**Files:** `libs/algs/src/tracking/monocular_measurement.cpp`, `DropFeaturesBeyondMaxRayAngle`

**Status:** Deferred; interim 85° cutoff in place (OQ 22).

---

### 5-Point Essential Solver

**Issue:** Current 8-point solver is ambiguous on narrow-FOV and wide-FOV tests (wide-FOV not representative; ORB-SLAM3 tuned on narrow).

**Files:** `libs/algs/src/estimators/essential_estimator.cpp`

**Priority:** Raised (narrow-FOV robustness); none implemented yet.

---

### Refinement Inlier-Set Lock-In

**Issue:** Least-squares over hard-thresholded inliers (χ² 3.841) drops true inliers and keeps 2–3 outlier levers (translation error 3–10° on half of seeds, worst reaches 10°). LM converges but more rounds do not help.

**Files:** `libs/algs/src/estimators/essential_estimator.cpp`, `Refine` method

**Remedy:** Robust-loss local optimization (Cauchy/Huber or truncated-χ²) over all correspondences, like MSAC/MAGSAC. BA polishes but does not fix lock-in.

---

### ORB Extraction FAST Cell Assignment

**Issue:** Currently FAST is run per cell; after porting FAST to run once per pyramid level (2026-10-11), need to assign corners to cells (≥20 if any, ≥7 else) and re-record golden digests.

**Files:** `libs/algs/src/features/fast/fast.h`, `libs/algs/src/features/orb_feature_extractor.cpp`

**Impact:** Removes blind strips at cell borders but changes extractor output.

---

### Contrast Equalization (CLAHE)

**Issue:** ORB-SLAM3 applies CLAHE(3.0, 8×8) to every TUM-VI image; none in voyahslam yet.

**Files:** Preprocessing step, not yet implemented

**Status:** Deferred to app-side (not in SLAM library).

---

### Scale Correction Without IMU

**Issue:** Mono scale drifts freely between loops (only corrected at loop). Visual-inertial (ORB-SLAM3 IMU init) out of scope.

**Files:** `libs/slam/src/loop_closing/`, initialization

**Status:** Known limitation; no IMU support planned.

---

## Security Considerations

### DBoW2 License and Phantom Node

**Issue:** `ORBvoc.txt` is used as-is but licensing terms must be settled before distribution. Also, DBoW2 has a phantom node bug (971,815 words vs 971,814 correct), creating uninitialised values (UB).

**Files:** `ORBvoc.txt` (not committed, user-provided), `libs/algs/src/bow/vocabulary.cpp`

**Risk:** 
- Phantom node allows features to be assigned incorrect word weights
- Licensing unclear

**Status:** Deferred user decision (2026-10-07); terms not settled.

**Workaround:** Vocabulary training not yet implemented; only ORB extractor has a vocabulary.

---

### ORB Descriptor Distance Overflow

**Issue:** ORB `Distance` returns `uint8_t`; 256-bit difference wraps to 0, potentially taking farthest for closest.

**Files:** `libs/algs/src/bow/`, ORB distance computation

**Not observed** on DBoW2 fixture (no child distance 256).

**Fix:** Widen return type.

---

## Environmental and Build Concerns

### OpenCV 5.1 Branch Name in README

**Issue:** README clone command uses `--branch 5.1.0`, which doesn't exist on GitHub (only tags 5.0.0 and 5.0.0-alpha).

**Files:** `README.md`

**Workaround:** Dev machine uses branch `5.x` at commit `20e367198c` (5.1.0-dev). FAST port and tests validated against this.

**Fix:** Pin README to that commit once a release tag exists.

---

### Symbol Visibility and Shared Build on Windows

**Issue:** `.so`s export all symbols (~7000 spdlog/fmt symbols among them); no visibility control. Windows `shared` builds rejected by `validate()`.

**Files:** `conanfile.py`, build rules

**Fix Approach:** 
- Build with `-fvisibility=hidden` and `CXX_VISIBILITY_PRESET hidden`
- Export macros (`GenerateExportHeader`)
- Allows `shared` on Windows

---

### Stale Build Directories

**Issue:** After restructure (2026-10-07), Debug and Release build trees still contain old `src/` and `tests/` directories.

**Files:** `cmake-build-debug/build/Debug`, `build/build/Release`

**Impact:** Stale artifacts; potential confusion during clean rebuilds.

---

---

*Concerns audit: 2026-10-12*
