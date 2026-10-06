# TODO

Concrete pending work. Reasoning and background live in `docs/DESIGN.md`
(referenced as "OQ n" = Open Question n). Remove items when done; add new ones
as soon as they come up.

## Performance

- [ ] **Profile 8-bit vs float working images.** With OpenCV 5.1, 8-bit is
      ~8% slower than float (≈ 76 vs 70 ms per mono 1024² frame). Suspects:
      per-pixel `std::round` in `BilinearInterpolation`, 8-bit → float
      conversion per tap in `images::Filter`. Profile both variants before the
      extractor rework. (OQ 21)
- [ ] Benchmark on TUM-VI **512×512** (`dataset-corridor1_512_16`, being
      downloaded) — the resolution ORB-SLAM3's configs and timings use.
- [ ] Optionally build ORB-SLAM3 against OpenCV 5.1 with `REGISTER_TIMES` and
      run it on the 512 dataset as a reference.

## ORB extractor (OQ 21)

- [ ] Own FAST-9-16, run once per pyramid level; assign corners to cells
      (≥ 20 if any, else ≥ 7). Removes OpenCV from detection and the blind
      strips at cell borders.
- [ ] Preallocate blur and pyramid buffers (no per-frame image allocations).
- [ ] Round rotated descriptor pattern offsets (ORB-SLAM3 uses `cvRound`; we
      truncate).
- [ ] Check border handling (ORB-SLAM3 pads levels by 19 px).
- [ ] Remove the unused `cv::minMaxLoc` per cell.
- [ ] Parallel left/right extraction for stereo.

## Dataset / preprocessing

- [ ] Contrast equalization (own CLAHE) as an app-side preprocessing step —
      ORB-SLAM3 applies CLAHE(3.0, 8×8) to every TUM-VI image.
- [ ] Radial-tangential distortion model for EuRoC MAV. (OQ 23)

## Geometry

- [ ] Bearing vectors instead of z = 1 normalized points (full fisheye FOV);
      interim 85° cut-off in place. Decide with the Tracker design. (OQ 22)
- [ ] **5-point essential solver** (priority raised: narrow-FOV robustness; the
      8-point solver is ambiguous on narrow FOV and wide-FOV tests are used).
- [ ] Fix refinement inlier-set lock-in: least squares over the hard-thresholded
      (χ² 3.841), self-selected inlier set drops true inliers and keeps 2-3
      outlier levers (translation-direction error 3-10° on about half of the
      seeds; on the worst seeds the true E outscores the returned model). LM
      converges; more rounds or a looser re-collection threshold do not help.
      Remedy: robust-loss local optimization over all correspondences
      (Cauchy/Huber or a truncated-χ² loss matching the score; MSAC/MAGSAC-style
      LO). Bundle adjustment after initialization (done 2026-10-05) polishes the result but not the lock-in.
- [ ] Optional: run the H and E RANSACs in parallel.

## Initialization (monocular, 2026-10-04)

- [ ] Tuning on TUM-VI 512 (`dataset-corridor1_512_16`) after the Pangolin
      viewer, plus a per-frame "dropped by the 85° rule" statistic.
- [ ] Tuning: rescale to median depth 1 *before* the initialization BA (faster
      convergence than the current 5 + 40 iterations on the unscaled problem).
- [ ] Tuning: initialize later / require more parallax. The absolute accuracy of
      the initialization is noise-limited at the first accepted pair (synthetic
      seeds initialize at frame 3, baseline 0.15 vs depth 4-8; the ML optimum is
      0.3-2.7 degrees off), so BA cannot improve on it.
- [ ] `OrbFeatureExtractor::Distance` returns `uint8_t`, so a 256-bit
      difference wraps to 0 — widen the return type.

Smoke run note (2026-10-05, after the initialization bundle adjustment, same
dataset/flags, Release): mono still initializes at frame 121 (not at 120) with
73 points; debug line `Initialization BA: cost 25.37 -> 21.06 in 36 iterations,
0 outliers removed, 73 points survive`. Binocular (`--max-frames=20`) unchanged:
355 points. (The debug line needs `Logger::SetLevel(debug)`, which the apps do
not call; a temporary local change was used.)

Smoke run note (2026-10-04, `dataset-corridor1_1024_16`, ORB, Release,
`--rate=0 --max-frames=200`): mono initialized at about frame 121
(`Initializing -> Mapping (map 1, 2 keyframes, 73 points)`; 73 points is low,
tuning deferred). Binocular (`--max-frames=20`) initialized on the first frame
(`Initializing -> Mapping (map 1, 1 keyframes, 355 points)`); median point
depth 1.96 m (quartiles 1.50 / 2.13 m, range 0.25-9.50 m). The viewer run
(`--viewer`) completed without errors; no screenshot (X_GetImage failed on the
root window).

Smoke run note (2026-10-05, tracking, Release, apps at debug level,
`dataset-corridor1_1024_16`): binocular `--rate=0 --max-frames=200`: init on the
first frame (355 points), tracked all frames (198 local-map successes, median
189 inliers, range 38-243), no loss, 1 map. Monocular `--rate=0
--max-frames=400`: init at about frame 120, then repeated loss after 1-11
tracked frames (first loss after 2: `local map failed: 28 matches (11 new), 26
inliers (< 30)`), re-initialization each time, 8 maps (67-94 points each),
median 36 local-map inliers. Cause: the initialization map is small and never
grows (no keyframes yet, sub-project 2). `--viewer` screenshot capture failed
again (xwd/import on the Wayland session).

## Camera model (raw-pixel residuals, 2026-10-05)

See DESIGN "Camera Model and Raw-Pixel Residuals". Smoke run 2026-10-06
(Release, corridor1 1024, `--rate=0`): mono 400 frames 82 tracked / 18
re-initializations (19 maps, 62-109 points), gate 20: 137 / 14; stereo 200
frames, init on frame 1 with 358 points, no loss, median 232 inliers.

- [ ] Fallback note: a calibration-free angle-dependent noise allowance
      (σ · 1.5 · (1 + r²), window · (1 + r²); spike 2: within 0.6 % of the KB
      stretch up to 60°, 85 tracked frames / 99 with gate 20) for lenses
      without a distortion model. Not implemented.
- [ ] Keep features beyond 85° (bearing vectors on the ray side; residuals are
      already in raw pixels). (OQ 22)
- [ ] Tuning: the local-map inlier gate (30; 20 gives 137 vs 82 tracked mono
      frames) and the mono initial map size (62-109 points). Mono misses the
      ≤ 5 re-initializations criterion (18); see the init-acceptance item.
- [ ] Per-model camera classes (approach B, ORB-SLAM3 `GeometricCamera`), only
      if ever needed (e.g. several lens models in one rig).
- [ ] Behind-camera edges: g2o edges call `Camera::Project` before the depth
      check, violating its `z > 0` precondition (finite mirrored value; z == 0
      gives NaN). Check depth first.
- [ ] `Camera::Project` heap-allocates (dynamic 3×1 for `Distort`) per call in
      the hot path (g2o edges, matchers); add an allocation-free path.
      `Unproject` also inverts K per call.
- [ ] Deduplicate the TUM-VI Kannala-Brandt test fixture (copied in 5–9
      test files) into `tests/support`.
- [ ] Validate K beyond finiteness (fx, fy > 0) in `OptimizePose`,
      `BundleAdjust` and `Camera`.
- [ ] Test that `BundleAdjust` uses each keyframe's own camera (keyframes with
      different focal lengths); today's BA tests share one K.
- [ ] Fisheye test pinning `MapInitializer`'s grid and `last_matched_` on
      `distorted_points()` (map_initializer.cpp); pinhole test helpers build
      the grid on `points()`.
- [ ] Init acceptance in raw pixels: the raw-px init matcher (window 100 px,
      effectively wider in the periphery) accepts many adjacent-frame pairs
      (19 maps vs 6 with the undistorted-px matcher; 11 adjacent-frame inits
      tracked ~2.5 frames). Consider a minimum reference age / parallax or an
      init-quality check before accepting.
- [ ] Instrument first-frame map deaths: three adjacent-frame maps (smoke run
      2026-10-06, ≈ frames 311–318) found 39–47 zero-motion matches but
      `OptimizePose` round 0 kept 0–1 inliers. Log the frame gap to keyframe 2
      and the per-match reprojection error at the predicted pose.
- [ ] Viewer: `ProjectToImage` (`vizualization/include/voyahviz/keypoint_drawing.h`)
      recomputes raw pixels as K·Distort(normalized); draw
      `distorted_points()` directly, drop the separate distortion-model
      argument and the stale "use ProjectToImage()" comment.
- [ ] Test `Camera::ProjectJacobian` exactly on the optical axis
      (Xc = (0, 0, z) → K₂ₓ₂·[1/z, 0, 0; 0, 1/z, 0]).
- [ ] Tests under `tests/geometry` for the moved distortion models still use
      namespace `test::features`; rename to `test::geometry`.

## Stereo (2026-10-04)

- [ ] Epipolar-guided stereo search (optimization of the brute-force matcher).
- [ ] Tuning on TUM-VI 512, including the ~17 m stereo depth cap implied by
      `max_parallax_cos` 0.99998 at the 0.11 m baseline (ORB-SLAM3's KB
      `TriangulateMatches` uses 0.9998, about 1.15 degrees / 5.5 m cap).
- [ ] Tests to add (deferred review findings): stereo observation depth and
      right index are not read back by any test (add when a consumer exists);
      the initializer path for an unmatched measurement.
- [ ] Matcher: the one-to-one drop loses a valid second candidate (ORB-SLAM3
      behaviour); consider second-chance matching.

## Map & events

- [ ] `MapPointStorage` growth (OQ 2): ids are append-only (removal only marks
      a point dead), so storage grows with every point ever created in a map;
      decide whether/when to compact (a compaction would need an id remap
      event for the replicas).
- [ ] Size event pools per event type. (OQ 14)
- [ ] `ObservationsRemoved` event (sub-project 3): observations dropped by
      culling / fusion; `KeyFrameCreated` only adds.
- [ ] Immutable shared measurement in events (`shared_ptr<const>`):
      `FrameTracked`, `KeyFrameCreated` and `InitializationMeasurement` copy
      the measurement (about 80 KB per frame).

## Threads

- [ ] Tracker measurement policy (OQ 19).
- [ ] `Tracker::Stop()` leaves still-queued measurements in the queue: they are
      processed after a restart, or destroyed with the Tracker. Decide whether
      Stop() should drain or discard them together with the measurement
      policy. (OQ 19)
- [ ] **Future (not now): mixed mono/stereo sessions.** Mono and stereo
      measurements may arrive in one session; Mapper and Tracker behaviour
      depends on the sequence (mono then mono: today's two-view init; mono
      then stereo, stereo then mono, …: to be designed). Keep both
      observation storages and per-keyframe measurement types to allow it.
- [ ] **Tracking sub-projects 3-4 (deferred; the user has their own designs for
      them, ask before designing; sub-project 2 is done, see DESIGN "Keyframes
      and Map Growth"):** (3) local BA and culling; (4) relocalization /
      reference-keyframe tracking via place recognition (BoW).
      Includes relocalization in `TrackerState::Lost` (now it behaves like
      `WaitingForMap`; the Mapper starts a new map on `TrackingLost`).
- [ ] Derive `TrackerOptions::level_scale` and `ProjectionMatcherOptions`
      `level_scale`/`max_level` from the feature extractor instead of keeping
      them in sync by hand.
- [ ] BoW-less fallback limitation: a wide-window (x4) zero-motion projection
      search replaces `TrackReferenceKeyFrame`; it recovers only small motion.
- [ ] Local-map search: add ORB-SLAM3's scale-invariance distance check (now
      only the predicted level is clamped).
- [ ] Debug `OptimizePose` costs about 0.5 s per call with about 500 stereo
      edges (Release is fine): profile.
- [ ] Stereo tracking test margin: 1.79 cm error against the 2 cm bound.
- [ ] Tracking test helpers assume 2 (mono) / 1 (stereo) keyframes per map
      (`ExpectPositionsNearTruth` in `slam_test.cpp`); the end-to-end tests
      that use it stop before the Mapper adds a Frame. Use the first keyframes
      for the alignment (as `MaxCentreError` does).
- [ ] A newer `MapCreated` while `Tracking` leaves an empty `last_frame_`
      (unreachable today); handle before multi-map / relocalization.
- [ ] Mono translation accuracy at default init (about 10 % at 1 deg parallax,
      decision T): tuning.
- [ ] LoopCloser logic (its `ProcessPending()` only acknowledges events).

## Keyframes and map growth (sub-project 2, 2026-10-06)

Spec `docs/superpowers/specs/2026-10-06-keyframes-design.md`; DESIGN
"Keyframes and Map Growth" (smoke table there).

- [ ] **Mono Frame creation stalls (smoke 2026-10-07):** after the first
      created Frame, `ref_points` stays at the reference Frame's tracked
      points with >= 3 observations (36 on corridor1): triangulated points
      have 2 observations and nothing adds a third until the next Frame, so
      `inliers < 0.9 · ref_points` never holds while tracking is healthy and
      the map dies at the 30-inlier gate instead (map 1: 129 tracked frames,
      1 Frame created after initialization). Fusion (sub-project 3) adds the
      observations ORB-SLAM3 relies on; revisit the rule if it does not.
- [ ] **Stereo loss at fast rotation (smoke 2026-10-07):** frame ~360 of
      corridor1 (mocap 120-170 deg/s): motion model 17 matches (< 20), zero
      motion 6 inliers. Needs reference-Frame tracking / relocalization
      (sub-project 4).
- [ ] **Measure triangulation matching cost; accelerate if needed.**
      Brute force per neighbour: about 25 ms per neighbour on corridor1
      (1500 features, Release); mono uses up to 30 neighbours (about 0.75 s
      per Frame once the map has them). Faiss vs DBoW2/FBoW spike for place
      recognition (sub-project 4) decides the index.
- [ ] Faiss vs DBoW2/FBoW spike for place recognition (sub-project 4).
- [ ] Interruptible BA and ORB-SLAM3's busy-Mapper branch of the Frame rule
      (sub-project 3; nothing long runs in the Mapper yet).
- [ ] Point fusion, point and Frame culling, local BA (sub-project 3). Without
      them duplicate points accumulate (stereo smoke: 67 879 points in map 2
      after 637 tracked frames).
- [ ] The post-relocalization Frame guard (sub-project 4).
- [ ] Synthetic end-to-end error bounds: `SlamTest.*MapGrowsWhileLeavingInitialView`
      allow 30 % of the path (mono) / 0.3 m (stereo); measured 20 % / 0.15 m
      at 0.5 px noise, 0 without noise. The plan's 5 % / 2 cm need local BA:
      tighten then.
- [ ] ORB `Distance` returns `uint8_t`: a 256-bit difference (all bits
      differ) wraps to 0. Pre-existing; widen the return type.
- [ ] The 80-Frame local-map cap can drop the only K1 Frame of a matched
      point, hiding it from the local points (`visible ⊇ matched` corner
      case): always include matched points in the mask.
- [ ] `Mapper::FindKeyFrame` assumes consecutive Frame ids per map (assert
      only); breaks with Frame culling (sub-project 3).
- [ ] `CovisibilityIndex` does not purge removed points; consumers filter by
      `IsAlive`. Purge with `MapPointsRemoved` (sub-project 3).
- [ ] The replica feeds a `KeyFrameCreated` Frame's own observations into
      covisibility unvalidated (out-of-range id → huge resize, duplicate →
      index disagreement); added observations are checked against
      `positions_.size()`, not `IsAlive`; a re-sent `frame_id` leaves stale
      covisibility.
- [ ] Triangulation filter tests missing: rotation histogram, one-to-one,
      `th_low`, ratio test, positive depth (no test fails if the z > 0 check
      is removed), stereo fallback with the depth on side b, the
      neighbour-skip rules (deleting either passes), at most one new point per
      new-Frame feature across neighbours; the fisheye test cannot tell the
      Ruling F band from the old one.
- [ ] Slow Mapper tests: about 5 s each in Debug (they start from a mono
      initialization); share a fixture or a pre-built map.
- [ ] Smaller review leftovers: `ShouldCreateFrame` one-sided boundary tests;
      `MeanViewingDirection` returns zero on empty / cancelling input;
      `CreateFrame` feature index check is assert-only and one feature paired
      with two points is not rejected; `LeftOf` / `TimestampOf` duplicated in
      `tracker.cpp` and `mapper.cpp`; `DrainsToLatest` does not assert that the
      newest was evaluated, the reference fallback and covisibility clear are
      untested; `PublishFrameTracked` copies instead of moving vectors; the
      viewer's local-Frame lookup is O(K·L).
- [ ] Replica / covisibility test gaps (Task 1 review): `tracker_test`
      compares only the count of `observed_points`; the atomic-apply test does
      not check updated-before-Frame order or point 3's distance; test helpers
      create featureless carrier Frames (use Frames with real features).
- [ ] Triangulation nits: the `TriangulationView` measurement pointer
      defaults to `nullptr`; `PixelChi2` is reached through `features::detail`
      (make it a public helper); add an early-out before the geometry checks;
      drop the `asin` in the epipolar test (compare sines).
- [ ] `Mapper` constructor: dead copy of the initializer options inside
      `options_` (remove).
- [ ] Map-growth test gaps (Task 5 review): the replica's point reference
      level / distance are not checked; `StereoClosePointsFromDepth` lacks
      `ExpectReplicaConsistent`; the Mapper's storage for new points is not
      checked directly.
- [ ] `Mapper::FeaturesWithPoints` counts features of dead points as taken
      (filter by `IsAlive`).
- [ ] The local-map merge test reuses a new-point start id (violates
      append-only ids inside the test); local-map tests were RED only at
      compile time.
- [ ] `SceneState` test `KeyFrameCreatedAppliesUpdatedPoints` had no RED run.
- [ ] `SlamTest.*MapGrowsWhileLeavingInitialView` do not assert that the last
      frames share no point with the first view; their 120 s waits equal the
      ctest timeout.
- [ ] ATE against mocap for the smoke runs: the apps do not write their
      trajectory; add a TUM-format trajectory dump and an alignment script.
- [ ] Final-review follow-ups (2026-10-07):
  - Comments at `tracking/local_map.h` (`ReferenceFrame`) and `tracker.h`
    say the reference Frame is chosen among K1 / local Frames; Ruling L
    chooses among all observing Frames. Fix the comments.
  - `ObjectPool::Create()` aborts when the 256 `FrameTracked` slots are
    exhausted; a Mapper blocked in long `CreateFrame` calls (30 mono
    neighbours, Debug) could reach it. Bound in-flight events or degrade
    (drop) instead of aborting.
  - No test that the replica and the Mapper agree after a map switch
    (removing either `covisibility_.Clear()` fails nothing).
  - The zero-motion fallback re-centres on the last pose with a 28 px window,
    which cannot cover 6–8° per frame; the stereo loss in fast rotation
    (corridor1 ~frame 360) needs a rotation-aware fallback or reference-Frame
    tracking (sub-project 4).

## Visualization

- [ ] Event serialization + transport (HTTP or ROS) for a remote viewer: a
      serializing async observer on the SLAM side, a receiver feeding
      `SceneState` on the viewer side.
- [ ] Separate image flow for the camera view (images/keypoints are outside the
      SLAM events).
- [ ] `MapViewer` hardening (deferred review findings, 2026-10-03/04):
      reject non-finite / extreme `fps`, and wake `Stop()` immediately (low fps
      delays it by a period); `width` / `height` / `camera_size` aren't
      validated; `Start()` thread-creation failure leaves the state stuck;
      renderer partial-creation failure cleanup; exceptions on the loop thread:
      in `Start()` mode ANY exception (incl. `std::exception`) from
      Apply/Draw/FinishFrame calls `std::terminate`; in `RunOnCurrentThread()`
      mode it propagates but queued events stay unacknowledged until the next
      `Stop()`/destructor; `RunOnCurrentThread()` consumed-stop hazard (a
      cross-thread `Stop()` before entry; currently documented only);
      a pre-set stop with `show_window` creates and immediately destroys the
      window.
- [ ] Viewer tests: window-mode paths have no automated test (consider a test
      seam that forces renderer-creation failure).
- [ ] Viewer: points are drawn in immediate mode - use a VBO for large maps.
- [ ] macOS setup: playback on a worker thread, viewer (`RunOnCurrentThread()`)
      and image window on the main thread. In that mode other threads must end
      the viewer with `RequestStop()` only; `Stop()` and the destructor run on
      the main thread after `RunOnCurrentThread()` returns.
- [ ] 2D initialization overlay in the viewer (matches between the reference and
      current frame).
- [ ] Viewer extras: covisibility edges, colouring points by number of
      observations, video recording.
- [ ] Install Pangolin 0.9 on the second development machine.
- [ ] `SceneState`: counter array sized from the last `EventType` enumerator;
      add a count sentinel / `static_assert` and bounds-check types (needed once
      events are deserialized from the network).

## Event system

- [ ] `AsyncObserver` destruction doesn't acknowledge events still queued, so
      their pool slots never return. Drain on destruction.
- [ ] **Event pool lifetime** (OQ 25): components own the pools of the events
      they publish but consumers (e.g. the `MapViewer` inbox) can hold them.
      Without care, on exception unwinding `~Slam` runs while the viewer thread
      may still be draining into its pools, and the viewer's destructor drain
      recycles into freed pools. The apps now guard against it (a guard after
      `slam` runs `slam.Stop(); viewer.Stop()` on every exit path), but the
      library still needs pools that outlive all producers and consumers.

## Review follow-ups (tracking states, 2026-10-03)

Deferred minor findings from the per-task and final reviews.

- [ ] **Before relocalization / multi-map:** the Tracker adopts a newer map's
      id but drops its `MapCreated` while `Tracking` (treated as a duplicate);
      the next `FrameTracked` would carry the new map id with the old
      frame/pose. Make the duplicate check specific to the last `MapCreated`.
- [ ] `SceneState` and `LoopCloser` don't apply the `map_id` consumer rule
      (no replicas yet) — apply it when they get replicas. DESIGN §6 "rule for
      every consumer" should exempt the Mapper (it assigns map ids).
- [ ] Docs: the `slam.h` stop-order comment overstates (the graph is
      cyclic, events stay queued in stopped inboxes); DESIGN.md Tracker section
      keeps the older loop pseudo-code beside the state machine; event pool
      size (256) is not documented.
- [ ] Mapper: `keyframes_.back()` is unguarded if a real `TryInitialize`
      returns an engaged empty result — treat it as failure; transition log
      wording ("0 keyframes" with the old map id, "1 keyframes").
- [ ] Share one `NumFeatures(TMeasurement)` helper (the "left camera for
      binocular" visit is duplicated in `tracker.cpp` (`LeftOf`) and
      `map_initializer.cpp`, in two dispatch styles).
- [ ] Document thread-usage rules: one waiter per `WakeSignal` (`notify_one`);
      never call `WorkerThread::Start()`/`Stop()` from its callback (self-join);
      the defensive join in `Start()`.
- [ ] Tests to add: inbox-first / one-measurement-per-pass ordering in the
      Tracker; consumer rule for stale/newer non-`MapCreated` events; Mapper
      acknowledging `LoopDetected` and returning false on
      an empty inbox; `Slam` forwarding `MapperOptions`; 49/50 and 99/100
      feature boundaries; e2e checks of every `FrameTracked` map id.
- [ ] Test robustness: `WaitBlocksUntilNotify` hangs rather than fails if
      `Wait` breaks (relies on ctest `--timeout`); `RunsUntilNoWorkThenSleeps`
      20 ms settle may flake on loaded CI and calls `Notify` needlessly;
      `ObservableDeliversThroughPush` doesn't assert the queue is empty;
      unseeded `Random()` in `tests/support/test_measurements.h`.
- [ ] Hygiene: include order / unused alias in `event_system_test.cpp`; missing
      `<type_traits>` / `<variant>` includes; `MapId::value` (like `FrameId`)
      has no default initializer; `tracker.h` and `full_slam_common.h` pull in
      heavy headers.

## Review follow-ups (two-view geometry, 2026-10-04)

Deferred minor findings from the per-task reviews.

- [ ] Robustness: `ransac.h` `std::clamp` is UB when `min_iterations >
      max_iterations` (compute bounds once with `std::min/max`);
      `SelectPose` lacks an `inliers.size() == correspondences.size()` guard
      and counts a NaN reprojection error (z exactly 0) as good (use
      `!(err <= max)`); estimator samples larger than `kSampleSize` are
      silently truncated (document or assert); LM rejected-trial / Jacobian
      failure on the first linearization returns `[t]xR` rather than the exact
      input (doc mismatch, harmless).
- [ ] Docs: `uniform_int_distribution` is not reproducible across
      libstdc++/libc++ (seeded results differ per standard library); Ransac
      confidence edge cases; `TwoViewCorrespondences::sigma` has no default
      initializer; homography `Refine` doc should state the H(2,2) = 1 result;
      `plane_normal` of non-selected candidates is flipped without t
      (document "only meaningful for the selected candidate"); an essential
      test comment says ~90 % expected, actual ~83 %.
- [ ] Hygiene: `M_PI` / duplicated `kRadToDeg` (use `std::numbers::pi`);
      repeated `1e-12` literal; Hartley centroid/spread duplicated in
      `Normalize`; two SVDs per homography hypothesis;
      `Transform(col expression)` is ambiguous between `TPoint3DF` /
      `TPoints3DF` overloads (callers must wrap).
- [ ] Tests to add: scene helper checks (planar x2 consistency, outlier range,
      noisy-scene noise level); unseeded `Random()`/`UnitRandom()` in
      `rigid_transformation_test`, point-level inverse round trip; infinity
      and zero-epipolar-line scores pinned exactly (5.991);
      `TrueModelOutscoresPerturbed` with the true H; Ransac exact
      `min_iterations` early stop, `max_iterations` cap, no-positive-score path
      (the 50 % test takes ~2.8 s in Debug); decomposition with negated H and
      several poses, near-degenerate singular-value band; rejection tests
      (`NarrowFrontalPlanar`, `PureRotation`, `TinyBaseline`) should assert the
      reason, and the "H because E failed" branch is untested; wide-FOV scenes
      still draw outliers from ±0.6 (scale with `half_fov_tan`) and the
      refined >= unrefined check holds only at the chosen seed.

## Review follow-ups (monocular initialization, 2026-10-04)

- [ ] Tests: Debug runtime of the mono-init tests (~0.5 s per two-view
      attempt; `PureRotationNeverInitializes` ~18 s) — use cheaper
      `TwoViewOptions` for tests or fewer frames.
- [ ] Matcher: per-candidate descriptor copy (matters for SIFT cost).

## Environment

- [ ] g2o build under a clang/libc++ profile and an Android NDK profile (only gcc 13
      is checked today). Expose more g2o libraries in the recipe only when needed.
- [ ] Local and global bundle adjustment reusing `optimization::BundleAdjust`
      (Mapper keyframe insertion / loop closing; needs a windowed `BaProblem`
      builder with fixed keyframes).
- [ ] BundleAdjust hardening: the all-edges-deactivated path relies on g2o
      returning -1 plus stderr noise; validate `huber_chi2` (> 0, finite);
      handle zero-depth initial points explicitly.
- [ ] g2o recipe `package()`: upstream's CMake config files are installed into the
      package (`lib/cmake`); remove them (`rmdir`) since the recipe declares the
      components itself.
- [ ] Second dev machine: build OpenCV 5.x from source into `/usr/local` with
      `WITH_GTK=ON` (Conan no longer provides OpenCV).
- [ ] Recurring: syntax-check all sources with clang-18 + libc++ (clean on
      2026-10-06, 92 sources + 82 public headers); later add an Android NDK
      arm64 cross-build.
