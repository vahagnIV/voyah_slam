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
      frames) and the mono initial map size (62-109 points; maps last ~4
      frames). Mono misses the ≤ 5 re-initializations criterion (18).
- [ ] Per-model camera classes (approach B, ORB-SLAM3 `GeometricCamera`), only
      if ever needed (e.g. several lens models in one rig).
- [ ] Behind-camera edges: g2o edges call `Camera::Project` before the depth
      check, violating its `z > 0` precondition (finite mirrored value; z == 0
      gives NaN). Check depth first.
- [ ] `Camera::Project` heap-allocates (dynamic 3×1 for `Distort`) per call in
      the hot path (g2o edges, matchers); add an allocation-free path.
      `Unproject` also inverts K per call.
- [ ] Deduplicate the TUM-VI Kannala-Brandt test fixture (copied in about 8
      test files) into `tests/support`.
- [ ] Validate K beyond finiteness (fx, fy > 0) in `OptimizePose`,
      `BundleAdjust` and `Camera`.
- [ ] Test that `BundleAdjust` uses each keyframe's own camera (keyframes with
      different focal lengths); today's BA tests share one K.
- [ ] Fisheye test pinning `MapInitializer`'s grid and `last_matched_` on
      `distorted_points()` (map_initializer.cpp); pinhole test helpers build
      the grid on `points()`.

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
- [ ] `NewKeyFrameCandidate`: add the tracked `(map_point_id, feature_idx)`
      observations.
- [ ] Mapper: `PublishKeyFrameCreated` scans all observations per keyframe
      (O(obs x keyframes)) — add a per-frame observation index before
      keyframes are created during Mapping.
- [ ] Mapper: `PublishMapPointsAdded` publishes `[0, num_ids())` of a fresh
      map; points added later during Mapping need a `start_id = num_ids()`
      before the additions (ids are append-only, so the range is contiguous).

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
- [ ] **Tracking sub-projects 2-4 (deferred; the user has their own designs for
      them, ask before designing):** (2) keyframe decision and map growth
      (`NewKeyFrameCandidate`, Mapper keyframe creation); (3) local BA and
      culling; (4) relocalization / reference-keyframe tracking via place
      recognition (BoW).
      Includes: keyframe decision (Tracker publishes `NewKeyFrameCandidate`,
      Mapper handling; now only acknowledged) and relocalization in
      `TrackerState::Lost` (now it behaves like `WaitingForMap`; the Mapper
      starts a new map on `TrackingLost`).
- [ ] `SearchFromLocalMap` scans the whole replica (allocates per call):
      covisibility-based local map with sub-project 2.
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
- [ ] Tracking test helpers assume 2 (mono) / 1 (stereo) keyframes per map;
      revisit with sub-project 2.
- [ ] A newer `MapCreated` while `Tracking` leaves an empty `last_frame_`
      (unreachable today); handle before multi-map / relocalization.
- [ ] Mono translation accuracy at default init (about 10 % at 1 deg parallax,
      decision T): tuning.
- [ ] LoopCloser logic (its `ProcessPending()` only acknowledges events).

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
      the next `PositionUpdated` would carry the new map id with the old
      frame/pose. Make the duplicate check specific to the last `MapCreated`.
- [ ] `SceneState` and `LoopCloser` don't apply the `map_id` consumer rule
      (no replicas yet) — apply it when they get replicas. DESIGN §6 "rule for
      every consumer" should exempt the Mapper (it assigns map ids).
- [ ] Docs: `PositionUpdated.frame_id` is the `MapCreated` keyframe id, not the
      current frame's; the `slam.h` stop-order comment overstates (the graph is
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
      acknowledging `NewKeyFrameCandidate`/`LoopDetected` and returning false on
      an empty inbox; `Slam` forwarding `MapperOptions`; 49/50 and 99/100
      feature boundaries; e2e checks of every `PositionUpdated` map id.
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
