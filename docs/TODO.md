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
      LO). Bundle adjustment after initialization provides further accuracy.
- [ ] Optional: run the H and E RANSACs in parallel.

## Initialization (monocular, 2026-10-04)

- [ ] Tuning on TUM-VI 512 (`dataset-corridor1_512_16`) after the Pangolin
      viewer, plus a per-frame "dropped by the 85° rule" statistic.
- [ ] Bundle adjustment after initialization (accuracy; see the refinement
      lock-in item under Geometry).
- [ ] `OrbFeatureExtractor::Distance` returns `uint8_t`, so a 256-bit
      difference wraps to 0 — widen the return type.

Smoke run note (2026-10-04, `dataset-corridor1_1024_16`, ORB, Release,
`--rate=0 --max-frames=200`): mono initialized at about frame 121
(`Initializing -> Mapping (map 1, 2 keyframes, 73 points)`; 73 points is low,
tuning deferred). Binocular (`--max-frames=20`) initialized on the first frame
(`Initializing -> Mapping (map 1, 1 keyframes, 355 points)`); median point
depth 1.96 m (quartiles 1.50 / 2.13 m, range 0.25-9.50 m). The viewer run
(`--viewer`) completed without errors; no screenshot (X_GetImage failed on the
root window).

## Stereo (2026-10-04)

- [ ] Epipolar-guided stereo search (optimization of the brute-force matcher).
- [ ] Stereo-aware tracking: use depths and right observations in the
      `PoseEstimator`.
- [ ] Tuning on TUM-VI 512, including the ~17 m stereo depth cap implied by
      `max_parallax_cos` 0.99998 at the 0.11 m baseline.
- [ ] Tests to add (deferred review findings): stereo observation depth and
      right index are not read back by any test (add when a consumer exists);
      the initializer path for an unmatched measurement; Tracking-state stereo
      matching before `Estimate()`.
- [ ] Matcher: the one-to-one drop loses a valid second candidate (ORB-SLAM3
      behaviour); consider second-chance matching.

## Map & events

- [ ] `MapPointStorage`: stop reusing freed slots (ids are never reused);
      decide storage-growth strategy. (OQ 2)
- [ ] Size event pools per event type. (OQ 14)
- [ ] `NewKeyFrameCandidate`: add the tracked `(map_point_id, feature_idx)`
      observations.
- [ ] Mapper: `PublishKeyFrameCreated` scans all observations per keyframe
      (O(obs x keyframes)) — add a per-frame observation index before
      keyframes are created during Mapping.
- [ ] Mapper: `PublishMapPointsAdded` assumes point ids are `[0, size())`;
      breaks once points are removed (`MapPointStorage` still reuses freed
      slots; see the item above, OQ 2).

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
- [ ] Real `PoseEstimator` (track with motion model / reference keyframe /
      local map; motion-only BA) — geometry spec; replaces the stub in place.
- [ ] Apply map diffs to the Tracker replica (currently a no-op).
- [ ] Keyframe decision: Tracker publishes `NewKeyFrameCandidate`; Mapper
      handling (currently only acknowledged).
- [ ] Relocalization in `TrackerState::Lost` (now it behaves like
      `WaitingForMap`; the Mapper starts a new map on `TrackingLost`).
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
      binocular" visit is duplicated in `pose_estimator.cpp` and
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

- [ ] Second dev machine: build OpenCV 5.x from source into `/usr/local` with
      `WITH_GTK=ON` (Conan no longer provides OpenCV).
- [ ] Recurring: syntax-check all sources with clang-18 + libc++ (clean on
      2026-10-04, 81 sources); later add an Android NDK arm64 cross-build.
