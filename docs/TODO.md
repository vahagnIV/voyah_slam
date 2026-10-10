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
- [ ] `check` / `check_all` build only test binaries, so an app-only build break
      (examples) is not caught before a merge; add the apps as dependencies of
      `check_all` or build `all` before merging (restructure final review,
      2026-10-07). The "Library layout" block sits under DESIGN §7 "Coding
      Conventions"; give it its own section.
- [ ] Map-quality final-review follow-ups (2026-10-07):
  - Long runs scale with point ids ever issued, not live points: fusion and
    culling make ids ~6× the live points, the Tracker scans all ids per frame.
    Estimate for 5000 stereo frames: ~140 MB and ~540k ids scanned per frame
    (see OQ 2); compact or index live points.
  - BA outlier removal keeps a point with 1 observation (removes only at 0);
    Frame culling removes below 2. Align (ORB-SLAM3 removes < 2 in culling only).
  - An urgent event during T1 stops BA before its first iteration, yet T2 still
    republishes every unchanged pose and point; skip unchanged entries.
  - Test the Mapper's fallback when `reference_frame_id` names a culled Frame.
  - After a merge, `last_frame_` matches can hold one point on two features
    (ORB-SLAM3 too); dedupe.
  - Stale text: older DESIGN sections (initialization publication order,
    "Keyframes and Map Growth" fast pickup / viewer lines) and some TODO items
    still name `KeyFrameCreated` / `MapCreated` as current.
  - `tools/ate.py` ignores the camera–body lever arm (few cm).

## ORB extractor (OQ 21)

- [ ] Own FAST-9-16, run once per pyramid level; assign corners to cells
      (≥ 20 if any, else ≥ 7). Removes OpenCV from detection and the blind
      strips at cell borders.
- [ ] Preallocate blur and pyramid buffers (no per-frame image allocations).
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
      test files) into `libs/test_support/support`.
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
- [ ] Tests under `libs/common/tests/geometry` for the moved distortion models still use
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
- [ ] Immutable shared measurement in events (`shared_ptr<const>`):
      `FrameTracked`, `MapTransaction` and `InitializationMeasurement` copy
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
- [ ] Derive `TrackerOptions::level_scale` and `ProjectionMatcherOptions`
      `level_scale`/`max_level` from the feature extractor instead of keeping
      them in sync by hand.
- [ ] BoW-less fallback limitation: without a vocabulary a wide-window (x4)
      zero-motion projection search is the only fallback (no
      `TrackReferenceKeyFrame`, no relocalization); it recovers only small
      motion.
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

## Keyframes and map growth (sub-project 2, 2026-10-06)

Spec `docs/superpowers/specs/2026-10-06-keyframes-design.md`; DESIGN
"Keyframes and Map Growth" (smoke table there).

- [ ] **Stereo loss at fast rotation (smoke 2026-10-07):** frame ~360 of
      corridor1 (mocap 120-170 deg/s): motion model 17 matches (< 20), zero
      motion 6 inliers. Since 2026-10-08, with a vocabulary, reference-Frame
      tracking runs next and a loss relocalizes into the same map; without a
      vocabulary the rotation is still uncovered by the zero-motion fallback.
- [ ] Faiss vs DBoW2/FBoW spike for place recognition (sub-project 4).
      Roadmap step 2 ported DBoW2 (see "Bag of words" below); revisit only
      if its recall or speed falls short in relocalization / loop detection.
- [ ] ORB-SLAM3's busy-Mapper branch of the Frame rule (interruptible local BA
      is done; a Mapper pass now takes about 250 ms, so drain-to-latest skips
      about 40 % of the `FrameTracked` on corridor1).
- [ ] The 80-Frame local-map cap can drop the only K1 Frame of a matched
      point, hiding it from the local points (`visible ⊇ matched` corner
      case): always include matched points in the mask.
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
    (corridor1 ~frame 360) needs a rotation-aware fallback. Reference-Frame
    tracking exists since 2026-10-08 but only with a vocabulary.

## Bag of words (roadmap step 2, 2026-10-07)

Spec `docs/superpowers/specs/2026-10-07-bow-design.md`; DESIGN "Bag of Words"
(smoke table there). Triangulation matching cost is solved there
(`SearchByBoW`: 26–28× faster per Frame mono, 39× stereo).

- [ ] **`SearchForTriangulation` ratio test:** it applies a 0.6
      best/second-best ratio test that ORB-SLAM3's `SearchForTriangulation`
      does not. Predates the BoW branch; with BoW it often passes trivially
      because only one candidate is left. (The stereo drift it was to be
      evaluated with is fixed: the motion-model gate, 2026-10-08.)
- [ ] **Extra stereo BoW run lost tracking at frame 325** (fast rotation, the
      known weakness below; runs without the vocabulary dip to 38 / 52 inliers
      at frames 325 / 355). The 2026-10-07 spike traced it to 10–29-inlier
      motion-model poses; the motion-model gate (2026-10-08) removed it in
      every run since, and a loss there would now relocalize. Close after a
      few more stereo runs without a loss.

- [ ] **Vocabulary training** (out of scope of step 2): our own k-majority
      tree builder, so we do not depend on ORB-SLAM3's `ORBvoc.txt`, and a
      vocabulary for other extractors (SIFT has none).
- [ ] **`ORBvoc.txt` licence** (deferred, user decision): the file is used
      as is on the user's PC and never committed. Settle its terms before any
      distribution; DBoW2's own code is BSD (keep its notice with the port).
- [ ] **Phantom node vs ORB-SLAM3** (ruling D): DBoW2 turns the empty read
      after `ORBvoc.txt`'s trailing newline into a phantom leaf (971 815 words
      vs our 971 814; uninitialised values, UB). A feature DBoW2 drops at that
      node (word weight 0) gets a real sibling's word with us. Ours is the
      correct tree; remember it when comparing word ids or scores with
      ORB-SLAM3 output.
- [ ] ORB `Distance` returns `uint8_t`: a 256-bit difference (all bits
      differ) wraps to 0, so a vocabulary descent or a matcher could take the
      farthest candidate for the closest. Not seen on the DBoW2 fixture (no
      child distance of 256). Widen the return type.
- [ ] Mono smoke with BoW (2026-10-07): triangulation finds fewer matches per
      pair at the start of a map (291 vs 365 new points for the first Frame),
      so `ref_points` grows slower and both runs created no Frame for 27
      frames (dataset 136–163; at most 11 without BoW). Tracking was unaffected
      (inliers 410–450). Watch it; ORB-SLAM3 also triangulates via BoW.
- [ ] Deferred review minors:
  - `LoadBinary` reads through `istreambuf_iterator` (byte-wise; 0.13 s for
    ORBvoc, fine); a bulk read or mmap if it ever matters.
  - No NaN / inf check on float descriptors in the loaders (only ORB has a
    vocabulary today).
  - Vocabulary tests: none for a leaf with depth between `L − levels_up` and
    `L`; two tests do not assert `offsets`.
  - `RotatedOffsetsAreRounded`'s 512-point loop is partly self-referential;
    only the (5, 9) case pins the rounding.
  - The DBoW2 reference programs are described in
    `tools/dbow2_reference/README.md` only, not committed; the fixture is
    144 KB (planned about 100 KB).
  - `SearchByBoW` tests: none pins the ratio test among shared-node
    candidates or the fallback with only one FeatureVector pointer; no debug
    assert that FeatureVector indices are below the feature counts; the
    "subset of brute force" test name overstates (ruling F: holds for its
    scene only).
  - Mapper BoW tests: no stereo BoW test; the FeatureVector and `levels_up`
    are asserted only indirectly; `MapperBowRun` is a hand-driven gtest
    fixture subclass.

## Loop closing (roadmap step 4a+4b, 2026-10-09)

Spec `docs/superpowers/specs/2026-10-09-loop-closing-design.md`; DESIGN
"Loop Closing" (smoke table there).

- [ ] **Loop closing is opt-in and off by default** (user decision
      2026-10-10) until the same-heading look-alikes are rejected: see "TUM-VI
      tuning (roadmap step 6)" below.
- [ ] **The normal gate blocks true loops once the map is rotated.** It
      compares normals in map coordinates. After a same-heading false loop
      is corrected, the two ends of the map are rotated against each other,
      and the gate then rejects the true return: stereo run 1 of the
      2026-10-09 smoke rejected 178 return candidates 0.1–1.0 m apart at
      0.54–1.00, and mono run 1 rejected 70. Before such a correction it
      rejected no true loop in any run. Large mono rotation drift would do
      the same. Comparing the normals through the hypothesized Sim3
      measures something else, so this is not a fix in itself; decide
      together with the same-heading item above.
- [ ] **Re-run ThreadSanitizer after every loop-closing or global-BA
      change.** Last run 2026-10-10, global BA (Tasks 2 and 4):
      `voyah_slam_tests` (217) and `voyah_integration_tests` (18) with no
      warnings.
- [ ] **Mono scale after corrections:** mono loops carry extreme scales after
      wrong corrections (0.314–1.774 in the 2026-10-09 smoke). Revisit once
      wrong loops are rejected.
- [ ] **Single loop connection:** in corridor maps most corrections had only
      the query–match loop connection (other new links share < 100 points,
      ORB's `minFeat`), so the essential graph pulls the loop through one
      edge. ORB-SLAM3 behaves the same and relies on global BA (4c, done
      2026-10-10: DESIGN "Global BA").
- [ ] **Map merging** (roadmap step 5): detection finds merges in inactive
      maps (Task 4, 2026-10-10) and the Mapper merges them (Task 5,
      2026-10-11); the Tracker does not follow a merge yet (Task 6), and the
      integration tests and smoke runs are Task 7.
- [ ] **Mono merge in the Mapper is untested (step 5, Task 7):** the
      `MapperMergeTest` scene is stereo; the mono path (free-scale similarity,
      scale folded by `CorrectWindow`, the essential graph of the rest with
      free scales, which ORB-SLAM3 skips for mono) is only reached by the
      `SlamTest.MonoMergesMaps` integration test.
- [ ] **Merge weld cost (step 5):** `WeldWindows` optimizes every point the
      two windows observe with up to 50 Frames, not interruptible (ORB passes
      a stop flag that is never set); measure on corridor1 in Task 7.
- [ ] **Hypothesis-independent check for merges (step 6):** the normal gate
      is not applied to merges (ruling 2026-10-10: the two maps' worlds are
      related only by the merge hypothesis), so false-merge protection is
      weaker than for loops. Candidates: a gravity / up-direction
      consistency check (needs an IMU or a level prior), or verify-before-
      publish (e.g. track the query's next Frames in the match map before
      the merge is applied).
- [ ] **Thin mono merge scale margin (step 6):** `LoopCloserMergeTest`
      measured up to 1.34 % scale error against the 2 % bound (about 1.5x
      margin; the loop tests have about 3x).
- [ ] **Merge detection test gap (step 6):** a merge region confirmed by a query is
      not searched again in that query (ORB `bMergeDetectedInKF`); no test
      tells this from a re-search, since in the synthetic scene both give
      the same confirmations (mutation M7 of Task 4 survives).
- [ ] **Scale without IMU:** mono scale is corrected only at a loop (the
      essential graph spreads the loop's Sim3 scale over the graph, start
      included); between loops it drifts freely, and nothing makes the map
      metric. Visual-inertial (ORB-SLAM3's IMU initialization) is out of
      scope so far.
- [ ] **Point reference distances after a mono scale correction:** points
      moved by the correction keep their old min / max invariance distances
      (ORB-SLAM3 `UpdateNormalAndDepth` recomputes them), so scale-gated
      projections reject them until they are re-observed (Task 6 ruling).
- [ ] **The LoopCloser is not wired to `TrackingLost`** in `Slam` (it handles
      a final one if wired); a lost map's late loops are rejected by the
      Mapper's map-id check instead, after wasted detection work (Task 5
      ruling). Duplicate loops are not suppressed in the LoopCloser either;
      the Mapper ignores them. Since 2026-10-10 a repeat (a loop published
      at an epoch that already had one) has no priority, so it no longer
      stops a running local BA; a genuinely new loop found before the
      previous correction arrives (or after the Mapper rejected the previous
      one) also waits for the BA.
- [ ] **Dropped loop queries** (2026-10-10, final-review I1): beyond
      `max_query_backlog` (8) the oldest waiting queries are dropped, so a
      long-lagging LoopCloser misses detections (counted in
      `QueryStats::dropped`, logged once per backlog episode, summarised by
      `Stop()`). Per-query detection on the synthetic loop scenes: 4.5–9.3 ms
      mean, 26 ms max (Debug tree, `-O2`); corridor1 Release (2026-10-10):
      mono 55 / 133 ms (mean / max, under load), stereo 43.5 / 96.5 ms,
      none dropped. Keep reading `Stop()`'s line in the smoke runs.
- [ ] **ORB's match-window widening for sparse maps** (fewer than 10
      covisible Frames, `DetectAndReffineSim3FromLastKF`) is skipped: slightly
      fewer confirmations on sparse maps (Task 5 ruling).
- [ ] **Essential-graph weighting:** edges are identity-weighted Sim3s; in the
      Task 6 ring test the middle Frames' rotation error grew from 2.0° to
      3.2° under the correction. Revisit (weights by shared points, or
      ORB-SLAM3's information) if corridor or other loops show it.
- [ ] **Spanning-tree deviations from ORB-SLAM3** (Task 1 ruling B): a new
      Frame sharing no points takes the previous newest Frame as parent (ORB
      leaves none); re-parenting on a cull uses live shared-point counts with
      lower-id ties (ORB: cached weights >= 15).
- [ ] **Graph checks:** `CheckGraph` checks no acyclicity or single root; the
      replica's `SetParent` has no cycle guard; the initial-map
      `ConnectToParent` asserts the Frame is `keyframes_.back()` (true for at
      most two initial Frames).
- [ ] **`SearchBySim3Projection` / `SearchPointsBySim3` descriptors:** they use
      the observing feature's descriptor and level, not the map point's
      distinctive descriptor (the interface has none); lower projection
      counts than ORB-SLAM3 are possible (Task 3 ruling).
- [ ] **Matched pairs drop features without points** (`LoopDetected`
      `matched_points`): fewer pairs to fuse than ORB-SLAM3 (Task 5 M6).
- [ ] **Mono Tracker after a scale correction:** `last_frame_` follows its
      reference Frame, but its relative translation and the velocity stay in
      the old scale for one frame (Task 7 ruling).
- [ ] **The replica applies `map_epoch` unconditionally:** a stale epoch-0
      transaction after a correction would regress it; guard with max.
- [ ] **Deferred test gaps:** `LoopDetected` interrupting a running local BA
      (and a same-batch drain not stopping an earlier BA); the essential-graph
      failure path (publishes the rigid correction and the epoch);
      `IsValid` rejections and a newer map id without `new_map` in the
      LoopCloser; `CorrectionWhileRecentlyLost` passes without the Task 7
      code (pins behaviour only); `AnchorsBeforeCorrection` has no null
      check; `SolveSim3Ransac` does not guard a non-positive or infinite
      sigma; the `OptimizeSim3` finite-difference test uses a hand-written
      `exp(δ)∘S`, its FixScale test starts only at scale 1;
      `SearchBySim3Projection`'s depth-range and behind-camera gates are
      untested, and its radius is pinned only at level 0 and in x;
      `OptimizeSim3`'s `RejectsOutliers` uses only gross outliers;
      `SolveSim3Ransac`'s mirror-degenerate scenes peak at 11 of 20 inliers
      (Task 2); `OptimizeEssentialGraph` has no fix-scale-off test with a
      non-unit corrected loop connection, its end-to-start bar is easy (the
      RMS and step ratio carry it), and its 8 tests take about 2.4 s in
      Debug; the covisible-confirmation path of the normal gate has no test
      of its own (the candidate and refinement paths do).
- [ ] **`friend struct MapperTestAccess`** (`mapper.h`, Task 1): the first
      friend test seam in the codebase, carried to Task 6 and still there;
      replace it with the real `LoopDetected` path in the tests that use it.
- [ ] **`SlamTest.MonoClosesLoop` costs 14–25 s** (three seeds, each with and
      without loop closing, lock-step): the longest test in `check_all`.
      Natural mono drift is a random walk, so the scale part needs a seed
      range; a scene with a systematic mono scale drift would allow one seed.

## Global BA (roadmap step 4c, 2026-10-10)

Spec `docs/superpowers/specs/2026-10-10-global-ba-design.md`; DESIGN "Global
BA" (smoke table there).

- [ ] **Late joining:** a component that starts mid-run (a GlobalBA or
      LoopCloser switched on later, or a remote one) needs a full-map
      snapshot to build its replica. Today components exist from the start
      or not at all (user decision).
- [ ] **Serialization and a gRPC transport** for pluggable components: the
      events are plain data (`GlobalBAFinished`, `LoopDetected`,
      `MapTransaction`) but nothing serializes them yet.
- [ ] **Memory, about 1 MB per Frame:** stereo corridor1 peaked at 1.7 GB with
      1780 Frames and 113k points. The 2026-10-10 global-BA smoke measured
      2.28 / 2.33 GB without global BA (1649 / 1709 Frames, about 101k / 106k
      points) and **4.23 GB with it** (1758 Frames, 108k points); mono 1.51 /
      1.47 GB without and 2.24 / 2.38 GB with. Global BA adds another full
      replica plus the g2o problem of a full BA. Each replica holds every
      Frame's measurement; find what dominates (measurements, BoW,
      per-replica copies, the BA problem) before trimming. Since 2026-10-10
      (map merging Task 1) the measurements are shared, not copied per
      replica and event: the same run (default flags plus `--vocabulary`)
      peaked at **1.21 GB** (1794 Frames, 114k points; 1.7 GB before).
- [ ] **Global BA iteration or size cap:** on corridor1 (2026-10-10 smoke)
      a run took 3.3–28.2 s in mono and up to 82.5 / 85.2 s in stereo
      (1465 / 1654 Frames, 92k / 103k points), at 10 iterations; in mono
      run 2, 3 of 4 runs were interrupted by the next correction after
      15–35 s, so their work was lost. An interruption lands only at the next
      LM iteration (up to about 6 s on the largest mono map).
      `Slam::Stop()` waits for the current LM iteration too, about 6-8 s on a
      full corridor1 stereo map. ORB-SLAM3 has
      no cap either. Options: fewer iterations on large maps, a time budget,
      or polling inside an iteration.
- [ ] **More maps with global BA on corridor1** (2026-10-10 smoke): both mono
      runs and stereo run 2 with global BA lost map 1 after a false-loop
      correction (2, 2 and 4 maps), while the four essential-graph-only runs
      kept one map. No loss was caused by a merge itself; with two runs per
      mode the effect is not separable from run-to-run loop variance.
      Re-measure once look-alikes are rejected (step 6), with more runs.
- [ ] **A dropped result is not retried:** when the `GlobalBAFinished` pool
      (2) is exhausted, the map keeps the essential-graph correction until
      the next loop (counted in `num_results_dropped`). ORB-SLAM3 has no pool.
- [ ] **The snapshot contract is enforced by documentation:**
      `ChangesDuringRunDoNotAffectResult` catches a `RunGlobalBa` that reads
      the replica after its first poll only when the data it reads changed.
- [ ] **Integration coverage of the merge's propagation:** the lock-step
      harness waits for each correction's merge before feeding the next
      frame, so no Frame or point is created during a run and the
      propagation branch (Frames and points outside the result) is exercised
      only by the Mapper unit tests; "no tracking loss across a merge" there
      means the Tracker follows the new epoch on the next frame. A dropped
      result makes the harness wait 60 s once per run before it fails.
- [ ] **Deferred test gaps:** `RunGlobalBa` returning nullopt on a
      non-finite result; `robust = false` keeping outliers; a single
      intrinsics in the problem tests (no per-Frame camera, no mixed mono and
      stereo); the pose of a Frame without observations; the gauge when the
      oldest surviving Frame has no observations (nothing fixed in practice,
      as in ORB-SLAM3); `SkipsCulledAndFused` does not tell fused from culled
      points; the merge's uncorrected-subtree branch (neither the root nor an
      ancestor in the result) is untested.

## TUM-VI tuning (roadmap step 6)

- [ ] **First: reject same-heading look-alikes in loop closing.** On
      corridor1, a regular corridor pattern repeats 7–30 m apart with the
      same heading. Examples:
      - frames 2215–2236 against 692–807 (14.9 m apart);
      - 3210→792 (22 m);
      - 4666→4029 (22 m);
      - 5017→2950 (30 m).

      The pairs' normals agree, so the normal gate passes them. ORB-SLAM3's
      visual gates pass them as well: the false-loop diagnosis,
      `.superpowers/sdd/2026-10-09-loop-closing/false-loop-diagnosis.md`,
      found this inherent to those gates, not a fidelity bug. In the
      fix-round-1 smoke, every loop-closing run corrected at least one. The
      next local BA then started at 10⁷–10⁸ with 25–60 % outliers, and the
      ATE was worse than without loop closing (mono 0.679 / 0.669 m against
      0.341 / 0.500 m; stereo 1.295 / 18.16 m against 0.098 / 0.138 m).
      This is why loop closing is opt-in (2026-10-10).

      Candidate fixes, to compare on corridor1 and decide with the user:
      1. **A drift-bound gate:** reject a loop whose query and match region
         lie farther apart in the map than the drift could explain since the
         match, for example a bound per metre travelled. It is safe in
         stereo while drift ≪ 15 m; mono needs a scale-aware bound.
      2. **Verify before publishing:** check the reprojection error of the
         fused points after the essential graph, or the next local BA's
         initial cost, and drop a correction that breaks the map. This was
         rejected for the first round, on 2026-10-09.
      3. **Both together.**

      The normal gate's map-frame weakness (see "Loop closing") disappears
      once no false correction gets through, so retest the gate after the fix.

## Relocalization (roadmap step 3, 2026-10-08)

Spec `docs/superpowers/specs/2026-10-07-relocalization-design.md`; DESIGN
"Relocalization and Reference-Frame Tracking" (smoke table there).

- [ ] **Relocalization into other maps** (roadmap step 5, multi-map Atlas):
      today `RecentlyLost` searches only the current map, and `Lost` still
      clears it.
- [ ] **MLPnP** (Urban et al. 2016, ORB-SLAM3's PnP) as an alternative to
      Lambda Twist P3P + `OptimizePose`: it uses the bearing covariances and
      needs no minimal-solver disambiguation. Compare recall and time on
      blackout runs.
- [ ] **Cap the relocalization candidates:** every candidate group at
      ≥ 0.75 · best is kept, so one measurement costs up to C × 60
      verifications (C candidates × 60 rounds; each up to 3 `OptimizePose`
      and 2 `SearchFromFrame`, about 1–4 ms in Release). Cap C (for example
      the top 10 groups) or bound the time per measurement; check recall on
      real data.
- [ ] **`FrameDatabase` duplicates each Frame's `BowVector`**
      (`FrameDatabase::frames_` and `ReplicaKeyFrame::bow`): about 22 KB per
      Frame (1400 words × 16 B). Bounded by the live Frames, not a leak.
      Store only the word ids, or let the replica own the single copy.
      `Remove` is O(words × posting length), O(W · F) worst case.
- [ ] **RANSAC is more permissive than ORB-SLAM3's effective settings:**
      ORB asks max(10, 0.5 · N) inliers and stops after about 35 iterations
      (ε 0.5); ours 10 inliers and up to 300. The 50-inlier verification
      guards it; measure false hypotheses and time on harder data.
- [ ] **Near-collinear PnP samples:** only exactly collinear points are
      rejected; with 1 mm–1 cm of collinearity and 1 px noise RANSAC can
      report success with a wrong pose (mitigated by the 50-inlier
      verification). Add a conditioning test on the sample.
- [ ] **Culled reference Frame:** reference-Frame tracking just fails when
      the Mapper culled the reference Frame; ORB-SLAM3 follows the parent.
      Pick the culled Frame's best covisible survivor.
- [ ] **Backwards timestamps** never time out `RecentlyLost` (the elapsed
      time is negative). Treat a backwards jump as a timeout or a reset.
- [ ] **`SearchFromFrame` has no scale-distance gate** (ORB-SLAM3's min /
      max invariance distance check in `SearchByProjection(Frame, KeyFrame)`);
      it relies on the level window only. Same gap as the local-map search
      item under "Threads".
- [ ] **`Slam` member order:** the Tracker copies the vocabulary pointer
      before the Mapper's constructor moves it, so correctness depends on
      `tracker_` being declared before `mapper_`. Pass by copy to both or add
      a comment / static check.
- [ ] **Time per relocalization attempt** on corridor1 is in the DESIGN
      smoke table; a candidate stuck at 10–49 inliers is re-verified every
      round until 300 iterations (ORB-like), which costs time per failed
      measurement while `RecentlyLost`.
- [ ] **`SlamTest.StereoRelocalizesAfterBlackout` is load-sensitive:** under
      12 parallel copies its centre error exceeds the 1.5 × 0.0217 m bound in
      about 1 run in 20 (0.036–0.058 m), before and after ruling J alike
      (3 / 60 and 4 / 60); it passes alone and in `check_all`. The threaded
      Mapper's pace changes the map. Widen the bound or make the run
      deterministic (`RunSetup::lock_step`, used by the loop tests, takes
      the Tracker and Mapper of a run out of thread timing).
- [ ] **Post-relocalization guard window** (ruling J) is 1 s of measurement
      timestamps and the map-size condition a fixed 20 Frames; ORB-SLAM3
      uses `mMaxFrames` = fps frames for both. Derive both from the frame
      rate if a dataset's rate differs much from 20 Hz.
- [ ] **Synthetic maps make few Frames** (3–5 over 60 frames): the
      integration blackout tests and the relocalization unit tests set
      `min_map_frames_to_relocalize` 0, so the default 10-Frame rule is only
      unit-tested (`SmallMapGoesStraightToLostByDefault`).
- [ ] Deferred review minors:
  - Motion-model gate test forces 100000 (no 10–29 band / exactly-30 case);
    the "zero motion:" assertion is not discriminating; the gate test reads
    debug-log text through a spdlog sink (the `stage` field can replace it).
  - No stereo-transaction BoW test; no test that the Tracker stores the
    vocabulary; no test that a removed Frame drops its BoW.
  - `FrameDatabase::Query` compares a float threshold (exact for 0.8 with
    integer counts).
  - The sorted-node merge loop of `SearchByBoW` duplicates
    `triangulation.h` (extract a shared walker); no test for the `nn_ratio`
    boundary or `histogram_bins`; a stale test comment ("1 vs 2").
  - PnP: no test catches a broken depth ≤ 0 rule; `ExpectCleanFailure` does
    not check the cleared mask / identity pose; no adaptive-stop or
    determinism test (determinism is per standard library); the 4th-point
    selection lowers per-iteration success against the w³ formula; R is not
    re-orthonormalized and the lambdas not re-checked after Gauss-Newton; the fisheye margin is thin (9.78e-6 against 1e-5).
  - No test for a `new_map` transaction arriving while `RecentlyLost`, nor
    for old-map transactions arriving while `Lost` after a final loss (both
    correct by reading and unreachable today; re-check with multi-map).
  - `RecentlyLost` is unit-tested only with threshold 0; the one-node test
    vocabulary does not exercise `levels_up` 4.
  - `LeftOf` is copied a third time (`relocalizer.cpp`); `OptimizeMatchedPose`
    / `PoseRefinement` live in `relocalizer.h`; `Relocalize` is about 135
    lines.

## Map quality (sub-project 3, 2026-10-07)

Spec `docs/superpowers/specs/2026-10-07-map-quality-design.md`; DESIGN "Map
Quality" (smoke table there).

- [ ] **Tuning** for TUM-VI (roadmap step 6): every map-quality threshold is an
      ORB-SLAM3 default; nothing was tuned.
- [ ] **Two σ sources (ruling K):** local BA takes `sigma_px` / `level_scale`
      from `MapperOptions::fuse`, the Tracker from `TrackerOptions`, and since
      2026-10-10 global BA from `GlobalBaOptions` (same defaults, a third
      source; global-BA pre-flight ruling). Derive all from one place (with
      the extractor's scale factor).
- [ ] **Reference distance goes stale after BA:** a point's reference
      (scale-prediction) distance is set when the reference is assigned; when
      BA moves the point or its reference Frame and the reference survives,
      the distance is not recomputed (ORB-SLAM3 `UpdateNormalAndDepth`).
      The same holds after a loop correction and after a global-BA merge
      (2026-10-10): the Mapper and the replicas keep consistent but slightly
      stale distances.
- [ ] **Culled Frames leave empty slots** in the Mapper's Frame array (lookup by
      id stays O(1) because ids are consecutive per map). Long runs grow it
      without bound; compact or switch to a map when it matters.
- [ ] **Make `AsyncObserver::queue()` private:** popping from it bypasses the
      priority counter (only `TryPop` / `Listener` decrement it).
- [ ] **Pre-existing flaky integration tests** (final review of
      `feat/loop-closing`, 2026-10-10; neither enables loop closing). Each
      was run 100 times (4 parallel processes × `--gtest_repeat=25`, both
      trees' runs concurrently, 16 processes) on master (d182865) and on the
      branch (ebdd88b):
  - `SlamTest.DuplicatesFusedInLongSequence`: master 2 / 100, branch
    2 / 100. Failures: a `TrackingLost` during the run (`tracking_lost`
    not empty, and once the alive-point count no longer matching), or one
    1 mm duplicate left. Thread timing (no `lock_step`).
  - `SlamTest.MonoRelocalizesAfterBlackout`: master 3 / 100 (2 hangs),
    branch 4 / 100 (2 hangs). Either a second non-final loss
    (`tracking_lost` {1, 1}), or a 120 s hang: the Tracker stays
    `RecentlyLost` at the end of the input, so neither a final
    `TrackingLost` nor the last frame is published and the harness wait
    (`slam_test.cpp`, after the frame loop) times out. Make the harness end
    on "input done and the Tracker idle", and the run deterministic
    (`RunSetup::lock_step`).
- [ ] `SlamTest.DuplicatesFusedInLongSequence`: with pixel noise the
      duplicates fusion leaves lie centimetres apart, so its 1 mm check only
      catches exact copies (without noise the synthetic run creates no
      duplicates at all). About 18 point pairs with descriptors within 10 bits
      survive fusion on the stereo leave-the-view run (112 without fusion):
      find which gate rejects them (scale range, χ², targets) and decide
      whether a descriptor-and-distance check (e.g. 10 cm) belongs in the test.
- [ ] Map-quality review leftovers (tests and nits deferred by the task
      reviews):
  - Events / replica: `MapTransaction::Reset()` frees `NewFrame` inner
    capacity; `SceneState`'s `new_map` clear is redundant with `AdmitMap`
    and may wipe an early `FrameTracked` of the new map; no test for a winner
    removed in the same transaction.
  - Urgent counter: tautological assertion in
    `AsyncObserverTest.PriorityCountTracksPushPop`; no two-observer test; the
    Tracker's urgent publish is not tested with forced low inliers.
  - `BundleAdjust`: non-positive χ² rejection untested; the stage-0 stop
    skipping outlier deactivation is not pinned by a test; the `StereoPixel`
    helpers re-derive the projection instead of reusing it; `StopCheck`
    holds the callback by reference; suite name `BundleAdjustTest`.
  - `FuseMatches`: stereo χ² uses (u_l, v_l, u_r) and ignores the right v
    (as ORB-SLAM3); level prediction rounds (`lround`) where ORB-SLAM3 uses
    `ceil`; a conflict's loser gets no fallback candidate.
  - `FramePointStats::level` is unused by `IsRedundantFrame`.
  - Mapper fusion: the old-Frame variant of the loser drop (removed record
    without re-add) is untested; an in-flight `FrameTracked` may name losers
    (ORB-like); the member-wise instantiation macro in `mapper_quality.cpp`
    is fragile.
  - Mapper refinement: `ReplicaStaysConsistent`'s "something removed" check
    is weakened by Frame culling; the ruling-J skip (no fixed Frame) is
    untested; a skipped BA logs stage 2; `UrgentEventStopsBa` checks only the
    new Frame's pose. Frame culling is aggressive on the synthetic mono unit
    sequences (about 3 Frames remain).

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
      unseeded `Random()` in `libs/test_support/support/test_measurements.h`.
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
      `TwoViewOptions` for tests or fewer frames. (2026-10-07: with
      `VOYAHSLAM_OPTIMIZE_TESTS` ON it is 0.24 s and the whole suite 2.3 s;
      this matters only with the option OFF.)
- [ ] Matcher: per-candidate descriptor copy (matters for SIFT cost).

## Library structure (restructure, 2026-10-07)

- [ ] **Descriptor-traits refactor:** template the event, map and viewer code on
      descriptor traits instead of the extractor class, so `voyah_common` no
      longer declares `OrbFeatureExtractor` / `SIFTFeatureExtractor` (their
      definitions are in `voyah_algs`; code linking only common that calls
      `DetectAndExtract` gets a link error, not a compile error).
- [ ] **Conan packaging of the separate libraries:** `install()` / package
      export per library (`voyah_common`, `voyah_events`, `voyah_algs`,
      `voyah_slam`, umbrella `voyahslam`), with `find_dependency(OpenCV)` in the
      config of the package that links it (DESIGN §7).
- [ ] **`libs/test_support/support/synthetic_sequence.h` needs algs**
      (`features/stereo_matcher.h`) but sits in the common-level support root
      (`voyah_test_support` links only `voyah_common`). Only a compile failure in
      a binary that does not link `voyah_algs` enforces it; move it to an
      algs-level support target (or into `libs/algs/tests/support`).
- [ ] **Narrow OpenCV in `voyah_common_tests`:** it links OpenCV only for the
      `calib3d` reference in the Kannala-Brandt test; link just the needed
      component (or replace the reference with stored values).
- [ ] **Watch for a flaky unit test:** one `ctest -L unit` run in Task 7 ended
      with "Errors while running CTest" and was not reproduced (5 × unit,
      2 × integration and later runs green). If it recurs, keep the
      `--output-on-failure` log and find the test.
- [ ] **Clean stale pre-restructure build directories:** the Debug and Release
      trees still contain `src/` and `tests/` build directories from the old
      layout (no longer generated); delete them or re-create the trees.

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
      2026-10-06, 92 sources + 82 public headers; 2026-10-07 each library's
      public headers with only its own and its dependencies' include roots:
      common 37, events 17, algs 23, slam 8, viz 6, clean); later add an Android NDK
      arm64 cross-build.
