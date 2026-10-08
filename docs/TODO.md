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
- [ ] LoopCloser logic (its `ProcessPending()` only acknowledges events).

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
- [ ] The post-relocalization Frame guard: ORB-SLAM3 creates no keyframe
      for about 1 s after a relocalization (see "Relocalization" below).
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
- [ ] **ORB-SLAM3's post-relocalization gate:** for about 1 s after a
      relocalization ORB-SLAM3 needs 50 local-map inliers (not 30) and
      creates no keyframe. Not carried over; add it if a wrong
      relocalization is ever seen.
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
    re-orthonormalized and the lambdas not re-checked after Gauss-Newton;
    the fisheye margin is thin (9.78e-6 against 1e-5); `pnp.h` still says
    callers refine on the inliers (ruling F refines on all matches).
  - `RecentlyLost` is unit-tested only with threshold 0; the one-node test
    vocabulary does not exercise `levels_up` 4.
  - `LeftOf` is copied a third time (`relocalizer.cpp`); `OptimizeMatchedPose`
    / `PoseRefinement` live in `relocalizer.h`; `Relocalize` is about 135
    lines.

## Map quality (sub-project 3, 2026-10-07)

Spec `docs/superpowers/specs/2026-10-07-map-quality-design.md`; DESIGN "Map
Quality" (smoke table there).

- [ ] **Global BA in a separate thread** (roadmap step 4, with loop closing):
      local BA stays in the Mapper thread; global BA needs its own thread and
      a way to merge its result into a map that kept changing meanwhile.
- [ ] **Tuning** for TUM-VI (roadmap step 6): every map-quality threshold is an
      ORB-SLAM3 default; nothing was tuned.
- [ ] **Two σ sources (ruling K):** local BA takes `sigma_px` / `level_scale`
      from `MapperOptions::fuse`, the Tracker from `TrackerOptions`. Derive
      both from one place (with the extractor's scale factor).
- [ ] **Reference distance goes stale after BA:** a point's reference
      (scale-prediction) distance is set when the reference is assigned; when
      BA moves the point or its reference Frame and the reference survives,
      the distance is not recomputed (ORB-SLAM3 `UpdateNormalAndDepth`).
- [ ] **Culled Frames leave empty slots** in the Mapper's Frame array (lookup by
      id stays O(1) because ids are consecutive per map). Long runs grow it
      without bound; compact or switch to a map when it matters.
- [ ] **Make `AsyncObserver::queue()` private:** popping from it bypasses the
      priority counter (only `TryPop` / `Listener` decrement it).
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
