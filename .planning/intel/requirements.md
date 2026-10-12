# Requirements (synthesized)

There are no PRD documents in the ingest set. This is a brownfield, already-implemented library (about 773 tests). Requirements below are derived from:
- the "Context and Scope" goals and "Success criteria" of the shipped specs in `docs/superpowers/specs/` (status: IMPLEMENTED / VALIDATED), with the acceptance results recorded in `docs/DESIGN.md` section 8 and `docs/ROADMAP.md`;
- pending work from `docs/ROADMAP.md` (step 6) and `docs/TODO.md` (status: PENDING).

Status vocabulary (written at the start of each `description`):
- `IMPLEMENTED/VALIDATED` - shipped and its acceptance bars were met.
- `IMPLEMENTED (acceptance gap)` - shipped, but a recorded smoke "Must" was missed; the gap is itself pending work.
- `PENDING` - not yet built.

Implementation plans (`docs/superpowers/plans/*`) were treated as history and produced no requirements of their own.
Where an older spec disagrees with `docs/DESIGN.md`, DESIGN.md wins (see INGEST-CONFLICTS.md, INFO section); the acceptance text below already follows DESIGN.md.

---

# Part A - Shipped capabilities (describe existing behaviour)

## REQ-event-memory-infrastructure
- source: docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md (sections 1, 2, 3, 14); docs/DESIGN.md sections 5, 6.1
- description: IMPLEMENTED/VALIDATED. Lock-free pooled, reference-counted, acknowledged events: `ObjectPool<T, SyncPolicy>` (Treiber-stack free list, grows on overflow since 0.3.0), `Recyclable` CRTP ref count, grow-only `ColumnBuffer` payloads, non-virtual `Event` base with type tag and function pointers, `Observable` (sync handlers then async enqueue), `AsyncObserver` (one queue per observer), move-only `Listener` with implicit acknowledgement. `Notify()` always takes ownership.
- acceptance: Unit tests for pool create/recycle lifecycle, ref counting and multi-observer recycle ordering; async fan-out to multiple observers; Listener auto-recycles; sync handlers run inline; ref count reaches 0 after all Listeners drain. ThreadSanitizer clean (2026-10-11: slam 266 and integration 21 tests, no warnings). Steady state allocates nothing per event.
- scope: libs/common memory, libs/events

## REQ-data-layer
- source: docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md (sections 4-9); docs/DESIGN.md section 5
- description: IMPLEMENTED/VALIDATED. Strong ids (`MapPointId`, `FrameId`, `MapId`), SoA `MapPointStorage` (append-only ids), `Frame` (keyframe; measurement held as `shared_ptr<const>`), `CovisibilityIndex` (observation record), `StereoRig` (immutable, `T_rl` left->right), `MonocularMeasurement`/`BinocularMeasurement` and the `TMeasurement` variant.
- acceptance: Per-class unit tests (storage add/remove, valid flags, measurement construction with rig, observation add for mono and stereo). IDs never reused: `FrameId` per run, `MapPointId` per map, `(map_id, id)` identifies a point.
- scope: libs/common map, features

## REQ-component-threads-and-state-machines
- source: docs/superpowers/specs/2026-10-03-tracking-states-design.md; docs/DESIGN.md section 4
- description: IMPLEMENTED/VALIDATED. `Slam<FE>` owns Tracker, Mapper, LoopCloser (and GlobalBA when loop closing is in effect), wires them asynchronously, `Start()` consumers before producers and `Stop()` producers first. Each component runs a `WorkerThread` bound to a `WakeSignal`; Tracker states `WaitingForMap`/`Tracking`/`RecentlyLost`/`Lost`; Mapper states `Initializing`/`Mapping`. Map identity through the replica rule.
- acceptance: (spec) component threads sleep when idle and wake immediately without locks; the init protocol runs end to end with real threads; stale-map events ignored everywhere; mono and stereo share event types; every state-machine transition unit-tested without threads.
- scope: libs/slam, libs/events

## REQ-two-view-geometry
- source: docs/superpowers/specs/2026-10-04-two-view-geometry-design.md; docs/DESIGN.md "Geometry Toolkit"
- description: IMPLEMENTED/VALIDATED. `RigidTransformation`, homography and essential (8-point) estimators with refinement, generic `Ransac<TEstimator>`, decomposition, triangulation-based `SelectPose`, `TwoViewReconstructor` on normalized undistorted points.
- acceptance: On synthetic general scenes with noise and up to 50 % outliers `Reconstruct` selects the essential model and recovers `T_21`; planar scenes select the homography; low parallax / too few inliers / ambiguity rejected; deterministic for the same seed; scores on a common scale. Accepted limits: translation direction < 10 degrees on general scenes, rotation < 1 degree, recovery >= 50 % (narrow) / >= 75 % (wide, 30 % outliers).
- scope: libs/algs estimators, geometry

## REQ-mono-initialization
- source: docs/superpowers/specs/2026-10-04-mono-initialization-design.md; docs/superpowers/specs/2026-10-05-init-bundle-adjustment-design.md; docs/DESIGN.md "Monocular Initialization"
- description: IMPLEMENTED/VALIDATED. Mapper initializes a real monocular map: ORB-SLAM3 `SearchForInitialization` matcher, two-view reconstruction, two-keyframe g2o bundle adjustment (5 + 40 iterations, fewer than 50 survivors fail), median-depth-1 scaling, publication as one transaction.
- acceptance: On a synthetic mono sequence the Mapper initializes once parallax is sufficient (not before) and the map's median depth is 1; matcher reproduces ORB-SLAM3 rules (window, ratio, one-to-one with replacement, orientation histogram, level restriction). BA reaches the ML optimum (translation-direction error within 0.1 degrees of an oracle on every seed 1-5; chi2 below the two-view estimate; mean direction error not worse). Apps run on corridor1 without crashing.
- scope: libs/algs, libs/slam mapping

## REQ-stereo-initialization
- source: docs/superpowers/specs/2026-10-04-stereo-initialization-design.md; docs/DESIGN.md "Stereo Initialization"
- description: IMPLEMENTED/VALIDATED. Stereo matcher (ORB-SLAM3 fisheye method, brute force ratio 0.7 plus triangulation checks) runs on the Tracker thread at dequeue; stereo `MapInitializer` builds one identity keyframe with every valid depth as a metric point (thresholds 500 features / 100 points).
- acceptance: Synthetic stereo matcher precision >= 98 % and noise-free depths exact to 1e-6; initializer points near true metric 3D points; Mapper publishes the stereo map; `binocular_full_slam corridor1 --viewer --max-frames=20` initializes within the first frames with hundreds of points and a plausible median depth.
- scope: libs/algs features, libs/slam

## REQ-camera-model-raw-pixel
- source: docs/superpowers/specs/2026-10-05-camera-model-design.md; docs/DESIGN.md "Camera Model and Raw-Pixel Residuals"
- description: IMPLEMENTED/VALIDATED (with an acceptance gap on re-initializations, later cured by map growth). `geometry::Camera{K, distortion}` with `Project`/`ProjectJacobian`/`Unproject`; projection matcher, `OptimizePose`, `BundleAdjust`, stereo gate and init matcher compare raw (distorted) pixels with sigma = `sigma_px * 1.2^level`. Brown-Conrady and Kannala-Brandt models.
- acceptance: `DistortJacobian`/`ProjectJacobian` match finite differences; Project/Unproject round trip to 85 degrees; pinhole invariance (1e-9 for fx = fy); fisheye points up to 80 degrees recovered; corridor1 mono >= 53 tracked of 400 (achieved 82), stereo tracks all 200 frames; "<= 5 re-initializations" was MISSED (18) and not tuned (history; map growth fixed the symptom).
- scope: libs/common geometry, libs/algs

## REQ-tracking-against-map
- source: docs/superpowers/specs/2026-10-05-tracking-design.md; docs/DESIGN.md "Tracking Against the Map"
- description: IMPLEMENTED/VALIDATED. Frame-to-map tracking: `MapReplica`/`AtlasReplica` fed by Mapper transactions; motion model (>= 30 inliers) -> zero-motion fallback (x4) -> reference-Frame tracking (with vocabulary) -> local map; motion-only `OptimizePose` (4 x 10 iterations, chi2 5.991 mono / 9.488 stereo); publishes `FrameTracked`; Tracker never mutates the map.
- acceptance: Synthetic: mono rotation < 0.5 degrees and translation < 2 % of travelled distance (measured on a 4-degree-parallax initial map); stereo rotation < 0.5 degrees, translation < 2 cm. Loss on empty/teleported frames then re-initialization works. ThreadSanitizer clean.
- scope: libs/slam tracking

## REQ-keyframes-and-map-growth
- source: docs/superpowers/specs/2026-10-06-keyframes-design.md; docs/DESIGN.md "Keyframes and Map Growth"
- description: IMPLEMENTED/VALIDATED (original smoke bars missed, cured by map quality). The Mapper decides Frame creation (`ShouldCreateFrame`), creates it (tracked observations, stereo close points, triangulation against covisible neighbours) as an atomic transaction; Tracker local map from covisibility (K1/K2, cap 80).
- acceptance: User symptom: "the tracker keeps on losing its position even in the beginning ... we should be able to at least track while the camera is in the initial room". Original smoke bars (first mono map >= 150 frames, stereo no loss) MISSED; satisfied by REQ-map-quality.
- scope: libs/slam mapping, tracking

## REQ-map-quality
- source: docs/superpowers/specs/2026-10-07-map-quality-design.md; docs/ROADMAP.md step 1
- description: IMPLEMENTED/VALIDATED. Every Mapper change is one `MapTransaction` (two per created Frame); fusion (`SearchInNeighbors`), recent-point culling, observation removal, Frame culling, and local BA in the Mapper thread interrupted by an urgent `FrameTracked`.
- acceptance (met): mono 400 frames of corridor1 keeps one map from initialization to the end (281-282 tracked, ATE Sim3 0.032-0.033 m); stereo 1000 no regression (one map, ATE SE3 0.036-0.037 m). Synthetic leave-the-view bounds tightened to 0.249 m (mono) / 0.037 m (stereo).
- scope: libs/slam mapping, libs/algs optimization

## REQ-bow-vocabulary
- source: docs/superpowers/specs/2026-10-07-bow-design.md; docs/ROADMAP.md step 2
- description: IMPLEMENTED/VALIDATED. OpenCV-free DBoW2 port (`bow::Vocabulary<FE>`, TF-IDF/L1, text loader, checksummed binary format, `tools/convert_vocabulary`), `SearchByBoW`, BoW per Frame, ORB offset rounding with `std::lround`.
- acceptance (met): reproduces reference DBoW2 on real descriptors (3 corridor1 frames: words, filing, bit-identical weights); triangulation time per Frame down >= 3x (achieved 26-39x); corridor1 no worse than latest table (one mono map; stereo no loss). Stereo ATE regression (0.056 / 0.189 m) cured by the motion-model gate in REQ-relocalization. Out of scope: vocabulary training, `ORBvoc.txt` licence.
- scope: libs/algs bow

## REQ-relocalization
- source: docs/superpowers/specs/2026-10-07-relocalization-design.md; docs/ROADMAP.md step 3
- description: IMPLEMENTED/VALIDATED. A short tracking loss is recovered into the same map: motion-model inlier gate (30), reference-Frame tracking, `RecentlyLost` state (5 s of measurement time), `bow::FrameDatabase` query, covisibility-grouped candidates, Lambda Twist P3P RANSAC, `OptimizePose` over all matches, ORB-exact projection passes, 50 inliers; post-relocalization guard. Only the timeout clears the map. Without a vocabulary: straight to Lost.
- acceptance (met): corridor1 mono 400 dropping 250-270 relocalizes into map 1 at frame 271; stereo 1000 dropping 500-520 relocalizes at 521/524; no-drop runs unchanged (stereo ATE SE3 0.036 m); stereo-with-vocabulary regression fixed (0.056 / 0.189 -> 0.036 m); about 10 ms per attempt.
- scope: libs/slam tracking, libs/algs

## REQ-library-restructure-and-fast-tests
- source: docs/superpowers/specs/2026-10-07-restructure-design.md; docs/superpowers/specs/2026-10-07-restructure-timings.md; docs/ROADMAP.md step 0
- description: IMPLEMENTED/VALIDATED. Four libraries (`voyah_common`, `voyah_events`, `voyah_algs`, `voyah_slam`) with separate include roots and unchanged include paths; umbrella `voyahslam`; one test binary per library plus `voyah_integration_tests`; `check`/`check_all` targets; `VOYAHSLAM_OPTIMIZE_TESTS`.
- acceptance (met): `check` < 15 s and `check_all` >= 2x faster than 76 s (measured 1.5 s / 2.3 s; originally 381 s serial Debug); wrong-direction include fails to compile; `voyahviz` links only common + events; ThreadSanitizer clean; smoke runs unchanged. Current repo (CLAUDE.md): 746 unit tests via `check` (about 10 s plus build), 773 via `check_all` (about 30 s).
- scope: build layout, CMake, tests

## REQ-loop-closing
- source: docs/superpowers/specs/2026-10-09-loop-closing-design.md; docs/ROADMAP.md step 4a+4b
- description: IMPLEMENTED (acceptance gap), opt-in and off by default (user decision 2026-10-10). LoopCloser detects on its own replica (ORB-SLAM3 `NewDetectCommonRegions`) and publishes `LoopDetected`; Mapper corrects (`CorrectLoop`: window, points, `SearchAndFuse`, essential graph, loop edge) as one `MapTransaction` at a new map epoch; spanning tree, loop edges, map epoch; normal-consistency gate (user decision); apps take `--loop_closing`.
- acceptance: Synthetic: end error with loop closing 0.04-0.15 of the error without, no tracking loss (MET). corridor1 smoke Musts (loop corrected, one map, ATE below the no-loop run) MISSED: same-heading look-alikes corrected; ATE worse with loop closing (mono 0.679 / 0.669 vs 0.341 / 0.500 m; stereo 1.295 / 18.16 vs 0.098 / 0.138 m). Gap carried by REQ-loop-lookalike-rejection-*.
- scope: libs/slam loop_closing, mapping

## REQ-global-ba
- source: docs/superpowers/specs/2026-10-10-global-ba-design.md; docs/ROADMAP.md step 4c
- description: IMPLEMENTED/VALIDATED. ORB-SLAM3 global BA (10 iterations, no robust kernel) after each loop correction in its own component `GlobalBundleAdjuster` with its own replica fed from the start; plain-data `GlobalBAFinished`; Mapper merges as one transaction at epoch + 1 propagating through the spanning tree. Exists only with loop closing; `--no_global_ba` turns it off.
- acceptance (met): synthetic loops (seeds 1-3) map-point RMS vs truth: stereo <= 0.8x essential-graph-only (measured 0.41-0.48), mono <= 0.95x per seed and <= 0.92x average (measured 0.87-0.92), end error <= 1.0x stereo / <= 1.05x mono; no tracking loss; corridor1 no crash and no loss caused by a merge. No ATE Must (false loops dominate).
- scope: libs/slam global_ba

## REQ-map-merging-atlas
- source: docs/superpowers/specs/2026-10-10-map-merging-design.md; docs/ROADMAP.md step 5
- description: IMPLEMENTED/VALIDATED. Atlas: lost maps with >= 10 Frames kept inactive; LoopCloser detects revisits of inactive maps; Mapper merges the active map into the inactive one (`MergeLocal`) as one transaction; target keeps its id and world; every replica mirrors the whole Atlas under the replica rule with shared read-only measurements; Tracker follows a merge without a reset. Opt-in with loop closing.
- acceptance (met): `SlamTest.{Mono,Stereo}MergesMaps` (seeds 1-3): one final map holding both parts, no tracking loss across the merge, whole-run error <= 1.5x the no-blackout run (measured 0.63-1.31x mono, 0.90-1.40x stereo; thin margins). Stereo corridor1 peak RSS 1.7 GB -> about 1.2 GB. corridor1 merge smoke: every merge true, no loss caused by a merge. No ATE Must.
- scope: libs/slam mapping, loop_closing, tracking

## REQ-map-viewer
- source: docs/superpowers/specs/2026-10-03-map-viewer-design.md; docs/superpowers/specs/2026-10-04-pangolin-viewer-design.md; docs/DESIGN.md "Visualization Library"
- description: IMPLEMENTED/VALIDATED. `voyahviz::MapViewer` (own thread, single inbox, fixed-rate loop) over an event-only `SceneState`, Pangolin renderer private to voyahviz, apps `--viewer`; shows the active map (tracked green, visible blue, other grey; local-map Frames orange).
- acceptance: Every event delivered to the viewer is acknowledged (pool slots return, including queued events at stop); `SceneState` depends only on event contents; headless viewer tests green including under ThreadSanitizer; closing the window stops playback and, if playback finishes first, the window stays open until closed.
- scope: vizualization/

## REQ-opencv-free-libraries
- source: docs/DESIGN.md section 2 (principle 8), section 8 (2026-10-03, 2026-10-11 rows); CLAUDE.md
- description: IMPLEMENTED/VALIDATED. The four `voyah_*` libraries use no OpenCV: FAST ported bit-exact from OpenCV 5 (scalar, SSE2, AVX2 run-time dispatch, NEON), SIFT removed, `BrownConradyDistortionModel` added, own PnP/BoW. Only `voyahviz`, tests and benchmarks link OpenCV.
- acceptance: ORB output byte-identical to the previous `cv::FAST` build (golden digests); FAST time 0.90-0.97 of OpenCV per whole level; a package export needs no `find_dependency(OpenCV)`.
- scope: libs/algs, build

## REQ-packaging-and-android
- source: docs/DESIGN.md section 7 "Build options and packaging", section 8 (2026-10-11, 2026-10-12 rows); CLAUDE.md
- description: IMPLEMENTED (partly validated; arm64 device runs pending). Conan 2 package `voyahslam` (currently 0.3.0) exporting `voyahslam::*` targets; options `with_tests`/`with_examples`; C++20 required; Android arm64 profile (API 24, clang 21, NDK r30, `c++_static`); shared immutable `TDistortionModelPtr`; object pools grow on overflow instead of aborting.
- acceptance: `conan create` builds the package and the consumer `test_package` links `voyahslam::voyahslam`; package build needs neither tests nor examples; Android `conan create` cross-builds (device run is a pending TODO).
- scope: conanfile.py, profiles/, CMake

---

# Part B - Pending work

## REQ-loop-lookalike-rejection-drift-bound
- source: docs/TODO.md "TUM-VI tuning" (candidate 1); docs/ROADMAP.md step 6
- description: PENDING (variant 1 of 3 competing candidates; user must choose). Drift-bound gate: reject a loop whose query and match region lie farther apart in the map than the drift could explain since the match, e.g. a bound per metre travelled. Safe in stereo while drift << 15 m; mono needs a scale-aware bound.
- acceptance: On corridor1 (Release, `--vocabulary --loop_closing`, full sequence) same-heading look-alikes (frames 2215-2236 vs 692-807 at 14.9 m; 3210->792 at 22 m; 4666->4029 at 22 m; 5017->2950 at 30 m) are rejected, true returns are kept, and ATE with loop closing is no worse than without (baselines: mono Sim3 0.341 / 0.500 m, stereo SE3 0.098 / 0.138 m). Retest the normal gate afterwards. (Acceptance bar inferred from the loop-closing spec smoke Must; the TODO states the goal, not numbers.)
- scope: loop closing, TUM-VI tuning

## REQ-loop-lookalike-rejection-verify-before-publish
- source: docs/TODO.md "TUM-VI tuning" (candidate 2); docs/DESIGN.md decisions log 2026-10-09 (option rejected for the first round)
- description: PENDING (variant 2 of 3 competing candidates; user must choose). Verify before publishing: check the reprojection error of the fused points after the essential graph, or the next local BA's initial cost, and drop a correction that breaks the map. Candidate 3 is both together.
- acceptance: Same corridor1 bars as REQ-loop-lookalike-rejection-drift-bound. This option was rejected for the first round on 2026-10-09 but is listed again as a candidate.
- scope: loop closing, merging, TUM-VI tuning

## REQ-loop-drift-consistency-check
- source: docs/TODO.md "TUM-VI tuning" (user decision 2026-10-11)
- description: PENDING. [USER DECISION 2026-10-11] Reject a loop whose correction cannot be explained by drift: compare the loop's Sim3 with the map's own estimate of the query-match pose and reject when the difference exceeds what drift could accumulate along the path between them; bound rotation and translation per metre travelled; mono also needs a scale bound. Does not apply to merges (two maps share no world).
- acceptance: Diagnosed example 2262->1587 (map puts the Frames 9.2 m and 176 degrees apart, the Sim3 claims 0.2 m and 6 degrees, stereo drift on corridor1 about 0.1 m) must be rejected. Context: ORB-SLAM3's visual-inertial mode rejects a correction rotating roll/pitch by >= 0.008 rad or yaw by >= 0.349 rad.
- scope: loop closing

## REQ-merge-false-match-protection
- source: docs/TODO.md "Loop closing" (hypothesis-independent check for merges), "Map merging follow-ups"; docs/DESIGN.md decisions log 2026-10-10 (normal gate not applied to merges)
- description: PENDING. Merges currently have weaker false-merge protection than loops (no normal gate across maps; merging stays opt-in). Needs a hypothesis-independent check, likely verify-before-publish.
- acceptance: No false merge on corridor1 `--drop_frames` smoke (currently every observed merge was true); thin integration margins (map 1 holds 10-11 Frames vs `min_frames_to_keep` 10; stereo seed 2 ratio 1.33-1.40 vs bar 1.5) widened.
- scope: map merging

## REQ-tumvi-tuning
- source: docs/ROADMAP.md step 6; docs/TODO.md (Map quality "Tuning", Initialization, Stereo, Camera model sections)
- description: PENDING ("at the very end"). Tune every map-quality, initialization and stereo threshold on TUM-VI 512 (`dataset-corridor1_512_16`): local-map inlier gate (30 vs 20), initialization acceptance and parallax, rescale before init BA, stereo depth cap (about 17 m), mono translation accuracy at default init, `ShouldCreateFrame` parameters.
- acceptance: not stated in the sources (to be set when step 6 is specced).
- scope: tuning

## REQ-orb-extractor-rework
- source: docs/TODO.md "ORB extractor (OQ 21)"; docs/DESIGN.md Open Question 21, section 10 "Next"
- description: PENDING. Run FAST once per pyramid level and assign corners to cells (>= 20 if any, else >= 7); preallocate blur and pyramid buffers; check 19-px border handling; parallel left/right extraction; profile 8-bit vs float working images; re-record ORB golden digests (`VOYAHSLAM_RECORD_ORB_GOLDEN=1`).
- acceptance: not stated numerically; fixes undetectable 6-px strips at cell borders and per-frame allocations, then re-profile.
- scope: libs/algs extractor

## REQ-relocalization-into-inactive-maps
- source: docs/TODO.md "Relocalization", "Map merging follow-ups"; docs/DESIGN.md section 10 "Next"
- description: PENDING. Relocalization currently targets the active map only; extend to inactive Atlas maps. Also MLPnP as an alternative, candidate cap, scale-distance gate in `SearchFromFrame`.
- acceptance: not stated.
- scope: Tracker, Atlas

## REQ-remote-visualization-and-serialization
- source: docs/TODO.md "Visualization", "Global BA" (serialization and gRPC); docs/DESIGN.md section 4 "Visualizer"
- description: PENDING. Event serialization and transport (HTTP/ROS/gRPC) so SLAM and viewer (and pluggable components) run on different machines; separate image flow for the camera view; inactive maps in the viewer; VBO rendering for large maps; macOS setup.
- acceptance: not stated; constraint: consumers use only event contents (plain data, no pointers into map objects).
- scope: viewer, events

## REQ-component-late-joining
- source: docs/TODO.md "Global BA"; docs/DESIGN.md decisions log 2026-10-10
- description: PENDING. A component starting mid-run (GlobalBA, LoopCloser, remote) needs a full-map snapshot to build its replica; today components exist from the start or not at all (user decision).
- acceptance: not stated.
- scope: components, serialization

## REQ-global-ba-hardening
- source: docs/TODO.md "Global BA"
- description: PENDING. Iteration or size cap / time budget (stereo corridor1 runs took up to 85 s; interruptions land only at the next LM iteration, up to about 6 s; `Slam::Stop()` waits 6-8 s); memory (about 1 MB per Frame; peak RSS 4.23 GB stereo with global BA vs 2.28 GB without, before shared measurements); retry of a dropped `GlobalBAFinished`; re-measure map loss once look-alikes are rejected.
- acceptance: not stated.
- scope: global BA

## REQ-lossy-observer-delivery
- source: docs/TODO.md "Map & events" (deferred 2026-10-12)
- description: PENDING. Per-observer delivery mode (e.g. `AddObserver(o, Delivery::Lossy)`) with a bounded queue that drops and releases the oldest events, for display-only consumers. Never for `MapTransaction` replicas or the Mapper's `FrameTracked` input.
- acceptance: a stalled display-only observer no longer grows the Tracker/Mapper pools without bound.
- scope: events

## REQ-mixed-mono-stereo-maps
- source: docs/DESIGN.md section 1, Open Question 17; docs/TODO.md "Threads"
- description: PENDING (long-term goal beyond ORB-SLAM3). Monocular and binocular keyframes coexist in one map (e.g. a stereo rig whose second camera drops out, or merging mono and stereo sessions). Today `Frame<FE, TMeasurement>` fixes one type per map.
- acceptance: not stated; behaviour of mono-then-stereo and stereo-then-mono sequences to be designed.
- scope: data model, Mapper, Tracker

## REQ-storage-growth-and-pool-sizing
- source: docs/DESIGN.md Open Questions 2, 14, 25, section 10 "Required follow-up"; docs/TODO.md "Map & events"
- description: PENDING. Decide `MapPointStorage` growth/compaction (append-only ids leave dead slots; Tracker scans all ids per frame); size event pools per event type; event pool lifetime (pools that outlive all producers and consumers).
- acceptance: not stated.
- scope: memory, events

## REQ-tracker-measurement-policy
- source: docs/DESIGN.md Open Question 19; docs/TODO.md "Threads"
- description: PENDING. Track every measurement in order, or skip to the newest when falling behind (possibly configurable); decide whether `Tracker::Stop()` drains or discards queued measurements.
- acceptance: not stated.
- scope: Tracker

## REQ-geometry-robustness
- source: docs/TODO.md "Geometry", "Camera model"; docs/DESIGN.md Open Question 22
- description: PENDING. 5-point essential solver (priority raised for narrow FOV); fix refinement inlier-set lock-in; bearing vectors instead of z = 1 points (full fisheye FOV, features beyond 85 degrees); `rational_polynomial` distortion; per-model camera classes only if needed.
- acceptance: not stated.
- scope: libs/algs geometry, measurements

## REQ-dataset-validation
- source: docs/TODO.md "Dataset / preprocessing", "Performance"
- description: PENDING. Real-data run on EuRoC MH_01 (mono and stereo, Brown-Conrady, never run on EuRoC images; check tracking, maps, ATE with `tools/ate.py`); own CLAHE as app-side preprocessing; benchmark on TUM-VI 512x512.
- acceptance: tracking, maps and ATE reported.
- scope: apps, tuning

## REQ-platform-validation
- source: docs/TODO.md "ORB extractor", "Library structure", "Environment", "Map & events"
- description: PENDING. Run FAST NEON path and `test_package` on arm64 hardware/Android device; Android logcat sink; symbol visibility for shared builds; real package licence (currently placeholder `proprietary`); ASan runs abort in g2o (Eigen malloc ODR mix); g2o under clang/libc++; `check_all` should also build the apps.
- acceptance: NEON FAST bit-exact against the arm64 reference and timed with `fast_benchmark`.
- scope: build, Android

## REQ-vocabulary-training-and-licence
- source: docs/TODO.md "Bag of words"; docs/superpowers/specs/2026-10-07-bow-design.md
- description: PENDING (deferred by user decision). Vocabulary training (own k-majority); `ORBvoc.txt` licence; phantom-node difference vs ORB-SLAM3.
- acceptance: not stated.
- scope: BoW

## REQ-descriptor-traits-refactor
- source: docs/TODO.md "Library structure"; docs/superpowers/specs/2026-10-07-restructure-design.md (out of scope)
- description: PENDING. Template event, map and viewer code on descriptor traits instead of the extractor class so `voyah_common` no longer declares `OrbFeatureExtractor`.
- acceptance: code linking only common that calls `DetectAndExtract` fails at compile time rather than link time.
- scope: layering
