# Constraints (synthesized)

Extracted from the SPEC documents (`docs/superpowers/specs/*`) as amended by `docs/DESIGN.md` (precedence 0, wins on conflict). Each block states the CURRENT contract; superseded spec wording is listed in INGEST-CONFLICTS.md (INFO). Detailed option defaults live in DESIGN's per-feature sections.

---

## Event delivery and pooled lifecycle
- source: docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md (sections 1-3); docs/DESIGN.md section 6.1
- type: protocol
- content: Events are pooled (`ObjectPool` per concrete type, lock-free Treiber free list with 64-bit tagged head), reference-counted (`Recyclable`, count starts at 1, `SetRefCount(n)` for fan-out, acq_rel), and kept alive between uses (`Reset()` if defined; payloads in grow-only `ColumnBuffer`). Producer: `Create()` (never fails; grows by doubling since 0.3.0, logs once per growth) or `TryCreate()` (null on exhaustion, used by LoopCloser and GlobalBA which drop on purpose); fill payload; `Notify()` takes ownership (sync handlers first, then `SetRefCount(N)` and enqueue to N async queues); the producer must not touch the event afterwards; events without `Reset()` (`LoopDetected`, `TrackingLost`, `InitializationMeasurement`) require the producer to set every field. Consumer: acknowledge implicitly by asking the `Listener` for the next event; never keep an `Event*` beyond that; treat payloads read-only (no moving out); do not sit on events. Per-channel delivery is FIFO. Priority events increment the inbox `pending_priority` counter on `Push` and only `TryPop`/`Listener` decrement it.

## MapTransaction contract
- source: docs/superpowers/specs/2026-10-07-map-quality-design.md (section 2); docs/superpowers/specs/2026-10-09-loop-closing-design.md (section 1); docs/superpowers/specs/2026-10-10-map-merging-design.md (section 1); docs/DESIGN.md sections 6, "Map Quality"
- type: schema
- content: The only Mapper event. Parts (each may be empty): `map_id`, `new_map` + `start_frame`; added Frames (id, `T_cw`, shared measurement, own observations, BowVector/FeatureVector, `parent`); new points (`new_points_start_id`, contiguous ids, positions/normals/descriptors); `added_observations` (frame, feature, point, depth, right index; -1 when absent); updated points; `replaced_points` (loser, winner); `removed_points` (every removal listed, including implicit ones); `updated_poses`; `removed_observations`; `removed_frames`; `parent_changes`; `added_loop_edges`; `map_epoch` (+1 per loop correction/merge/global-BA merge; 0 for a new map); `active_map` (set on every transaction), `removed_maps`, `absorbed_map`. Application order is fixed: new map, add Frames, add points, add observations, update points, update poses, remove observations, remove points, remove Frames; applied atomically in one step. A transaction must never be dropped (pooled, default 256, grows on overflow). Consumers apply on receipt and never hold one for long work.

## Replica rule (Atlas mirroring)
- source: docs/superpowers/specs/2026-10-10-map-merging-design.md (section 1); docs/DESIGN.md section 6 (replica rule)
- type: protocol
- content: Every replica (Tracker, LoopCloser, GlobalBA `tracking::AtlasReplica`; viewer `SceneState`) mirrors the Mapper's Atlas. Per transaction of map `map_id`: map 0 is ignored; `new_map` creates the map; any other transaction applies to the live map it names, an unknown or removed map is ignored whole; `absorbed_map` first moves the absorbed map's Frames/observations/points into the named map by the id rule (`map::AbsorbedPointIds`); then `removed_maps` are dropped and remembered (never re-admitted); then the active map is taken from `active_map` only, never from the newest id. Other events change no replica. The Mapper builds map n+1 only after a final `TrackingLost(n)`.

## ID rules
- source: CLAUDE.md; docs/DESIGN.md section 2 (principle 5), section 8 (2026-10-05)
- type: schema
- content: `FrameId` unique per run, never reused. `MapPointId` unique within a map, append-only (`Remove` only marks dead; `num_ids()` = next id); restarts per map; `(map_id, id)` identifies a point. `MapId` starts at 1, only increases, 0 = not tied to a map. Stale ids from lagging replicas are detectable by a plain `IsValid()` check. After a merge the absorbed map's alive points take the target's next ids in old-id order (id rule); Frame ids may interleave across merged maps.

## Pointer rule and shared immutable payloads
- source: docs/superpowers/specs/2026-10-03-map-viewer-design.md ("Constraints Carried From the Conversation"); docs/DESIGN.md section 6 (pointer rule)
- type: nfr
- content: An event may point only at data that never changes. Raw pointers only to data that lives for the whole run (configuration); immutable data that may die earlier is shared-owned. Two payloads are `shared_ptr<const ...>`: a Frame's measurement (`TSharedMeasurement<FE>`, since 2026-10-10) and a `DistortionModel` (`TDistortionModelPtr`, since 2026-10-12, null = pinhole). Everything mutable or short-lived travels by value. A serializer must be able to turn references into values.

## Threading and map access
- source: docs/DESIGN.md sections 2, 4; docs/superpowers/specs/2026-10-03-tracking-states-design.md
- type: nfr
- content: No locks on map data. Mapper is the only writer; Tracker/LoopCloser/GlobalBA are read-only and publish requests or facts; GlobalBA and LoopCloser exchange plain data only (no pointers into map objects) so an out-of-process transport stays possible. Components use `WakeSignal` (atomic counter with C++20 wait/notify; snapshot taken before the `running` check) and `WorkerThread`. `Start()` order Mapper, GlobalBA, LoopCloser, Tracker; `Stop()` reverse. `Slam` is neither copyable nor movable. A component changes its state before publishing the event that announces it.

## Library layering and public API surface
- source: docs/superpowers/specs/2026-10-07-restructure-design.md; docs/DESIGN.md section 7; CLAUDE.md
- type: api-contract
- content: `voyah_common` (base), `voyah_events` and `voyah_algs` (each on common only), `voyah_slam` (on all three), `voyahslam` umbrella; each library has its own include root, with unchanged `#include <voyahslam/...>` paths. A lower layer including an upper header must fail to compile. Public headers never include OpenCV, Pangolin or g2o; those are linked PRIVATE; small deps (Eigen, spdlog, concurrentqueue) may be public. The four `voyah_*` libraries use no OpenCV at all. Extractor classes are declared in common and defined in algs. `voyahviz` depends only on common + events (OpenCV, Pangolin PRIVATE).

## Coding conventions
- source: docs/DESIGN.md section 7; CLAUDE.md
- type: api-contract
- content: C++20. Google naming (`PascalCase` types/functions, trivial accessors may be `snake_case`, `snake_case` locals, `snake_case_` members, `out_` for non-const-reference outputs, `kPascalCase` constants, `T`-prefixed template parameters/aliases, `enum class`, namespaces `voyahslam::<module>`). Include guards `VOYAHSLAM_<PATH>_H` (no `#pragma once`); every header self-contained; one event type per header; `[[nodiscard]]` on const accessors; templates over the extractor instantiated with `INSTANTIATE_CLASS` (ORB only); `.clang-format` 4-space; new sources in `libs/<lib>/CMakeLists.txt`; a test above 1 s in the optimized build goes to `libs/slam/tests/integration/`.

## Build, test and packaging contract
- source: docs/DESIGN.md section 7; CLAUDE.md; docs/superpowers/specs/2026-10-07-restructure-design.md (section 6)
- type: nfr
- content: Conan 2 + CMake >= 3.23. Debug `cmake --build cmake-build-debug/build/Debug -j8`; fast loop target `check` (label `unit`); `check_all` runs `unit` + `integration` + `vocabulary`; both run in the Debug tree with `VOYAHSLAM_OPTIMIZE_TESTS=ON` (-O2 -g, asserts on). Tests, examples and tools are built only with `VOYAHSLAM_BUILD_TESTS` / `VOYAHSLAM_BUILD_EXAMPLES` (OFF by default). g2o is a local Conan package (`g2o/20241228.1@voyah/stable`) built once per machine; OpenCV 5.x is built from source in `/usr/local` for dev tools and tests. Package requires `compiler.cppstd >= 20`; `shared=True` rejected on Windows and with `c++_static`; Android uses a single JNI `.so` and `c++_static`. Current counts per CLAUDE.md: 746 tests in `check`, 773 in `check_all`.

## Event payload and Frame/measurement schema
- source: docs/superpowers/specs/2026-10-06-keyframes-design.md (section 2); docs/DESIGN.md section 6
- type: schema
- content: `FrameTracked<FE>` (Tracker -> Mapper, viewer): timestamp, `pose` (`T_cw` after `OptimizePose`), shared measurement, inlier pairs `observed_features[i] -> observed_points[i]`, `visible_points` (matched ones included), `reference_frame_id` (optional), `local_frames`, `stage` (`MotionModel`/`ZeroMotion`/`ReferenceFrame`/`Relocalization`), `num_matches`, `num_inliers`, `urgent` (mirrored in `Event::priority`; set when final inliers < 1.5 x `min_local_map_inliers`), `map_epoch`. `LoopDetected`: `query_map_id`, `match_map_id`, `query_frame`, `match_frame`, `S_query_match` (`geometry::Sim3`, x_query = S * x_match, camera to camera, scale 1 for stereo), `matched_points`; pool of 16, `TryCreate`. `GlobalBAFinished`: `map_id`, `map_epoch`, `frame_poses`, `point_positions` (plain data); pool of `max_events_in_flight` (2), `TryCreate`. `TrackingLost`: timestamp, `final`. `InitializationMeasurement`: `TMeasurement<FE>`, `map_id` 0.

## Geometry conventions
- source: docs/superpowers/specs/2026-10-04-two-view-geometry-design.md; docs/superpowers/specs/2026-10-04-stereo-initialization-design.md; docs/DESIGN.md "Geometry Toolkit", "Camera Model"
- type: api-contract
- content: `RigidTransformation`: `T_ab` maps points from frame b to frame a (`T_ac = T_ab * T_bc`), representation hidden. Keyframe poses are `T_cw` (world -> camera); the world frame of a map is its first Frame's camera. `StereoRig::T_rl` maps left -> right (`p_r = T_rl p_l`). Two-view geometry and triangulation work on rays (z = 1 normalized, features beyond 85 degrees off axis dropped). All residuals and gates (projection matcher, `OptimizePose`, `BundleAdjust`, stereo gate, init matcher windows, triangulation chi2) are in raw distorted pixels with sigma = `sigma_px * level_scale^level`, chi2 5.991 mono / 9.488 stereo.

## Camera model API
- source: docs/superpowers/specs/2026-10-05-camera-model-design.md (section 1)
- type: api-contract
- content: `geometry::Camera{K, distortion}` value type with `Project`, `ProjectJacobian`, `Unproject`; `DistortionModel` (Dummy, Kannala-Brandt, Brown-Conrady) in `geometry/distortion_models/` with analytic `DistortJacobian`; measurements keep raw keypoints (`distorted_points()`) and `camera()`. Hot paths do not copy a `Camera` (g2o edges hold `const Camera&`). Pinhole invariance holds exactly only for fx = fy.

## Optimization API
- source: docs/superpowers/specs/2026-10-05-init-bundle-adjustment-design.md (section 2); docs/superpowers/specs/2026-10-05-tracking-design.md (section 3); docs/DESIGN.md "Optimization"
- type: api-contract
- content: g2o vendored pristine, private, plain Eigen types in public API. `BundleAdjust`: two-stage (Huber 5 iterations, deactivate chi2 > 5.991 or depth <= 0, plain 10 iterations by default; init BA 5 + 40), accepts `std::function<bool()> should_stop` checked before the first iteration and after every iteration; outliers removed only when `stage_reached >= 1`. `OptimizePose`: unary g2o edges on one `VertexSE3Expmap`, 4 rounds x 10 iterations restarting from the initial pose, Huber in rounds 0-1, success >= 3 inliers; stereo edges have 4 rows via `StereoCameras{right, T_rl}`.

## Tracker flow thresholds
- source: docs/superpowers/specs/2026-10-05-tracking-design.md; docs/superpowers/specs/2026-10-07-relocalization-design.md; docs/DESIGN.md "Tracking Against the Map", "Relocalization..."
- type: nfr
- content: Motion model window 15 px mono / 7 px stereo scaled by `level_scale^level`, needs 20 matches (else window x2) and >= 30 final inliers (gate since 2026-10-08); zero motion window x4, >= 10 inliers; reference-Frame tracking (with vocabulary) `SearchByBoW` ratio 0.7, >= 15 matches, >= 10 inliers; local map `th` 1.0, >= 30 inliers. Loss: with vocabulary and a map of more than 10 Frames, failure enters `RecentlyLost` for 5 s of measurement timestamps and relocalizes each measurement (50 inliers, post-relocalization guard 1 s, Mapper guard 1.0 s while map > 20 Frames); only the timeout publishes final `TrackingLost`. Without a vocabulary or on a small map: straight to `Lost`.

## Mapper pipeline and options
- source: docs/superpowers/specs/2026-10-06-keyframes-design.md; docs/superpowers/specs/2026-10-07-map-quality-design.md (sections 1, 6); docs/DESIGN.md "Mapper", "Map Quality"
- type: protocol
- content: Per created Frame in the Mapper thread: new Frame and tracked observations, recent-point culling, new points (stereo close points, triangulation with optional `SearchByBoW`), fusion (`SearchInNeighbors`), local BA (20-Frame cap including the new Frame; skipped with no free or no fixed Frame; interrupted only by an urgent `FrameTracked`), Frame culling (loop-edge Frames and the root never culled). Two transactions per Frame: T1 (before BA), T2 (BA results and culling). `FrameTracked` drained to the latest per map; skipped ones counted as backlog. A stereo-matched observation counts 2 for `ref_points`/`min_obs`. A `FrameTracked` of an older epoch is dropped (after counting visible/found). Inactive maps are frozen.

## Bag-of-words file formats and API
- source: docs/superpowers/specs/2026-10-07-bow-design.md (sections 1-2); docs/DESIGN.md "Bag of Words"
- type: schema
- content: `bow::Vocabulary<FE>` (TF-IDF, L1 only): `LoadText`, `LoadBinary`, `SaveBinary`, `FromTree`, `Transform(descriptors, levels_up, out_bow, out_features)`, `Score`. `BowVector` = sorted (WordId, weight) pairs, L1-normalized; `FeatureVector` = CSR (nodes, offsets, features). Binary format magic `VOYBOW\0\1`, FNV-1a 64 checksum; loaders never throw and reject descriptor-size mismatch. `levels_up` 4. The vocabulary is loaded once by the app and shared as `std::shared_ptr<const bow::Vocabulary<FE>>`; null means no BoW. The 145 MB `ORBvoc.txt` is never committed; `$VOYAHSLAM_ORBVOC` enables the `vocabulary` test label.

## Loop closing options and algorithm contract
- source: docs/superpowers/specs/2026-10-09-loop-closing-design.md (sections 2-4); docs/DESIGN.md "Loop Closing"
- type: protocol
- content: LoopCloser (detection only, on its own replica): `min_map_frames` 12; `num_candidates` 3; `num_covisibles` 10; `min_bow_matches` 20; RANSAC inliers 15 (cap 300 iterations, confidence 0.99, chi2 9.21); `OptimizeSim3` 20; coarse projection 50; projection 80; `confirmation_projection` 30; `refine_min_optimized` 50; `refine_min_projection` 100; `required_coincidences` 3; `max_unconfirmed_queries` 2; normal-consistency gate `max_normal_disagreement` 0.5 over 60 degrees (loops only); `max_query_backlog` 8. Mapper correction (`CorrectLoop`): window, point correction, `SearchAndFuse` (`fuse_radius` 4), essential graph (`min_essential_weight` 100, 20 iterations), loop edge, one transaction at `map_epoch + 1`. Loop closing is opt-in (`LoopCloserOptions::enabled` false by default, apps `--loop_closing`) and requires a vocabulary.

## Global BA contract
- source: docs/superpowers/specs/2026-10-10-global-ba-design.md; docs/DESIGN.md "Global BA"
- type: protocol
- content: `GlobalBundleAdjuster` runs in its own thread with its own replica fed from the start (no late joining). A run starts on an applied transaction of the current map with a new loop edge; 10 iterations, no robust kernel; snapshot taken before the first poll and never re-read; apply-on-poll interruption by a newer correction, new map or stop; result `GlobalBAFinished`. Mapper merge: only in `Mapping`, current map and epoch; breadth-first spanning-tree walk, parents first; Frames/points created during the run follow their parent/reference Frame; moved points' normals recomputed; one transaction at `map_epoch + 1`, no loop edge. Created only with loop closing in effect; `GlobalBaOptions::enabled` default true; `--no_global_ba`.

## Atlas and map-merging contract
- source: docs/superpowers/specs/2026-10-10-map-merging-design.md (sections 1-5); docs/DESIGN.md "Mapper" (merge)
- type: protocol
- content: Mapper holds maps by id with at most one active. On final `TrackingLost`: fewer than `atlas.min_frames_to_keep` (10) Frames -> discarded (`removed_maps`), else kept inactive; the next initialization takes the next id. Inactive maps are frozen (no local mapping, culling, BA, global BA). Merge detection reuses `LoopCloserOptions`, no normal gate for merges, merge wins over a loop at the same query, mono merges free-scale. Merge (`MergeLocal`, visual): absorb M into N by the id rule, `CorrectWindow` (`atlas.merge_window` 25), re-root tree, `FuseLoopPoints`, weld (`WeldWindows`), move rest of M, essential graph with welded window fixed, merge edge stored as a loop edge; one transaction with `absorbed_map = M`, `removed_maps = {M}`, `active_map = N`. Rejected when the query map is not active, the match map is not live and inactive, a Frame is gone or the Sim3 is invalid. Lost maps are kept only when `Slam` has a LoopCloser (loop closing enabled and a vocabulary). The Tracker follows a merge without a reset; a Lost Tracker re-announces its final loss for the target.

## Viewer lifecycle contract
- source: docs/superpowers/specs/2026-10-03-map-viewer-design.md; docs/superpowers/specs/2026-10-04-pangolin-viewer-design.md; docs/DESIGN.md "Map Viewer"
- type: protocol
- content: `SceneState` is rebuilt purely from events (no access to Slam/components/shared memory), no threads or GL; only the viewer-loop thread touches `SceneState`, renderer and GL context. Loop: drain inbox through a `Listener` (acknowledges each event), apply, render if the window is open, sleep to the next absolute deadline. `Start()` runs on its own thread; `RunOnCurrentThread()` on the caller's thread (macOS). Window close does not end the loop (keeps draining headless); only `RequestStop()`/`Stop()` end it; `fps <= 0` throws. Connect with `AddObserver(viewer.inbox())` before producers start; stop `Slam` before the viewer. Shows the active map only.

## Reference-application CLI contract
- source: docs/DESIGN.md "Example Applications"; docs/superpowers/specs/2026-10-04-pangolin-viewer-design.md (section 4)
- type: api-contract
- content: `<app> <dataset_root> [--rate=R] [--max-frames=N] [--show] [--viewer] [--trajectory_out <file>] [--vocabulary <file.bin>] [--loop_closing] [--no_global_ba] [--drop_frames a:b]`. `--loop_closing` needs the vocabulary and is off by default. Trajectory output is TUM format camera-to-world with one `# map <id>` section per map; `tools/ate.py` (Umeyama Sim3/SE3, 20 ms association, `--map ID`). Configuration lives in the app; the dataset reader (EuRoC/TUM-VI layout, Kalibr or EuRoC calibration, Brown-Conrady strings `radial-tangential`/`radtan`/`plumb_bob` with 4 or 5 coefficients) is synchronous and paced by `PlaybackClock`.

## Performance and resource budgets recorded as requirements-by-measurement
- source: docs/ROADMAP.md; docs/DESIGN.md section 8 (2026-10-07 .. 2026-10-12 rows)
- type: nfr
- content: Triangulation per Frame at least 3x faster with a vocabulary (measured 26-39x); relocalization attempt about 10 ms; FAST at least as fast as `cv::FAST` single thread (0.90-0.97); stereo corridor1 peak RSS about 1.2 GB with shared measurements (global BA adds a replica: 4.23 GB measured before sharing); global BA 3-85 s on corridor1 with merges of 4-32 ms; local BA about 105 ms mono / 135 ms stereo per Frame; Mapper pass about 140 ms; object pools default 256 slots, growth logged and counted. TUM-VI reference dataset: `dataset-corridor1_1024_16` (20 Hz stereo, 1024 x 1024, equidistant fisheye, baseline about 0.101 m).
