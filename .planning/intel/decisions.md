# Decisions (synthesized)

Source of truth: `docs/DESIGN.md` section 8 "Decisions Log" (precedence 0, treated as the authoritative set of LOCKED decisions per the ingest override), plus the principles in DESIGN section 2 and the user decisions recorded in `docs/ROADMAP.md`.

There are no formal ADR files in the ingest set. Each row of the decisions log is one entry below, in log order (date, then log position). Rationale lives in the source row; it is summarized only where it changes how the decision applies.

Status convention: `locked` = current and in force. `locked (superseded by X)` = a log entry that was later reversed or narrowed; kept for history and must NOT be treated as current. Entries marked `[USER DECISION]` are recorded in the log as explicit user decisions.

Every entry's `source` is `docs/DESIGN.md` section 8 (row dated as in the title) unless stated otherwise.

---

## Core principles (DESIGN section 2)

## PRINCIPLE-1: No locks on the map or event path
- source: docs/DESIGN.md section 2 (principle 1)
- status: locked
- decision: No mutex anywhere on the map or event path. Cross-thread communication only via events on `moodycamel::ConcurrentQueue`; event pools are lock-free.
- scope: whole library

## PRINCIPLE-2: One replica of the map per thread
- source: docs/DESIGN.md section 2 (principle 2)
- status: locked
- decision: Tracker, Mapper, LoopCloser, GlobalBA (and the Visualizer) each own a local copy of the map (`MapPointStorage`, `Frame`s, `CovisibilityIndex` as observation record since 2026-10-07).
- scope: threading, map replicas

## PRINCIPLE-3: Mapper is the single writer
- source: docs/DESIGN.md section 2 (principle 3)
- status: locked
- decision: All changes to map objects (create/update/cull/fuse points, create Frames, adjust poses, close loops, merge maps) happen in the Mapper, which publishes diff events that every other replica applies verbatim.
- scope: Mapper, all replicas

## PRINCIPLE-4: Other threads are read-only on the map
- source: docs/DESIGN.md section 2 (principle 4)
- status: locked
- decision: Non-Mapper threads compute from their replica and publish requests or facts; they never mutate map objects.
- scope: Tracker, LoopCloser, GlobalBA, viewer

## PRINCIPLE-5: IDs are never reused
- source: docs/DESIGN.md section 2 (principle 5); CLAUDE.md; decisions-log row 2026-10-05
- status: locked
- decision: `FrameId` unique for the whole run; `MapPointId` unique within its map (append-only, restarts per map), so `(map_id, id)` identifies a point. Stale IDs from lagging replicas are always detectable.
- scope: ids, events, replicas

## PRINCIPLE-6: No virtual dispatch on hot paths
- source: docs/DESIGN.md section 2 (principle 6)
- status: locked
- decision: Events dispatch on a type tag + `static_cast`; recycling through a function pointer; mono/stereo chosen at compile time via templates.
- scope: events, templates

## PRINCIPLE-7: Structure-of-Arrays storage
- source: docs/DESIGN.md section 2 (principle 7)
- status: locked
- decision: Map points and observations use SoA storage so hot loops (projection, matching, BA) run over contiguous Eigen matrices.
- scope: map storage

## PRINCIPLE-8: OpenCV is an implementation detail
- source: docs/DESIGN.md section 2 (principle 8)
- status: locked
- decision: Public headers never include OpenCV; OpenCV linked PRIVATE. Since 2026-10-11 the four `voyah_*` libraries use no OpenCV at all; only `voyahviz`, tests and benchmarks link it. Small deps (Eigen, spdlog, concurrentqueue) may appear in public headers.
- scope: public API, build

---

## Decisions log (section 8)

## DEC-2026-09-30-1: Event delivery layer is non-template and carries Event*
- source: docs/DESIGN.md section 8 (2026-09-30)
- status: locked
- decision: `Observable`/`AsyncObserver`/`Listener` are non-template and carry `Event*`; concrete types known only at the consumer's `switch`.
- scope: events

## DEC-2026-09-30-2: Sync handlers before async enqueue
- source: docs/DESIGN.md section 8 (2026-09-30)
- status: locked
- decision: Sync handlers run before async enqueue; sync-only notify skips ref counting (stack-allocated events possible for sync).
- scope: events

## DEC-2026-09-30-3: Recyclable::Recycle() uses acq_rel
- source: docs/DESIGN.md section 8 (2026-09-30)
- status: locked
- decision: `Recyclable::Recycle()` uses `acq_rel` ordering for correct cross-thread release of event payloads.
- scope: memory, events

## DEC-2026-10-02-1: Zero locks, replica per thread, pub/sub only
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Zero locks; each thread owns a map replica; communication via pub/sub only.
- scope: architecture

## DEC-2026-10-02-2: Mapper is the only map writer, including loop closure logic
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Mapper is the only map writer including all loop closure logic; LoopCloser only detects.
- scope: Mapper, LoopCloser

## DEC-2026-10-02-3: Tracker optimizes only its own pose
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Tracker optimizes only its own pose and never mutates map objects; its replica changes only via Mapper diffs.
- scope: Tracker

## DEC-2026-10-02-4: MapPointId / FrameId never reused
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: `MapPointId` and `FrameId` are never reused (scope made precise by DEC-2026-10-05-1).
- scope: ids

## DEC-2026-10-05-1: Precise scope of the id rule
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: `FrameId` unique per run; `MapPointId` unique within a map and append-only (`MapPointStorage::Remove` only marks a point dead; free-list slot reuse removed; `num_ids()` = next id); point ids restart per map; `(map_id, id)` identifies a point. CLAUDE.md updated to match.
- scope: ids, MapPointStorage

## DEC-2026-10-02-5: Tracker lag between FrameTracked and resulting transaction accepted
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Tracker keeps tracking with its current replica between publishing and receiving the Mapper's result (originally between `NewKeyFrameCandidate` and diffs; since 2026-10-06 between `FrameTracked` and `KeyFrameCreated`). Matches ORB-SLAM3.
- scope: Tracker/Mapper lag

## DEC-2026-10-02-6: ObjectPool free list is a lock-free Treiber stack
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: `ObjectPool` free list is a lock-free Treiber stack with tagged head; links in a separate index array; verified with ThreadSanitizer.
- scope: memory

## DEC-2026-10-02-7: Observable::Notify() always takes ownership
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: `Observable::Notify()` always takes ownership of the event, regardless of wiring.
- scope: events

## DEC-2026-10-02-8: Pooled objects are kept alive; Create() takes no arguments
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Pooled objects are default-constructed once and kept alive; `Recycle()` does not destroy; reuse calls optional `Reset()`; `Create()` takes no arguments.
- scope: memory

## DEC-2026-10-02-9: Tracker has its own thread and queue
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Tracker has its own thread; `Track()` only enqueues.
- scope: Tracker

## DEC-2026-10-02-10: Tracker uses two queues
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Map diffs (inbox) drained first, then measurements; `Track()` takes the measurement by value.
- scope: Tracker

## DEC-2026-10-02-11: Measurement naming and timestamps
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: `CameraFrame` -> `MonocularMeasurement`, `BinocularCameraFrame` -> `BinocularMeasurement`; timestamp stored in the measurement (binocular uses left's); removed from `Frame` and keyframe events.
- scope: naming, measurements

## DEC-2026-10-02-12: Long-term goal of mixed mono/binocular keyframes
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Long-term goal is mixed monocular/binocular keyframes in one map; Tracker accepts both measurement types now; binding decided later (Open Question 17).
- scope: mixed maps

## DEC-2026-10-02-13: Configuration lives in the user application
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Configuration lives in the user application; reference app in `examples/full_slam/`. Library stays free of config-file parsing.
- scope: apps

## DEC-2026-10-03-1: images::TWorkingImage is 8-bit
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: `images::TWorkingImage` is 8-bit (`TImageU8Gray`), not float. The extractor interface (image in, features out) is the GPU boundary.
- scope: images, extractor

## DEC-2026-10-03-2: Long-term goal to drop OpenCV
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked (completed for the libraries on 2026-10-11, see DEC-2026-10-11-13)
- decision: Long-term goal: drop the OpenCV dependency.
- scope: dependencies

## DEC-2026-10-03-3: Link source-built OpenCV 5.1 from /usr/local
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked (narrowed: only dev tools/tests/benchmarks link OpenCV since 2026-10-11)
- decision: Link a source-built OpenCV 5.1 from `/usr/local` instead of Conan's OpenCV 4.12 (4.12 `cv::FAST` resizes output to 10 000 keypoints per call).
- scope: build, dev dependencies

## DEC-2026-10-03-4: Map viewer split into SceneState + MapViewer
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: Map viewer = `SceneState` (event-driven model) + `MapViewer` (own thread, single inbox, fixed-rate loop), attached by the apps, not by `Slam`. Emulates the future remote viewer.
- scope: viewer

## DEC-2026-10-03-5: Respect the event boundary now, serialization later
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: Keep the event boundary for the viewer now; serialization + transport (HTTP/ROS) later.
- scope: viewer, remote visualization

## DEC-2026-10-03-6: No OpenCV types in public headers; voyahviz reworked
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: No OpenCV types in public headers; OpenCV linked PRIVATE; APIs use our types (`TColorImage`, `Color`). Small libs may stay public. User requirement.
- scope: public API

## DEC-2026-10-03-7: Visualization in separate voyahviz library
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: Visualization in a separate `voyahviz` library (`vizualization/`), allowed to use OpenCV; images via `cv::imshow`, own keypoint drawing.
- scope: viewer

## DEC-2026-10-03-8: Drop features beyond 85 degrees off axis
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: Measurements drop features whose ray is > 85 degrees off the optical axis (`kMaxRayAngle`), tested in distorted space via `Distort()` of an 85-degree ray.
- scope: measurements, fisheye

## DEC-2026-10-02-14: Dataset reader is synchronous
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Dataset reader is synchronous (no thread); apps pace with `PlaybackClock` (absolute deadlines).
- scope: examples

## DEC-2026-10-02-15: ObjectPool::Create() aborts on exhaustion in every build
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked (superseded by DEC-2026-10-12-3: Create() now grows)
- decision: `ObjectPool::Create()` aborts on exhaustion in every build type instead of `assert`.
- scope: memory

## DEC-2026-10-02-16: Orchestrator named Slam<FE>; shared EventNode base
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Orchestrator is `Slam<FE>`; components share a non-virtual `EventNode` base for observable + inbox.
- scope: orchestration

## DEC-2026-10-02-17: One event type per header
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: One event type per header plus `events/events.h` umbrella; `map_events.h` removed.
- scope: events layout

## DEC-2026-10-02-18: Event payloads use grow-only ColumnBuffer
- source: docs/DESIGN.md section 8 (2026-10-02)
- status: locked
- decision: Event payloads use grow-only `ColumnBuffer` (capacity + count), not plain `Eigen::Matrix`, to reach zero-allocation steady state.
- scope: events, memory

## DEC-2026-10-03-9: Scope A for the first protocol (stubs)
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked (history: stubs replaced by real implementations in later steps)
- decision: First protocol delivered state machines + loops + initialization protocol with geometry behind rule-based stubs (`PoseEstimator`, `MapInitializer`).
- scope: history

## DEC-2026-10-03-10: Mono/stereo measurements travel as std::variant
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: Measurements travel as `std::variant` (`features::TMeasurement<FE>`) inside events instead of a template parameter (resolves OQ 24).
- scope: events, measurements

## DEC-2026-10-03-11: Idle wait via WakeSignal + WorkerThread
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: Per-component `WakeSignal` (atomic counter, C++20 wait/notify) + `WorkerThread`; snapshot taken before the `running` check (resolves OQ 20).
- scope: threading

## DEC-2026-10-03-12: Mapper performs initialization
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: The Mapper (only map writer) performs initialization from `InitializationMeasurement`s forwarded by the Tracker.
- scope: initialization

## DEC-2026-10-03-13: Maps identified by monotonic map_id (original rule)
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked (superseded by DEC-2026-10-10-6, the replica rule)
- decision: Maps identified by a monotonic `map_id` on every event; consumer rule: ignore older, reset-and-adopt newer, apply equal; 0 exempt.
- scope: map ids

## DEC-2026-10-03-14: State changes before publishing the announcing event
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: A component changes its state (and `current_map_id_`) before publishing the event that announces it.
- scope: state machines

## DEC-2026-10-03-15: Mono init needs many frames; stereo one
- source: docs/DESIGN.md section 8 (2026-10-03)
- status: locked
- decision: Monocular initialization needs many frames (reference + a later frame); stereo uses one measurement.
- scope: initialization

## DEC-2026-10-04-1: Essential solver 8-point now, 5-point later
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Normalized 8-point now, 5-point later (TODO, raised priority), behind an estimator interface returning several candidates per sample.
- scope: geometry

## DEC-2026-10-04-2: Generic Ransac<TEstimator> per model
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: One generic `Ransac<TEstimator>` per model (H, E), not ORB-SLAM3's shared sample loop.
- scope: geometry

## DEC-2026-10-04-3: Truncated chi-square scores with sigma in normalized units
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked (residual space later moved to raw pixels for the five consumers, DEC-2026-10-05-9; two-view RANSAC stays on rays)
- decision: ORB-SLAM3 truncated chi-square scores with sigma in normalized units (sigma_px / f).
- scope: geometry

## DEC-2026-10-04-4: RigidTransformation with T_ab convention
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: `RigidTransformation` with hidden representation and `T_ab` (b -> a) convention, so `T_ac = T_ab * T_bc`; `TPose` users migrate later.
- scope: geometry types

## DEC-2026-10-04-5: Local refinement in each estimator
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Optional `Refine` (Levenberg-Marquardt over inliers) in each estimator, 5 rounds in `Ransac`; LM counts only accepted steps (20 accepted / 40 trials).
- scope: geometry

## DEC-2026-10-04-6: H-vs-E selection uses the COLMAP rule
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Select homography when `inliers_H >= 0.8 x inliers_E` (COLMAP rule), not the ORB-SLAM3 score ratio.
- scope: initialization

## DEC-2026-10-04-7: Wide-FOV test scenes
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Recovery/accuracy tests use wide-FOV scenes (`half_fov_tan = 1.0`); narrow-FOV and near-frontal planar scenes are tested as rejected/weak.
- scope: tests

## DEC-2026-10-04-8: Accepted two-view accuracy limits
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Translation direction < 10 degrees on general scenes (max 9.92 over 21 seeds), rotation < 1 degree, recovery >= 50 % (narrow) / >= 75 % (wide, 30 % outliers), `num_triangulated >= 0.5 x true inliers`.
- scope: geometry accuracy

## DEC-2026-10-04-9: Keypoint pyramid level stored per feature
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked (SIFT parts superseded by DEC-2026-10-11-12)
- decision: `ImageFeatures` stores pyramid level per keypoint (`TLevels`, uint8); each extractor exposes `ScaleFactor(level)` (ORB `scale^level`).
- scope: features

## DEC-2026-10-04-10: Matcher max_distance is double
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Matcher `max_distance` is `double` (spec said `uint32`).
- scope: matchers

## DEC-2026-10-04-11: Orientation histogram 12 bins of 30 degrees
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Default 12 bins of 30 degrees (ORB-SLAM3's effective resolution), not 30 bins of 12 degrees.
- scope: matchers

## DEC-2026-10-04-12: Failed reconstruction keeps the reference
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: A failed reconstruction keeps the reference and the matcher's updated `last_matched` positions (ORB-SLAM3 `mvbPrevMatched`).
- scope: mono initialization

## DEC-2026-10-04-13: Zero triangulated points is an init failure
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Zero triangulated points and a non-finite median depth count as initialization failure.
- scope: mono initialization

## DEC-2026-10-04-14: Monocular initialization publication
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked (event names later replaced by `MapTransaction`)
- decision: ORB-SLAM3 matcher + two-view reconstruction, scaled to median depth 1; per-map point ids; keyframes store `T_cw`.
- scope: mono initialization

## DEC-2026-10-04-15: Pangolin SceneState decodes events
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked (the "newest map wins" part superseded by DEC-2026-10-10-6)
- decision: `SceneState` decodes events into a scene model; `PangolinRenderer` is private to voyahviz.
- scope: viewer

## DEC-2026-10-04-16: Stop() during RunOnCurrentThread() only requests stop
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: `Stop()` during `RunOnCurrentThread()` only requests the stop; the loop drains itself on its own thread.
- scope: viewer

## DEC-2026-10-04-17: Closing the window does not end the loop
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Closing the window does not end the loop; only `RequestStop()`/`Stop()` do. The loop keeps draining headless so producer pools never stall.
- scope: viewer

## DEC-2026-10-04-18: RequestStop() before loop entry is preserved
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: A `RequestStop()` made before the loop is entered is never lost.
- scope: viewer

## DEC-2026-10-04-19: Wayland workaround
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: On Linux under Wayland with `PANGOLIN_WINDOW_URI` unset, `MapViewer`'s constructor sets it to `x11://`.
- scope: viewer

## DEC-2026-10-04-20: Pangolin linked PRIVATE
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Pangolin linked `PRIVATE` via `pango_display`; no Pangolin types in public headers.
- scope: viewer, build

## DEC-2026-10-04-21: Window-creation failure counts as window_closed()
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Window-creation failure counts as `window_closed()`, so `--viewer` without a display stops playback after the first frame.
- scope: viewer, apps

## DEC-2026-10-04-22: MapViewer::RunOnCurrentThread() as macOS hedge
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: `RunOnCurrentThread()` besides `Start()`; not used by the apps yet; other threads end the run with `RequestStop()` only.
- scope: viewer

## DEC-2026-10-04-23: Apps --viewer behaviour
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: `--viewer` opens the window; playback stops when closed; after the last frame the app waits for it to be closed.
- scope: apps

## DEC-2026-10-04-24: Stereo matcher follows ORB-SLAM3 fisheye method
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Brute-force descriptor match (ratio 0.7) plus triangulation checks (positive depth, chi-square, parallax). Epipolar-guided search deferred.
- scope: stereo

## DEC-2026-10-04-25: Stereo matching on the Tracker thread
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Stereo matching runs on the Tracker thread at dequeue, before the measurement is tracked or forwarded.
- scope: stereo, Tracker

## DEC-2026-10-04-26: StereoRig::T_rl replaces R_rl/t_rl
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: `StereoRig::T_rl` (left -> right, `RigidTransformation`) replaces `R_rl`/`t_rl`; the dataset reader's inverse was fixed.
- scope: StereoRig

## DEC-2026-10-04-27: StereoMatches in own header
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: `StereoMatches` is in `features/stereo_matches.h` to avoid an include cycle.
- scope: layout

## DEC-2026-10-04-28: Stereo initialization
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: One identity keyframe; every valid depth becomes a metric map point (ORB-SLAM3); thresholds 500 features / 100 points.
- scope: stereo initialization

## DEC-2026-10-04-29: InitialMapPoint::feature_in_second = -1
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: `feature_in_second = -1` means a single-keyframe map.
- scope: initialization

## DEC-2026-10-04-30: Stereo observations stored per map in ObservationStorage<Stereo>
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked (superseded by DEC-2026-10-07-9: CovisibilityIndex is the Mapper's observation record)
- decision: Stereo observations in an `ObservationStorage<Stereo>` beside the mono one.
- scope: observations

## DEC-2026-10-04-31: Stereo depth capped near 17 m
- source: docs/DESIGN.md section 8 (2026-10-04)
- status: locked
- decision: Stereo depth capped at about 17 m by `max_parallax_cos = 0.99998` at 0.11 m baseline; a matcher option, tune later.
- scope: stereo

## DEC-2026-10-05-2: g2o vendored pristine as a local Conan package
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: g2o is vendored pristine in `third_party/g2o/src/`, packaged by a local Conan recipe, linked PRIVATE; public API stays on Eigen types.
- scope: dependencies

## DEC-2026-10-05-3: g2o package reference
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked (version bumped to 20241228.1 on 2026-10-10)
- decision: Local Conan package `g2o/20241228@voyah/stable` with `user`/`channel` set in the recipe.
- scope: dependencies

## DEC-2026-10-05-4: BundleAdjust on undistorted normalized points
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked (superseded by DEC-2026-10-05-9: raw-pixel residuals)
- decision: `BundleAdjust` works on undistorted normalized points (z = 1), sigma in normalized units; poses `T_cw`; analytic Jacobians.
- scope: optimization

## DEC-2026-10-05-5: Two-stage BundleAdjust
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Huber pass (5 iterations), deactivate edges with chi2 > 5.991 or depth <= 0, plain pass (default 10 iterations), like ORB-SLAM3 LocalBA.
- scope: optimization

## DEC-2026-10-05-6: Mono init runs two-keyframe BA
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: After two-view reconstruction, a two-keyframe BA runs; outliers dropped; fewer than 50 survivors fail the attempt.
- scope: mono initialization

## DEC-2026-10-05-7: Init-time error is noise-limited; tests check ML optimum
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Tests check convergence to the maximum-likelihood optimum (oracle from ground truth), not an absolute error bound. Mean error 2.11 -> 1.34 degrees with BA.
- scope: tests

## DEC-2026-10-05-8: Init BA 5 + 40 iterations on the unscaled problem
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Init BA uses 5 + 40 iterations on the unscaled problem (baseline 1), before the median-depth rescale (ORB-SLAM3 order).
- scope: mono initialization

## DEC-2026-10-05-10: Root CMake picks the concurrentqueue target
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Root `CMakeLists.txt` selects `VOYAHSLAM_CONCURRENTQUEUE_TARGET` between `moodycamel::concurrentqueue` and `concurrentqueue::concurrentqueue`.
- scope: build

## DEC-2026-10-05-11: Tracking against the map (sub-project 1)
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: `MapReplica` fed by Mapper events; motion model -> zero-motion fallback (window x4) -> local map -> `OptimizePose`; the `PoseEstimator` stub is deleted.
- scope: Tracker

## DEC-2026-10-05-12: Projection matcher takes the point source as template parameter
- source: docs/DESIGN.md section 8 (2026-10-05, ruling Q)
- status: locked
- decision: `features/` does not depend on `tracking/`; `PointReference` in `map/point_reference.h`; rotation histogram shared in `features/rotation_histogram.h`.
- scope: layering

## DEC-2026-10-05-13: Replica does not count events (ruling R)
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: The replica is a pure map copy and does not count events.
- scope: replica

## DEC-2026-10-05-14: OptimizePose schedule (ruling S)
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: 4 rounds x 10 iterations; every round restarts from the initial pose; Huber in rounds 0-1; chi2 5.991 mono / 9.488 stereo (4 dof); success >= 3 inliers.
- scope: Tracker optimization

## DEC-2026-10-05-15: Mono accuracy limited by noise-limited initial map (ruling T)
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Tests use 4 degrees parallax for the accuracy bound and a separate default-init robustness test; tuning deferred.
- scope: tests, tuning

## DEC-2026-10-05-16: Matcher deviations from ORB-SLAM3
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Histogram built from final claims; points without a reference observation skipped by local-map search; no scale-invariance distance check (predicted level clamped).
- scope: matchers

## DEC-2026-10-05-17: BoW-less fallback
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked (applies only without a vocabulary since 2026-10-08)
- decision: A wide-window (x4) zero-motion projection search replaces `TrackReferenceKeyFrame` when there is no vocabulary.
- scope: Tracker

## DEC-2026-10-05-18: Camera model approach A
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: `geometry::Camera{K, distortion}` value type (`Project`, `ProjectJacobian`, `Unproject`); not camera-model class hierarchy, not free functions.
- scope: camera model

## DEC-2026-10-05-9: Raw (distorted) pixel residuals for five consumers
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Projection matcher, `OptimizePose`, `BundleAdjust`, stereo matcher gate and initialization matcher windows compare raw pixels with sigma = `sigma_px * 1.2^level` px; chi2 thresholds unchanged (5.991 / 9.488). Rays stay rays (two-view RANSAC, triangulation, the 85-degree test). Supersedes the normalized-plane residual space.
- scope: camera model, optimization

## DEC-2026-10-05-19: Measurements keep distortion model pointer and raw keypoints
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked (ownership superseded by DEC-2026-10-12-1: shared_ptr)
- decision: Measurements keep both the distortion model pointer (`camera()`) and the raw keypoints (`distorted_points()`).
- scope: measurements

## DEC-2026-10-05-20: DistortionModel moved to geometry/distortion_models
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Moved from `features/distortion_models/` to `geometry/distortion_models/`; forwarding headers remain at the old path.
- scope: layering

## DEC-2026-10-05-21: Kannala-Brandt DistortJacobian (ruling U)
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Analytic `DistortJacobian`: J = (d/r) I + (d'(theta) theta'(r) - d/r) p-hat p-hat^T, theta'(r) = 1/(1 + r^2), identity for r < 1e-8; verified against finite differences.
- scope: camera model

## DEC-2026-10-05-22: Pinhole invariance only for fx = fy (ruling V)
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: Exact (1e-9) invariance with the old normalized code only for fx = fy; x and y now weighted separately in pixels.
- scope: optimization

## DEC-2026-10-05-23: Calibration-free angle allowance recorded, not implemented
- source: docs/DESIGN.md section 8 (2026-10-05)
- status: locked
- decision: sigma * 1.5 * (1 + r^2), window * (1 + r^2) is a fallback for lenses without a model; not implemented.
- scope: camera model

## DEC-2026-10-06-0: Camera-model smoke result
- source: docs/DESIGN.md section 8 (2026-10-05 row, smoke run 2026-10-06)
- status: locked
- decision: Mono 82 tracked / 18 re-initializations (criterion <= 5 missed, nothing tuned); stereo 197/200 local-map successes, no loss. Kept for more tracked frames (82 vs 56).
- scope: history, acceptance record

## DEC-2026-10-06-1: FrameTracked replaces PositionUpdated
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Every tracked measurement publishes pose, measurement, inlier pairs, visible points, reference Frame, local Frames, stage and counts. `PositionUpdated` deleted.
- scope: events

## DEC-2026-10-06-2: Mapper decides Frame creation
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Mapper decides via `ShouldCreateFrame` (ORB-SLAM3 rule and defaults); drains `FrameTracked` to the latest per map; skipped ones counted as backlog. Reverses the 2026-10-02 split.
- scope: Mapper/Tracker

## DEC-2026-10-06-3: NewKeyFrameCandidate deleted
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: No candidate protocol.
- scope: events

## DEC-2026-10-06-4: KeyFrameCreated as one atomic transaction
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked (generalized to `MapTransaction` by DEC-2026-10-07-1)
- decision: Frame, new points, observations, updated points in one event, used by initialization too; `MapPointsAdded` deleted.
- scope: events

## DEC-2026-10-06-5: Tracker local map from covisibility
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: K1 (Frames observing a match), K2 (top 10 covisible per K1 Frame), cap 80; local points are the local Frames' alive points.
- scope: Tracker

## DEC-2026-10-06-6: Triangulation epipolar test on rays, raw-pixel chi2
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Epipolar test on rays and raw-pixel chi2 (5.991, sigma = `sigma_px * level_scale^level`) in each Frame.
- scope: triangulation

## DEC-2026-10-06-7: Brute-force triangulation matching; Faiss vs DBoW2 deferred
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked (matching decision reversed by DEC-2026-10-07-5)
- decision: Matching for triangulation brute force for now.
- scope: matching

## DEC-2026-10-06-8: Rulings A, B, C on FrameTracked and options
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: (A) `reference_frame_id` is `std::optional<FrameId>`; fallback to newest Frame. (B) `sigma_px`/`level_scale` in `MapperOptions::triangulation`. (C) Mapper keeps its own `CovisibilityIndex` with Mapper-thread-only accessor.
- scope: Mapper

## DEC-2026-10-06-9: DistinctiveDescriptor rulings D, E
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Least median Hamming distance; even count takes mean of the two middle distances; ties to the lowest index.
- scope: descriptors

## DEC-2026-10-06-10: Epipolar band ruling F (spec amended)
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Exact directional 3-sigma bound `asin(|n.ray_b|) <= k*sigma*||(JJ^T)^-1 J n||` with J = `ProjectJacobian` at the unit ray.
- scope: triangulation

## DEC-2026-10-06-11: Scale consistency ruling G (spec amended)
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Compare dist_b / dist_a with level_scale^(level_a - level_b) (ORB-SLAM3 order).
- scope: triangulation

## DEC-2026-10-06-12: Stereo-matched observation counts 2 (ruling H)
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: For `ref_points`/`min_obs` a stereo-matched observation counts 2.
- scope: Frame rule

## DEC-2026-10-06-13: Later TrackingLost supersedes FrameTracked (ruling I)
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked (refined by DEC-2026-10-08-4: only a final loss supersedes)
- decision: A later `TrackingLost` of the same map in the drained batch supersedes a `FrameTracked`; all events keep order.
- scope: Mapper

## DEC-2026-10-06-14: Mono triangulation test on 4th Frame (ruling J)
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: The mono triangulation unit test triangulates on the 4th Frame.
- scope: tests

## DEC-2026-10-06-15: visible_points includes matched points; K2 takes top 10 of every K1 Frame (ruling K)
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: visible superset of matched; K2 top 10 neighbours of every K1 Frame.
- scope: Tracker local map

## DEC-2026-10-06-16: Reference Frame is the Frame sharing most inliers among all observers (ruling L)
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Reference Frame = Frame sharing the most inliers among all observing Frames, not only K1.
- scope: Tracker

## DEC-2026-10-06-17: Viewer colours
- source: docs/DESIGN.md section 8 (2026-10-06)
- status: locked
- decision: Tracked points green, visible blue, other grey; local-map Frames orange, other Frames blue.
- scope: viewer

## DEC-2026-10-07-0: Keyframes smoke result
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Mono 2 maps in 400 frames (first map 129 tracked); stereo lost once at ~360; both bars missed, nothing tuned; synthetic bounds loosened to 30 % / 0.3 m (later tightened by map quality).
- scope: history, acceptance record

## DEC-2026-10-07-1: Restructure into four libraries (approach C)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: `voyah_common`, `voyah_events`, `voyah_algs`, `voyah_slam` under `libs/<lib>/{include,src,tests}`, each with its own include root but unchanged `#include <voyahslam/...>` paths; `voyahslam` INTERFACE umbrella; `voyahviz` links only common + events; one test binary per library (label `unit`) plus `voyah_integration_tests` (label `integration`: every `SlamTest.*` and every test above 1 s); `check`/`check_all` targets; `VOYAHSLAM_OPTIMIZE_TESTS` (default ON, -O2, asserts on).
- scope: layout, build, tests

## DEC-2026-10-07-2: Every Mapper event is one MapTransaction
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: A single transaction type applied by every consumer in one fixed order (new map, add Frames, add points, add observations, update points, update poses, remove observations, remove points, remove Frames). Replaces `KeyFrameCreated`, `MapCreated`, `MapPointsUpdated`, `MapPointsRemoved`, `KeyFramePoseUpdated`.
- scope: events, Mapper

## DEC-2026-10-07-3: Two transactions per created Frame
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: T1 (tracked observations, recent-point culling, new points, fusion), T2 (local BA, Frame culling).
- scope: Mapper

## DEC-2026-10-07-4: Local BA in the Mapper thread, interrupted only by urgent FrameTracked
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Priority counter in the Mapper's inbox (incremented by `Push`, decremented only by `TryPop`/`Listener`). A separate BA thread was reserved for global BA (realized by the GlobalBA component).
- scope: Mapper, BA

## DEC-2026-10-07-6: BundleAdjust takes std::function<bool()> should_stop (rulings A, D, E)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Evaluated once before the first iteration and after every iteration; a stop after the last robust iteration but before outlier deactivation reports `stage_reached` 0 and keeps outliers; outliers removed only when `stage_reached >= 1`.
- scope: optimization

## DEC-2026-10-07-9: Mapper's ObservationStorages removed
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: The Mapper's `CovisibilityIndex` is its only observation record; stereo depth and right index come from the Frame's measurement (ruling G). Reverses the 2026-10-04 storage entry.
- scope: observations

## DEC-2026-10-07-10: Pass order as ORB-SLAM3 LocalMapping (ruling H)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Tracked observations, recent-point culling, new points, fusion. The spec's "culling before creation" was wrong.
- scope: Mapper

## DEC-2026-10-07-11: Every removed point is listed in removed_points (ruling C)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Including implicit removals (no observation left), so consumers without observation data (the viewer) keep no ghost points.
- scope: Mapper, events

## DEC-2026-10-07-12: Local BA rulings I, J, K
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: 20-Frame cap counts the new Frame; BA skipped with no free or no fixed Frame (ORB-SLAM3 gauge guard); sigma from `MapperOptions::fuse`.
- scope: local BA

## DEC-2026-10-07-13: Apps gain --trajectory_out; tools/ate.py
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: TUM format camera-to-world, one `# map <id>` section per map; `tools/ate.py` (pure Python, 20 ms association, Umeyama Sim3/SE3, largest section).
- scope: tools, apps

## DEC-2026-10-07-14: Map-quality smoke result
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Mono 400 keeps one map from frame 118, ATE Sim3 0.032-0.033 m; stereo 1000 one map, ATE SE3 0.036-0.037 m. Musts met; nothing tuned. Synthetic bounds tightened to 0.249 m mono / 0.037 m stereo.
- scope: acceptance record

## DEC-2026-10-07-5: Own OpenCV-free DBoW2 port
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: `bow/` in `voyah_algs`: `Vocabulary<FE>`, TF-IDF + L1 only, text loader and own checksummed binary format (`VOYBOW\0\1`, FNV-1a 64), `tools/convert_vocabulary`; loaders never throw. Reverses the 2026-10-06 "Faiss vs DBoW2/FBoW deferred" entry.
- scope: BoW

## DEC-2026-10-07-15: Vocabulary<FE> compiles for every INSTANTIATE_CLASS extractor
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked (ORB only since SIFT removal)
- decision: Loaders reject a descriptor-size mismatch at run time.
- scope: BoW

## DEC-2026-10-07-16: FeatureVector filing at depth L - levels_up (ruling B)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: A leaf shallower than that files under itself (DBoW2 leaves it uninitialised, UB).
- scope: BoW

## DEC-2026-10-07-17: Text loader ignores trailing blank lines (ruling D)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: No phantom node (DBoW2 creates one: 971 815 vs 971 814 words).
- scope: BoW

## DEC-2026-10-07-18: ctest label `vocabulary`
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: `voyah_vocabulary_tests` excluded from `check`, run by `check_all`, skips unless `$VOYAHSLAM_ORBVOC` is set. The 145 MB vocabulary is never committed.
- scope: tests

## DEC-2026-10-07-19: Vocabulary loaded once by the app and shared immutable
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: `--vocabulary <file.bin>`; shared as `std::shared_ptr<const bow::Vocabulary<FE>>` through `Slam`; null means no BoW and the old behaviour.
- scope: BoW, apps

## DEC-2026-10-07-20: Mapper KeyFrameRecord keeps BoW privately (ruling G)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked (superseded by DEC-2026-10-08-3: BoW now travels in MapTransaction)
- decision: BoW (levels_up 4) computed for every Frame and never published.
- scope: BoW

## DEC-2026-10-07-21: SearchForTriangulation optionally uses SearchByBoW (rulings F, H)
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Candidates only from shared-node pairs, all other checks unchanged; a one-node vocabulary reproduces brute force exactly.
- scope: triangulation

## DEC-2026-10-07-22: ORB rotated BRIEF offsets rounded with std::lround
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Bit-compatibility with ORB-SLAM3's vocabulary (`cvRound`).
- scope: ORB extractor

## DEC-2026-10-07-23: BoW smoke result
- source: docs/DESIGN.md section 8 (2026-10-07)
- status: locked
- decision: Triangulation 26-28x faster mono, 39x stereo; 3x Must met; stereo ATE regressed (0.056 / 0.189 m) and an extra run lost tracking at frame 325; nothing tuned (cause addressed by DEC-2026-10-08-1).
- scope: acceptance record

## DEC-2026-10-08-1: Motion-model inlier gate
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: `min_motion_model_inliers` = 30; fewer counts as failed and zero motion (minimum 10) runs next. Stereo 1000 + vocabulary ATE 0.036-0.037 m.
- scope: Tracker

## DEC-2026-10-08-2: Loss protocol (RECENTLY_LOST)
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: With a vocabulary and a map of more than 10 Frames a failure enters `RecentlyLost` (`TrackingLost(final = false)`, nothing published, Mapper keeps its map) and relocalizes every measurement for 5 s of measurement timestamps, then `Lost` (`final = true`, the only one that clears the map). Without a vocabulary or on a small map: straight to `Lost`.
- scope: Tracker, Mapper

## DEC-2026-10-08-4: Non-final TrackingLost does not supersede FrameTracked (ruling H)
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: A non-final `TrackingLost` does not supersede an earlier `FrameTracked` in the Mapper's drained batch; a final one does.
- scope: Mapper

## DEC-2026-10-08-5: TrackingStage names the initial-pose source (ruling C)
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: `MotionModel`/`ZeroMotion`/`ReferenceFrame`/`Relocalization`; `LocalMap` removed.
- scope: events

## DEC-2026-10-08-3: Every added Frame carries BowVector/FeatureVector; Tracker keeps FrameDatabase
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: Reverses the 2026-10-07 Mapper-only entry. The Tracker's replica stores them and keeps its own `bow::FrameDatabase`; vocabulary reaches the Tracker through `Slam`.
- scope: BoW, replicas

## DEC-2026-10-08-6: bow_vector.h moved to voyah_common (rulings D, A)
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: Same include path, free `bow::Score`; `FrameDatabase` not templated.
- scope: layering

## DEC-2026-10-08-7: bow::FrameDatabase per replica
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: Inverted index owned by each replica, synced by transactions; `Query` keeps common >= 0.8 x max; candidates grouped by top 10 covisible, kept at >= 0.75 x best group.
- scope: BoW

## DEC-2026-10-08-8: Reference-Frame tracking after zero motion
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: `SearchByBoW` ratio 0.7, >= 15 matches, >= 10 inliers, then the local map; values in `TrackingOptions`.
- scope: Tracker

## DEC-2026-10-08-9: geometry::SolvePnPRansac
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: Lambda Twist P3P on unit bearings, RANSAC 300 / 0.99 / >= 10, raw-pixel chi2 5.991 inliers; no OpenCV, no EPnP. MLPnP kept as an alternative (TODO).
- scope: relocalization

## DEC-2026-10-08-10: Relocalization refines with OptimizePose over all matches (rulings E, F, G)
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: `SolvePnPRansac` is a raw hypothesis generator; PnP tests assert over seeded scene sets (200 half-outlier scenes >= 95 %; 50 fisheye seeds at 1e-5).
- scope: relocalization, tests

## DEC-2026-10-08-11: Relocalization projection passes use SearchFromFrame (ruling I)
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: ORB-exact `SearchByProjection(Frame, KeyFrame)`: radius `th * 1.2^level` px, no ratio test, levels [l-1, l+1], orientation check; `th`/distance 10/100 then 3/64.
- scope: relocalization

## DEC-2026-10-08-12: Post-relocalization guard (ruling J)
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: For 1 s of measurement timestamps after a relocalization the local-map gate is 50 inliers; the Mapper creates no Frame from a `FrameTracked` within `relocalization_guard_seconds` (1.0) of the latest stage-`Relocalization` one while its map has more than 20 Frames; a map clear or new map resets it.
- scope: relocalization

## DEC-2026-10-08-13: Apps gain --drop_frames a:b
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: Featureless measurements at the frames' own timestamps for reproducible blackouts.
- scope: apps

## DEC-2026-10-08-14: Relocalization smoke result
- source: docs/DESIGN.md section 8 (2026-10-08)
- status: locked
- decision: Mono recovers at 271, stereo at 521/524 into map 1; ATE unchanged; all Musts met; about 10 ms per attempt.
- scope: acceptance record

## DEC-2026-10-09-1: Loop closing architecture (steps 4a+4b)
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: LoopCloser detects on its own replica (ORB-SLAM3 `NewDetectCommonRegions`) and publishes `LoopDetected`; Mapper corrects (`CorrectLoop`) as one `MapTransaction` at `map_epoch + 1`; Tracker shifts `last_frame_` with its reference Frame.
- scope: loop closing

## DEC-2026-10-09-2: geometry::Sim3 plain data in voyah_common (ruling A)
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Inline `Inverse`/`Compose`/`Apply`; Horn alignment and `SolveSim3Ransac` in `voyah_algs`.
- scope: layering

## DEC-2026-10-09-3: Spanning tree rule (ruling B)
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: A new Frame sharing no points takes the previous newest Frame as parent; a culled Frame's children are re-parented by live shared-point counts, lower-id ties.
- scope: map graph

## DEC-2026-10-09-4: SolveSim3Ransac adaptive iterations, best hypothesis
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Cap 300, confidence 0.99, best hypothesis kept (stricter than ORB-SLAM3's first hypothesis over the inlier count).
- scope: loop closing

## DEC-2026-10-09-5: Sim3 projection matchers compare observing feature descriptor/level
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Not the map point's distinctive descriptor (interface has none; TODO).
- scope: loop closing

## DEC-2026-10-09-6: EssentialGraphProblem splits edges from loop_connections
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Loop connections keep every weight; covisibility edges need >= 100 shared points and exclude parent/child; the match Frame is the fixed vertex.
- scope: loop closing

## DEC-2026-10-09-7: Loop thresholds follow ORB-SLAM3 source
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: RANSAC inliers 15, `OptimizeSim3` 20, projection 80, coarse projection 50, region dropped after 2 failed later queries; `S_query_match` is ORB Scm (x_query = S * x_match).
- scope: loop closing

## DEC-2026-10-09-8: Detection step 4 follows ORB
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: After RANSAC, candidate window points are projected into the query through the RANSAC Sim3 into a fresh match list (th 8, ratio 1.5, >= 50) before `OptimizeSim3`; match-window widening skipped.
- scope: loop closing

## DEC-2026-10-09-9: LoopCloser does not suppress duplicate loops; not wired to TrackingLost
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Mapper ignores a closed loop and another map's loop; features without points dropped from `matched_points`.
- scope: loop closing

## DEC-2026-10-09-10: Mono correction does not rescale point reference distances
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Following ORB `UpdateNormalAndDepth`; middle Frames' rotation error under identity-weighted essential graph accepted.
- scope: loop closing

## DEC-2026-10-09-11: LoopCloserOptions::enabled
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked (default changed to false by DEC-2026-10-10-1; `--no_loop_closing` replaced by `--loop_closing`)
- decision: Disabled, the LoopCloser behaves as without a vocabulary.
- scope: loop closing

## DEC-2026-10-09-12: Mapper counters and lock-step harness
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: `num_frame_evaluations()` counts a `FrameTracked` once its Frame is published and refined; `stale_epoch_skipped()`; integration `RunSetup::lock_step` takes Tracker and Mapper out of thread timing.
- scope: tests

## DEC-2026-10-09-13: SlamTest.{Mono,Stereo}ClosesLoop
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Synthetic outward-looking loop (450 degrees at 1.5 deg/frame) with place words; end error with loop closing <= 1/3 of without; no tracking loss across the correction.
- scope: tests

## DEC-2026-10-09-14: Fidelity diagnosis before fixing corridor1 false loops
- source: docs/DESIGN.md section 8 (2026-10-09, Task 8 ruling)
- status: locked
- decision: Diagnosis (`.superpowers/sdd/2026-10-09-loop-closing/false-loop-diagnosis.md`) found the weakness inherent to ORB-SLAM3's gates; gates must be extended, not fixed for fidelity.
- scope: loop closing

## DEC-2026-10-09-15: [USER DECISION] Normal-consistency gate
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: LoopCloser rejects a candidate/confirmation/refinement when more than `max_normal_disagreement` (0.5) of final matched pairs have normals more than 60 degrees apart; deliberate deviation from ORB-SLAM3. Not addressed: same-heading look-alikes; verify-before-publish rejected for the first round.
- scope: loop closing

## DEC-2026-10-09-16: Loop-closing smoke result
- source: docs/DESIGN.md section 8 (2026-10-09)
- status: locked
- decision: Musts MISSED on corridor1 (same-heading false loops); nothing tuned; left for user decision (TODO).
- scope: acceptance record

## DEC-2026-10-10-1: [USER DECISION] Loop closing is opt-in and off by default
- source: docs/DESIGN.md section 8 (2026-10-10); docs/ROADMAP.md step 4
- status: locked
- decision: `LoopCloserOptions::enabled` defaults to false; apps use opt-in `--loop_closing`; relocalization and BoW triangulation keep using the vocabulary. Branch merged as "implemented, opt-in". Next: look-alike rejection as first item of step 6.
- scope: loop closing, apps

## DEC-2026-10-10-2: LoopCloser applies transactions on receipt; query backlog dropped
- source: docs/DESIGN.md section 8 (2026-10-10, final-review fix wave)
- status: locked
- decision: Queues only new Frames' ids; `ProcessPending()` detects one query per call; beyond `max_query_backlog` (8) oldest queries dropped, counted, logged. Mapper never drops transactions. A raised map epoch drops the pending region.
- scope: LoopCloser

## DEC-2026-10-10-3: g2o recipe version 20241228.1
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: Version = upstream tag + recipe revision, bumped whenever the recipe changes what it builds.
- scope: dependencies

## DEC-2026-10-10-4: Global BA merge (step 4c)
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: `GlobalBAFinished` merged only in `Mapping`, of the current map and epoch; breadth-first spanning-tree walk, parents first; moved points' normals recomputed; one `MapTransaction` at `map_epoch + 1`, never with a loop edge.
- scope: global BA

## DEC-2026-10-10-5: Global BA wiring
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: `Slam` takes `GlobalBaOptions` after `LoopCloserOptions`; `GlobalBundleAdjuster` held as `std::optional`, created only with loop closing in effect (also a vocabulary); apps take `--no_global_ba`.
- scope: global BA

## DEC-2026-10-10-7: Global BA integration bars per mode
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: Stereo points RMS <= 0.8x the essential-graph-only run, end error <= 1.0x; mono (revised after measurement) points RMS <= 0.95x per seed and <= 0.92x average, end error <= 1.05x per seed.
- scope: tests, acceptance

## DEC-2026-10-10-8: [USER DECISION] GlobalBA owns a replica fed from the start; no late joining
- source: docs/DESIGN.md section 8 (2026-10-10); docs/ROADMAP.md step 4c
- status: locked
- decision: A component either exists from the start or not at all.
- scope: global BA, components

## DEC-2026-10-10-9: [USER DECISION] Components stay pluggable through gRPC later
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: GlobalBA (and LoopCloser) talk to the Mapper only through events of plain data (ids, poses, positions; no pointers into map objects).
- scope: components, events

## DEC-2026-10-10-10: GlobalBaOptions keeps its own sigma_px / level_scale
- source: docs/DESIGN.md section 8 (2026-10-10, pre-flight ruling)
- status: locked
- decision: Same defaults as `MapperOptions::fuse`; costs a third sigma source (TODO).
- scope: global BA

## DEC-2026-10-10-11: Global BA gauge
- source: docs/DESIGN.md section 8 (2026-10-10, Task 1 ruling)
- status: locked (refined by DEC-2026-10-11-3: root Frame is the gauge)
- decision: Oldest surviving Frame fixed even without observations.
- scope: global BA

## DEC-2026-10-10-12: BA observation helpers moved to mapping::detail
- source: docs/DESIGN.md section 8 (2026-10-10, Task 1 ruling)
- status: locked
- decision: `BaStereoOf`, `BaObservationOf` in `mapper_internal.h`, shared by local and global BA.
- scope: BA

## DEC-2026-10-10-13: Global BA apply-on-poll and snapshot contract (Task 2 rulings)
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: Each poll drains the inbox, judges every transaction for interruption against the run's `(map_id, epoch)`, applies and releases it; `RunGlobalBa` builds a self-contained problem before its first poll and never reads the replica again. Supersedes the first backlog-based version.
- scope: global BA

## DEC-2026-10-10-14: Test-only seams on GlobalBundleAdjuster
- source: docs/DESIGN.md section 8 (2026-10-10, Task 2)
- status: locked
- decision: `stop_requested()`, `set_run_hook()`, `set_poll_hook()`.
- scope: tests

## DEC-2026-10-10-15: Global BA Task 3 rulings
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: Merge recomputes moved points' normals; a `GlobalBAFinished` is also rejected when the Mapper is not `Mapping`.
- scope: global BA

## DEC-2026-10-10-16: Global-BA merge drops LoopCloser pending region (M4)
- source: docs/DESIGN.md section 8 (2026-10-10, final review)
- status: locked
- decision: A merge raises the map epoch (deviation from ORB-SLAM3); `Publish` skipped if `Stop()` requested after the run's last poll.
- scope: global BA

## DEC-2026-10-10-17: Slam creates GlobalBA only with loop closing in effect
- source: docs/DESIGN.md section 8 (2026-10-10, Task 5 review minor)
- status: locked
- decision: Vocabulary as well as `LoopCloserOptions::enabled` and `GlobalBaOptions::enabled`; apps print "Global BA: on/off" from `slam.global_ba()`.
- scope: global BA

## DEC-2026-10-10-18: Global BA smoke result
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: No crash, no tracking loss caused by a merge (Musts met); peak RSS stereo 4.23 GB with vs 2.28 GB without; map-loss counts not separable from loop variance; ATE not comparable.
- scope: acceptance record

## DEC-2026-10-10-19: Measurements shared and immutable (map merging Task 1)
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: `TSharedMeasurement<FE> = std::shared_ptr<const TMeasurement<FE>>` in `FrameTracked`, `NewFrame`, Mapper `Frame`, every `ReplicaKeyFrame`; created once per tracked frame; pooled events release in `Reset()`.
- scope: measurements, memory

## DEC-2026-10-10-6: The replica rule replaces "newest map id wins" (Task 2)
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: `MapTransaction` gains `active_map`, `removed_maps`, `absorbed_map`; `tracking::AtlasReplica<FE>` holds a `MapReplica` per live map; unknown/removed maps ignored; active map taken from `active_map` only.
- scope: replicas, Atlas

## DEC-2026-10-10-20: The Mapper's Atlas (Task 3)
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: Per-map state in private `MapState` in `std::map<MapId, MapState>` with `std::optional<MapId> active_`; final loss with fewer than `atlas.min_frames_to_keep` (10) Frames -> discarded (`removed_maps`), otherwise kept inactive. Inactive maps frozen.
- scope: Mapper, Atlas

## DEC-2026-10-10-21: Merge detection in the LoopCloser (Task 4)
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: `DetectNBestCandidates(atlas, ...)` queries every live map's `FrameDatabase`, splits into loop and merge candidates (best 3 each); merge verified against the inactive map's replica; a merge at the same query as a loop wins; merge not-found counter resets on confirmation (deviation).
- scope: merge detection

## DEC-2026-10-10-22: Normal-consistency gate not applied to merges (Task 4, fix round 1)
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: Cross-map normals are meaningless; share still measured and logged; loops keep the gate. Merging stays opt-in; hypothesis-independent check is a TODO.
- scope: merge detection

## DEC-2026-10-11-1: The merge absorbs first (Task 5)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: M moved into N by the id rule before correction steps; essential graph of step 6 holds only M's Frames with the welded window fixed (`also_fixed`); graph also runs for mono; `LoopDetected` pairs fused; N's recent points dropped; viewer's `SceneState` absorbs too.
- scope: map merging

## DEC-2026-10-11-2: Merges chain (Task 5, fix round 1)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: `MapState` slots Frames by id from `first_slot` and keeps the spanning tree `root` separately; `RootKeyFrame()` is the never-culled local-BA gauge.
- scope: map merging

## DEC-2026-10-11-3: DetectNBestCandidates group scoring counts only filtered hits (Task 4, fix round 2)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: Group score adds covisible Frames' scores only for those passing the common-words filter; `matched_points` drops self-pairs only for a loop. The global BA gauge is the root Frame (oldest Frame without a parent).
- scope: loop closing, merging

## DEC-2026-10-11-4: The Tracker follows a merge (Task 6)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: Keeps state, velocity and reference Frame; shifts `last_frame_` through loop-correction anchors; translates matches with `AtlasReplica::TranslatePoint`. Any other active-map change resets (`AdoptMap`).
- scope: Tracker, merging

## DEC-2026-10-11-5: A Lost Tracker re-announces its loss after a merge (Task 6, controller ruling)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: Publishes a final `TrackingLost(N)`; the Mapper needs no change.
- scope: Tracker, merging

## DEC-2026-10-10-23: [USER DECISION] Map merging scope A, ORB-SLAM3 parity
- source: docs/DESIGN.md section 8 (2026-10-10); docs/ROADMAP.md step 5
- status: locked
- decision: Keep lost maps of at least 10 Frames inactive; detect revisits of inactive maps in the LoopCloser; merge the active map into the inactive one (`MergeLocal`, visual path); relocalization stays active-map-only; every replica mirrors the whole Atlas with shared measurements; the target map survives and absorbs the active map (id rule); merging is part of loop closing (opt-in).
- scope: map merging

## DEC-2026-10-10-24: Map merging controller rulings
- source: docs/DESIGN.md section 8 (2026-10-10)
- status: locked
- decision: `AtlasReplica::TranslatePoint(absorbed, old)`; pooled events release their measurement when the slot is reused; `Mapper::KeyFrame(FrameId) const` accessor; `AtlasReplica::active_or_empty()`.
- scope: map merging

## DEC-2026-10-11-6: "A correction or merge is running" read as queue order (Task 5 controller ruling)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: A merge queued behind a loop correction of M is accepted (camera-to-camera Sim3 is world-independent); rejecting stale detections would need an epoch on `LoopDetected` (TODO).
- scope: map merging

## DEC-2026-10-11-7: Merge integration and harness (Task 7)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: `SlamTest.{Mono,Stereo}MergesMaps` use a 600-degree loop (401 frames) with blackout 140-165 and `recently_lost_seconds` 1 s; whole-run error = RMS of every final-map Frame centre under one Umeyama alignment; lock-step waits on `Tracker::num_processed()` and `Mapper::num_initialization_measurements()`; apps print the Atlas; `tools/ate.py --map ID`.
- scope: tests, tools

## DEC-2026-10-11-8: Lost maps kept only when Slam has a LoopCloser (final review I1)
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: Otherwise `atlas.min_frames_to_keep = SIZE_MAX` so every lost map is discarded.
- scope: Atlas

## DEC-2026-10-11-9: Brown-Conrady distortion model naming
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: `BrownConradyDistortionModel` (OpenCV order `[k1, k2, p1, p2, k3]`) = OpenCV standard = ROS `plumb_bob` = Kalibr `radtan` = EuRoC `radial-tangential`; `Undistort` is Newton (1e-12, <= 20 iterations); failures left unchanged; tests may use OpenCV as reference, the library may not.
- scope: camera model

## DEC-2026-10-11-10: FAST ported from OpenCV, bit-exact
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: `features::DetectFast` is OpenCV 5's `FAST_t<16>` + `cornerScore<16>` + AVX2 kernel, keeping the Apache-2.0 and Rosten BSD notices; ORB output byte-identical. [USER DECISIONS: own FAST to drop OpenCV; bit-exact.]
- scope: ORB extractor

## DEC-2026-10-11-11: FAST SIMD paths
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: Scalar, SSE2, AVX2 (run-time via `__builtin_cpu_supports`), NEON; each tested against live `cv::FAST`; speed target (user decision): at least as fast as `cv::FAST` single-threaded (measured 0.90-0.97).
- scope: ORB extractor

## DEC-2026-10-11-12: [USER DECISION] SIFT extractor removed
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: Header, source, tests and every instantiation removed; supersedes SIFT parts of earlier rows.
- scope: extractors

## DEC-2026-10-11-13: voyah_* libraries are OpenCV-free
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked
- decision: `voyah_algs` no longer links OpenCV; tests/benchmarks that compare with OpenCV link it themselves; `voyahviz` keeps it PRIVATE; `voyah_algs_dataset_tests` for corridor1 tests; `fast_benchmark` not in `check`/`check_all`.
- scope: dependencies

## DEC-2026-10-11-14: Conan package and build options
- source: docs/DESIGN.md section 8 (2026-10-11)
- status: locked (version history: 0.1.0 -> 0.2.0 -> 0.3.0)
- decision: `conanfile.py` recipe; `VOYAHSLAM_BUILD_TESTS`/`VOYAHSLAM_BUILD_EXAMPLES` OFF by default; four libraries + umbrella installed/exported as `voyahslam::*`; `validate()` requires C++20; `profiles/linux-gcc13`; Android `profiles/android-arm64` (API 24, clang 21, NDK r30) with `compiler.libcxx=c++_static` [USER DECISION]; c++_static safe only while the app has exactly one shared library containing C++.
- scope: packaging, Android

## DEC-2026-10-12-1: Distortion models are shared, immutable (TDistortionModelPtr)
- source: docs/DESIGN.md section 8 (2026-10-12)
- status: locked
- decision: [USER DECISION] `TDistortionModelPtr = std::shared_ptr<const DistortionModel>` in `geometry::Camera`, `StereoRig`, measurement constructors and the example reader; null still means pinhole; functions that only use a model during a call keep `const DistortionModel*`/`&`; hot-path g2o edges hold `const Camera&` (no copies); measured within noise.
- scope: ownership, camera model

## DEC-2026-10-12-2: Package version 0.2.0
- source: docs/DESIGN.md section 8 (2026-10-12)
- status: locked (superseded by 0.3.0)
- decision: API break (public `Camera`, `StereoRig`, measurement constructors) gets a new minor version while 0.x.
- scope: versioning

## DEC-2026-10-12-3: Object pool grows on overflow; MapTransaction must never be dropped
- source: docs/DESIGN.md section 8 (2026-10-12)
- status: locked
- decision: `ObjectPool::Create()` no longer aborts: adds a chunk doubling capacity, logs one warning per growth, counts `num_overflow_created()`; overflow objects kept; `TryCreate()` never grows (LoopCloser/GlobalBA still drop on purpose); Tracker/Mapper pools sized by `TrackerOptions::event_pool_size`/`MapperOptions::transaction_pool_size` (default 256); lossy delivery for display-only observers deferred (TODO).
- scope: memory, events

## DEC-2026-10-12-4: Package version 0.3.0
- source: docs/DESIGN.md section 8 (2026-10-12)
- status: locked
- decision: `ObjectPool` constructor gains a name, `Create()` semantics change, options gain pool-size fields; `conanfile.py` and `project()`; SOVERSION 0.3.
- scope: versioning

---

## Project rules from CLAUDE.md (repo-level, binding)

## RULE-NAMING: Google-style naming
- source: CLAUDE.md; docs/DESIGN.md section 7
- status: locked
- decision: `PascalCase` types/functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference output parameters, `kPascalCase` constants, `T` prefix for template parameters and aliases, `enum class`, namespaces `voyahslam::<module>`.
- scope: coding conventions

## RULE-LAYERING: Four-library layering
- source: CLAUDE.md; docs/DESIGN.md section 7
- status: locked
- decision: `voyah_common`; `voyah_events` and `voyah_algs` each on common only; `voyah_slam` on all three; `voyahslam` umbrella. A lower layer cannot include an upper one (compile error by include-path design).
- scope: build layout

## RULE-DOCS-LIVING: DESIGN.md is kept current
- source: CLAUDE.md; docs/DESIGN.md header
- status: locked
- decision: Whenever a design decision is made or changed, record it in DESIGN.md (decisions log + relevant section) in the same change; when specs disagree with DESIGN.md, DESIGN.md wins. Deferred work goes to `docs/TODO.md`; ordered next steps in `docs/ROADMAP.md`.
- scope: process

## RULE-WORKFLOW: Spec -> plan -> task-by-task implementation
- source: docs/DESIGN.md section 7 "Workflow"; docs/ROADMAP.md header
- status: locked
- decision: brainstorm -> spec -> plan -> implement task-by-task with tests on a feature branch -> review -> merge; nothing is merged without an explicit "merge and push".
- scope: process
