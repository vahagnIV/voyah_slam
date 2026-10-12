# voyahslam — Living Design Document

> **Purpose:** the single source of truth for *what* we are building and *why*.
> Update this file whenever a design decision is made or changed. A new session
> (human or Claude) should be able to read this file and continue the work.
>
> Detailed per-feature specs and implementation plans live in
> `docs/superpowers/specs/` and `docs/superpowers/plans/`. When they disagree
> with this document, **this document wins** — fix the spec or note the change here.

Last updated: 2026-10-08

---

## 1. Goal

An ORB-SLAM3-inspired visual SLAM library (mono + stereo) with a different
concurrency model: **no locks anywhere on the map**. Threads never share mutable
state; they communicate exclusively through publish/subscribe events over
lock-free queues, and every thread keeps its own replica of the map.

Predecessor project: `../slamlab2`. Image processing, feature extraction,
geometry, distortion models etc. were ported from there.

**Long-term goal beyond ORB-SLAM3:** *mixed* maps — monocular and binocular
keyframes in the same map (e.g. a stereo rig whose second camera drops out, or
merging mono and stereo sessions). Data structures should not hard-wire one
sensor type per map. For now the Tracker accepts both measurement types; how
keyframes of different types share the map is still open (Open Question 17).

## 2. Core Principles

1. **No locks.** No mutex anywhere on the map or event path. Cross-thread
   communication only via events on `moodycamel::ConcurrentQueue`; event pools
   are lock-free (§6.1).
2. **One replica of the map per thread.** Tracker, Mapper, LoopCloser (and later
   the Visualizer) each own a local copy: `MapPointStorage`, `Frame`s and a
   `CovisibilityIndex` as the observation record (since 2026-10-07; before,
   `ObservationStorage`).
3. **Mapper is the single writer.** *All* changes to map objects — creating,
   updating, culling, fusing map points; creating keyframes; adjusting keyframe
   poses; closing loops — happen in the Mapper. The Mapper then publishes diff
   events and every other replica applies them verbatim.
4. **Other threads are read-only on the map.** They may compute things *from*
   their replica (a pose, a loop candidate) and publish those as requests or
   facts, but they never mutate map objects themselves.
5. **IDs are never reused.** A `FrameId` is unique for the entire run; a
   `MapPointId` is unique within its map (point storage restarts per map), so
   `(map_id, id)` identifies a point. Within a map a stale ID in an event from
   a lagging replica is therefore always detectable (the slot is invalid) and
   can never alias a newer object.
6. **No virtual dispatch on hot paths.** Events dispatch on a type tag +
   `static_cast`; recycling goes through a function pointer. Mono/stereo is chosen
   at compile time via templates.
7. **Structure-of-Arrays storage** for map points and observations, so the hot
   loops (projection, matching, BA) run over contiguous Eigen matrices.
8. **OpenCV is an implementation detail.** Our public headers
   (`libs/<lib>/include/voyahslam/`, `vizualization/include/voyahviz/`) never include
   OpenCV; public APIs take and return our types (`images::Image`,
   measurements, `ImageFeatures`, ...) and OpenCV is linked `PRIVATE`, so a
   user of the library never needs its headers or `find_package(OpenCV)`.
   **Since 2026-10-11 the four `voyah_*` libraries use no OpenCV at all**
   (FAST ported, SIFT removed; see "FAST Corner Detector"): only `voyahviz`
   (dev tool), tests and benchmarks link it, the latter as a reference.
   Small dependencies (Eigen, spdlog, concurrentqueue) may appear in public
   headers — requiring users to have them is fine.

## 3. Terminology

| voyahslam | ORB-SLAM3 | Notes |
|---|---|---|
| `Frame` | KeyFrame | Only keyframes become `Frame` objects. |
| current tracking state | Frame | Lives privately inside the Tracker; never published as a map object. |
| `MapPointStorage` | set of MapPoints | SoA, indexed by `MapPointId`. |
| `CovisibilityIndex` | MapPoint↔KeyFrame observations | Per point its observing Frames (with feature index), per Frame its points; the Mapper's and the replicas' observation record (2026-10-07). |
| `MonocularMeasurement` / `BinocularMeasurement` | Frame (features part) | Features of one image / a stereo pair at a timestamp. Input to `Track()`. |
| `StereoRig` | camera calibration | Immutable, shared by all threads. |

## 4. Threads and Responsibilities

```
               FrameTracked                     LoopDetected
 Tracker ───────────────────────────► Mapper ◄────────────── LoopCloser
    ▲                                 ▲ │                        ▲
    │      map diff events            │ │   map diff events      │
    └─────────────────────────────────┼─┼────────────────────────┤
                                      │ │                        ▼
                     GlobalBAFinished └─┼──────────────── GlobalBA (with loop closing)
                                        ▼   map diffs, FrameTracked (from Tracker)
                                   Visualizer (MapViewer)
```

### Slam (orchestrator) and EventNode

`slam::Slam<TFeatureExtractor>` (`slam/slam.h`) owns the three components,
plus the GlobalBA when loop closing is in effect (see "Global BA"), and
wires them asynchronously in its constructor:

| Publisher | → Async observers |
|---|---|
| Tracker | Mapper |
| Mapper | Tracker, LoopCloser, GlobalBA |
| LoopCloser | Mapper |
| GlobalBA | Mapper |

`Start()` starts consumers before producers (Mapper, GlobalBA, LoopCloser,
Tracker); `Stop()` stops producers first (Tracker, LoopCloser, GlobalBA,
Mapper); the destructor calls `Stop()`. `Slam` is neither copyable nor movable — components hold
pointers to each other's inboxes. Extra observers (visualizer, logging) attach
via `slam.tracker()/mapper()/loop_closer()/global_ba()->AddObserver(...)`
before `Start()` (`global_ba()` is null without a GlobalBA).

Every component derives from `events::EventNode` (`events/event_node.h`, no
virtuals): an `Observable` it publishes through, one `AsyncObserver` inbox it
consumes, `AddObserver(AsyncObserver&)`, `AddSyncObserver(fn, ctx)`, `inbox()`.
Components are templated on `TFeatureExtractor`, declared in headers and
defined in `libs/slam/src/` with `INSTANTIATE_CLASS` (ORB; SIFT removed 2026-10-11).

| Component | Header | Public API so far |
|---|---|---|
| `tracking::Tracker<FE>` | `tracking/tracker.h` | `Start()`, `Stop()`, `Track(TMonocularMeasurement)`, `Track(TBinocularMeasurement)`, `ProcessPending()`, `state()`, `current_map_id()`, `pending_measurements()` |
| `mapping::Mapper<FE>` | `mapping/mapper.h` | `Mapper(MapperOptions)`, `Start()`, `Stop()`, `ProcessPending()`, `state()`, `current_map_id()` |
| `loop_closing::LoopCloser<FE>` | `loop_closing/loop_closer.h` | `LoopCloser(LoopCloserOptions, vocabulary)`, `Start()`, `Stop()`, `ProcessPending()` (applies every transaction to its replica, queues the new Frames as queries, detects one; with loop closing off or no vocabulary only acknowledges), `replica()`, `atlas()`, `num_coincidences()`, `num_merge_coincidences()`, `last_stats()`, `last_merge_stats()`, `num_dropped_loops()`, `query_stats()` — see "Loop Closing" (merge detection since map merging Task 4) |
| `global_ba::GlobalBundleAdjuster<FE>` | `global_ba/global_bundle_adjuster.h` | `GlobalBundleAdjuster(GlobalBaOptions)`, `Start()`, `Stop()` (interrupts a running BA), `ProcessPending()` (applies every transaction, runs one pending global BA), `replica()`, `num_runs_started()`, `num_runs_interrupted()`, `num_results_dropped()`; test-only `stop_requested()`, `set_run_hook()`, `set_poll_hook()`, `set_publish_hook()` — see "Global BA" |

Each component owns a `events::WorkerThread` (`events/worker_thread.h`) bound to
the `events::WakeSignal` of its `EventNode` (`events/wake_signal.h`) and a
public `[[nodiscard]] bool ProcessPending()` (one pass, returns true if it did
work; tests drive it without a thread). Spec:
`docs/superpowers/specs/2026-10-03-tracking-states-design.md`.

**Waking without locks (resolves OQ 20).** `WakeSignal` is a
`std::atomic<uint32_t>` counter with C++20 `wait`/`notify_one` (a futex on
Linux): `Snapshot()`, `Notify()`, `Wait(seen)`. `AsyncObserver::Push()` enqueues
and notifies the attached signal (`SetWakeSignal()`); `Tracker::Track()`
notifies after enqueuing. `WorkerThread` loop: `seen = Snapshot(); if (!running)
break; if (!process_pending()) Wait(seen);`. The snapshot is taken **before**
the `running` check — otherwise a `Stop()` landing between the check and the
snapshot is already counted in `seen` and `Wait` would never return (spec
amendment found during implementation). A `Notify()` between `Snapshot()` and
`Wait()` changes the counter, so no wake-up is lost.

**Geometry status.** Monocular initialization is real (see "Monocular
Initialization"): `mapping::MapInitializer<FE>` (`mapping/map_initializer.h`:
`TryInitialize()`, `Reset()`) matches, reconstructs and scales the two views.
Stereo initialization is real too (see "Stereo Initialization"). Tracking
against the map is real (see "Tracking Against the Map"); the `PoseEstimator`
stub was deleted on 2026-10-05.

### Tracker
- **State machine** (`TrackerState`, readable from any thread via `state()`):
  `WaitingForMap` -> `Tracking` -> (`RecentlyLost` ->) `Lost` -> `Tracking`...
  (`RecentlyLost` since 2026-10-08, with a vocabulary: see "Relocalization and
  Reference-Frame Tracking"; it returns to `Tracking` in the same map on a
  relocalization, and the rest of this bullet describes a failure without
  it.)
  One `ProcessPending()` pass: (1) drain the whole inbox applying the replica
  rule (§6) — every Mapper transaction goes to the Tracker's
  `tracking::AtlasReplica`, and the Tracker tracks in its active map; a
  `MapTransaction` with `new_map` sets
  `last_frame_` to its `start_frame`'s replica entry (measurement, `T_cw`, observations as matches),
  resets the velocity, sets the map's frame id and moves
  `WaitingForMap`/`Lost` to `Tracking`; (2) take **one**
  measurement: in `WaitingForMap`/`Lost` publish it as
  `InitializationMeasurement` (`map_id` 0); in `Tracking` run the frame-to-map
  tracking described in "Tracking Against the Map" — success publishes
  `FrameTracked`, failure moves to `Lost` and then publishes
  `TrackingLost(current map, final = true)`. `Lost` behaves like
  `WaitingForMap`; relocalization lives in `RecentlyLost` (2026-10-08).
- Runs its **own thread with two input queues**:
  - `inbox()` — map diffs from the Mapper (events),
  - `measurements_` — `std::variant<TMonocularMeasurement, TBinocularMeasurement>`
    filled by `Track()`.

  `Track()` does not track: it moves the measurement into the queue and returns
  (taken by value — pass an rvalue to avoid copying the feature matrices).
  Loop on the Tracker thread:
  ```
  while running:
      apply ALL pending map diffs from inbox()     // replica is now current
      take a measurement (see Open Question 19)
      track it; publish FrameTracked
  ```
  Diffs and tracking run on the same thread, so the replica never changes while
  being tracked against — no locks. Two queues rather than one: map diffs get
  priority, stale measurements can be dropped, and a single
  `moodycamel::ConcurrentQueue` wouldn't order Mapper diffs relative to
  measurements anyway (it is FIFO per producer only).
- Input: feature measurements (mono or binocular), each with its capture
  timestamp. Both types may arrive in the same session (mixed-map goal, §1).
  Feature extraction happens before `Track()` (in the user application for now).
- Tracks against **its own** map replica.
- Optimizes **only its own camera pose** relative to the map (motion-only BA).
  Never touches `Frame`s, map points, or observations.
- Publishes `FrameTracked` for each tracked measurement: pose, the measurement
  copy, the inlier `(feature, map point)` pairs, the visible local-map points,
  the reference Frame, the local Frames, the stage and the match / inlier
  counts (§6). It does **not** decide on keyframes: the Mapper decides from
  `FrameTracked` (see "Keyframes and Map Growth").
- Its replica only changes when Mapper diff events arrive. Between publishing a
  `FrameTracked` and receiving the resulting `MapTransaction` it simply keeps
  tracking with the map it has. This lag is accepted (ORB-SLAM3 behaves
  similarly while LocalMapping processes a keyframe); a transaction adding the
  Frame of the last tracked measurement is merged into `last_frame_` at once
  (fast pickup). Replaced points are remapped in `last_frame_` (loser ->
  winner) and removed ones dropped from it.
- Marks a `FrameTracked` **urgent** (`urgent` and `Event::priority`) when its
  final inliers are below `urgent_inlier_factor` (1.5) x
  `min_local_map_inliers`: the Mapper then cuts its local BA short (see "Map
  Quality").
- Any temporary helper points the Tracker uses (e.g. ORB-SLAM3's stereo "temporal
  points" for the last frame) are Tracker-private and never part of the map.

### Mapper
- The authoritative **Atlas** and the **only** map writer (since 2026-10-10,
  map merging Task 3): maps by id (`maps_`, each a private `MapState` with
  its Frames, points, covisibility, references, counters, recent points,
  spanning tree and loop edges, and correction epoch), at most one of them
  active (`active_`); the code works on `Active()`. Mapper-wide state
  (options, vocabulary, pools, counters, inbox, thread, next map / Frame id)
  is outside `MapState`. Inactive maps are **frozen**: no local mapping,
  culling, BA or global BA; a `FrameTracked`, `LoopDetected` or
  `GlobalBAFinished` naming one is "another map's" and ignored or rejected
  (only a merge changes one: it absorbs the active map). Test accessors: `map_ids()`,
  `active_map()`, `num_keyframes(MapId)`.
- **State machine** (`MapperState`, `state()`): `Initializing` -> `Mapping`.
  *Initialization protocol:* while `Tracker` has no map it forwards
  measurements as `InitializationMeasurement`. In `Initializing` the Mapper
  feeds a **copy** of each to `MapInitializer::TryInitialize()` (the event is
  shared; never move out of it). Monocular initialization takes many frames
  (reference + a later frame), stereo one measurement. On success the Mapper
  creates a `Frame` per initial keyframe (ids from a monotonic counter, never
  reused across maps), takes `current_map_id_ = next_map_id_++`, moves to
  `Mapping`, then publishes **one** `MapTransaction` with `new_map`, every
  initial Frame and point and `start_frame` (the last initial Frame); the
  Tracker switches to `Tracking` on it. `InitializationMeasurement` is ignored
  while `Mapping`.
  A final `TrackingLost` for the active map (in `Mapping`) deactivates it,
  following ORB-SLAM3 `Tracking`: with fewer than
  `MapperOptions::atlas.min_frames_to_keep` (10) Frames (culled ones
  excluded) it is **discarded** (`ResetActiveMap`), published as one
  `MapTransaction` of the lost map with `removed_maps = {it}` and no
  `active_map`; otherwise it is **kept inactive** (`CreateMapInAtlas`),
  published as one `MapTransaction` of the lost map with no `active_map` and
  nothing else. Either way the initializer is reset and the Mapper returns to
  `Initializing`, so the next success starts map n+1 (never a reused id).
  Every replica follows the transaction (since 2026-10-10, map merging Tasks
  2–3; before, replicas cleared on the first event of map n+1). Every other
  transaction names the active map as its `active_map`. A stale
  `TrackingLost` is ignored. `atlas.merge_window` (25, ORB-SLAM3
  `numTemporalKFs`) sizes the merge windows.
- Consumes a **merge** `LoopDetected` (`query_map_id` ≠ `match_map_id`;
  since 2026-10-11, map merging Task 5, ORB-SLAM3 `MergeLocal`, visual,
  `mapper_merge.cpp`): the active map M is merged into the inactive map N,
  which survives with its id and world and becomes active. Rejected when the
  query map is not active, the match map is not live and inactive, a Frame is
  no longer in its map, or the Sim3 is invalid; a `LoopDetected` with two map
  ids is never handled as a loop. Steps: (1) the windows, each in its own map
  (`MergeWindowOf`: the Frame, its connected Frames up to 25, then up to 5
  rounds of each member's best 12 while smaller than 25); (7, done first)
  M is **absorbed** into N by the id rule (`map::AbsorbedPointIds`: M's alive
  points by old id ascending take N's next ids; Frame records (slotted by
  id over both maps' ranges: after a chained merge the ids interleave, so
  `MapState` keeps `first_slot` and the tree's `root`, the local-BA gauge,
  apart), links,
  observations, references and counters move, M's recent points and
  relocalization guard replace N's), N becomes active, so the 4b steps run
  on one map; (2) `CorrectWindow` with S·T_match; (3) the query becomes a
  child of the match and M's parent chain to its root is inverted; (4)
  `FuseLoopPoints` with the pairs (query ids translated) and the merge
  window's points; (5) the **weld** (`WeldWindows`): `BundleAdjust` of the
  current window (free) and the merge window (fixed) with all their points,
  their observations only, outliers removed; (6) `MoveRestOfMap`: the other
  Frames of M take the similarity (points with their reference Frame), then
  an essential graph over M's Frames with the welded window fixed
  (`EssentialGraphProblem::also_fixed`) moves them again; (7) the merge edge
  as a loop edge; (8) one `MapTransaction` of N: `absorbed_map = M`,
  `removed_maps = {M}`, `active_map = N`, N's epoch + 1, every Frame of M's
  pose, the parent changes, the fusions, the loop edge and every point (as a
  loop correction). The loop edge starts a global BA of N. Logged with its
  time (about 90 ms for the 18-Frame test scene, Debug -O2).
- Consumes `FrameTracked` (current map, `Mapping` only): drains to the latest
  per map, decides with `ShouldCreateFrame` whether the measurement becomes a
  Frame, and creates it — tracked observations, stereo close points,
  triangulation against covisible neighbour Frames (see "Keyframes and Map
  Growth") — then fuses, culls and runs local BA around it, publishing two
  `MapTransaction`s per Frame (see "Map Quality"). Every `FrameTracked` of the
  current map, evaluated or skipped, updates the points' visible / found
  counters.
- Consumes `LoopDetected`: performs the **entire** loop closure — validation,
  Sim3/SE3 correction, map point fusion, essential graph optimization — as
  one `MapTransaction` at a new map epoch (see "Loop Closing"; global BA is
  roadmap step 4c).
- Publishes every change as one `MapTransaction` (see §6).
- Validates incoming IDs against its own replica: events from other threads were
  computed on a lagging replica, so referenced points/frames may already be culled.
  Thanks to ID non-reuse this is a plain `IsValid()` check.

### LoopCloser
- **Detection only.** Place recognition on its own replica (`FrameDatabase`,
  covisibility) and geometric verification of the candidate (Sim3 RANSAC,
  `OptimizeSim3`, projections, three confirmations), ORB-SLAM3
  `NewDetectCommonRegions` (see "Loop Closing").
- Publishes `LoopDetected` (query and match Frames and maps, the Sim3
  `S_query_match`, the matched point pairs). Never modifies the map.

### GlobalBA
- Created by `Slam` only with loop closing in effect. Owns a replica fed by
  the Mapper's transactions from the start; a transaction that adds a loop
  edge triggers ORB-SLAM3's global BA of a snapshot of the replica
  (see "Global BA").
- Publishes `GlobalBAFinished` (plain data); the Mapper merges it. Never
  modifies the map.

### Visualizer (MapViewer)
- `voyahviz::MapViewer` (see "Visualization Library") subscribes to Tracker
  and Mapper events through one inbox and builds a render-only `SceneState`
  from them. It is not part of `Slam`; the apps attach it.
- **Remote visualization (target):** SLAM and viewer run on different
  machines. On the SLAM machine an async observer serializes events and sends
  them over HTTP or ROS; on the viewer machine a receiver rebuilds the events
  and feeds the same `SceneState`/renderer code. Hence the viewer may use
  nothing but event contents. Camera images are a separate flow (acquisition
  and feature extraction are outside the SLAM logic).

## 5. Data Layer (implemented)

Detailed spec: `docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md`.

| Component | Header | Summary |
|---|---|---|
| `ObjectPool<T, SyncPolicy>` | `memory/object_pool.h` | Pre-allocated slab, intrusive free-list, asserts on exhaustion. |
| `Recyclable<T, SyncPolicy>` | `memory/recyclable.h` | CRTP atomic ref count (acq_rel); returns object to its pool at 0. `SetRefCount(n)` for fan-out. |
| `ThreadSafePolicy` / `NoSyncPolicy` | `memory/sync_policy.h` | Pool free-list mode: lock-free CAS / single-thread. |
| `ColumnBuffer<TScalar, Rows>` | `memory/column_buffer.h` | Grow-only `Rows × N` Eigen buffer; `Resize(n)` allocates only beyond capacity. |
| `MapPointId`, `FrameId` | `map/*_id.h` | Strong ID wrappers. |
| `MapPointStorage<FE>` | `map/map_point_storage.h` | SoA: positions, descriptors, normals, num observations, valid flags. |
| `MonocularMeasurement<T, L>` | `features/monocular_measurement.h` | `ImageFeatures` + K, raw keypoints (`distorted_points()`), undistorted & normalized points, `camera()` (`geometry::Camera`: K + distortion pointer), `timestamp()`. |
| `Frame<FE, TMeasurement>` | `map/frame.h` | Keyframe: id, pose, measurement (held as `shared_ptr<const TMeasurement>`, never null, since 2026-10-10; `measurement()` reads it, `shared_measurement()` shares it); `timestamp()` comes from the measurement. `MonoFrame<FE>`, `StereoFrame<FE>` aliases. |
| `ObservationStorage<Type>` | `map/observation_storage.h` | SoA observation table; stereo fields compiled out for mono. Unused by the Mapper since 2026-10-07 (the `CovisibilityIndex` is its observation record). |
| `CovisibilityIndex` | `map/covisibility_index.h` | Per point its observing Frames, per Frame its points; `Remove`, `RemovePoint`, `RemoveFrame` keep both sides in step. |
| `StereoRig` | `features/stereo_rig.h` | K left/right, `T_rl` (left → right: `p_r = T_rl p_l`), baseline, distortion models. |
| `BinocularMeasurement<T, L>` | `features/binocular_measurement.h` | Left/right `MonocularMeasurement`, copy of its `StereoRig`, stereo matches (`SetStereoMatches`; per-feature depth, right index); `timestamp()` = left's. |
| `Event`, `EventType` | `events/event_types.h` | Base event: type tag + `recycle_` / `set_ref_count_` fn pointers. |
| concrete events | `events/<event_name>.h` | One header per event (e.g. `events/map_transaction.h`); `events/events.h` includes all. See §6. |
| `Observable` | `events/observable.h` | Sync handlers (inline, fn ptr + ctx) run first, then async enqueue with ref count = #async queues. |
| `AsyncObserver` | `events/async_observer.h` | One queue per observer (1:1 channel); counts the pending priority events (`pending_priority()`). |
| `Listener` | `events/listener.h` | Iterates a queue; auto-recycles previous/last event; move-only. |

Ported from slamlab2: `images/`, `io/`, `algebra/`, `geometry/`, `features/`
(ORB extractor, brute-force matcher, Kannala-Brandt distortion; the SIFT
extractor also came over and was removed on 2026-10-11), `logging/`.
The distortion models now live in `geometry/distortion_models/` (moved from
`features/` on 2026-10-05; forwarding headers remain at the old path).

## 6. Events

| Event | Producer → Consumer | Payload |
|---|---|---|
| `FrameTracked<FE>` | Tracker → Mapper, viewer | every tracked measurement: timestamp, pose (`T_cw` after `OptimizePose`), the shared measurement (binocular: stereo-matched; since 2026-10-10 a `shared_ptr<const>`, not a copy), inlier pairs `observed_features[i]` -> `observed_points[i]`, `visible_points` (local points passing the projection gate, matched ones included), `reference_frame_id` (optional), `local_frames`, `stage` (the source of the initial pose the local-map step refined: `MotionModel` / `ZeroMotion` / `ReferenceFrame` / `Relocalization`; 2026-10-08, ruling C), `num_matches`, `num_inliers`, `urgent` (mirrored in `Event::priority`), `map_epoch` (the Tracker replica's epoch; since 2026-10-09 the Mapper drops a `FrameTracked` of an older epoch, after counting its visible/found) |
| `MapTransaction<FE>` | Mapper → all | **every Mapper change, as one atomic transaction** (since 2026-10-07; replaces `KeyFrameCreated`, `MapCreated`, `MapPointsUpdated`, `MapPointsRemoved`, `KeyFramePoseUpdated`). Parts, each possibly empty: `new_map` + `start_frame`; added Frames (id, `T_cw`, measurement, own observations, and since 2026-10-08 the Frame's `bow::BowVector bow` and `bow::FeatureVector features`, empty without a vocabulary); new points (`new_points_start_id`, contiguous ids, positions / normals / descriptors); `added_observations` (frame, feature, point, depth and right index, -1 when absent); updated points (ids, positions, normals, descriptors); `replaced_points` (loser, winner); `removed_points`; `updated_poses`; `removed_observations`; `removed_frames`; since 2026-10-09 (loop closing) the map graph and epoch: `NewFrame::parent` (spanning tree), `parent_changes` (frame, new parent), `added_loop_edges` (frame, frame) and `map_epoch` (+1 per loop correction; 0 for a new map). A loop correction is one more transaction carrying every corrected pose and point. Pooled (256 in the Mapper): a transaction is recycled once every consumer has acknowledged it, so consumers apply on receipt (the LoopCloser too, since 2026-10-10) and never hold one for long work. Apply order and contract: see "Map Quality" |
| `LoopDetected` | LoopCloser → Mapper | `query_map_id` (the active map), `match_map_id` (equal for a loop; an inactive map's id for a merge, since map merging Task 4), `query_frame`, `match_frame`, `S_query_match` (`geometry::Sim3`, x_query = S · x_match in camera coordinates, ORB Scm; scale 1 for stereo), `matched_points` ((query point, match point) pairs). `priority` (interrupts a running local BA) unless a loop was already published at the LoopCloser replica's epoch. Pool of 16, published with `TryCreate`. See "Loop Closing" |
| `GlobalBAFinished` | GlobalBA → Mapper | `map_id`, `map_epoch` (N, the correction epoch optimized), `frame_poses` ((FrameId, T_cw) of every Frame of the run), `point_positions` ((MapPointId, world position) of every point of the run): plain data only. `priority` (interrupts a running local BA). Pool of `GlobalBaOptions::max_events_in_flight` (2), published with `TryCreate` (exhausted: the result is dropped and counted). The Mapper merges it as one `MapTransaction` at N + 1, or rejects it (another map or epoch). About 1.2 MB for a corridor1 map of 600 Frames and 38k points. See "Global BA" |
| `InitializationMeasurement<FE>` | Tracker → Mapper (+ other Tracker observers) | `features::TMeasurement<FE>` (`map_id` 0) |
| `TrackingLost` | Tracker → all | timestamp, `final` (`map_id` = the lost map). Since 2026-10-08: `final = false` on entering `RecentlyLost` (the Mapper ignores it and keeps the map), `final = true` on entering `Lost` (only this one clears the Mapper's map) |

Rules:
- Events are coarse-grained (one event per batch, Eigen matrices inside).
- Each concrete event type has its own `ObjectPool`.
- Consumers never call `Recycle()` — the `Listener` does.
- Per channel delivery is FIFO, so a replica applying diffs in order always ends
  up identical to the Mapper's map as of the last applied event.
- Because IDs are never reused, a `MapTransaction` allocates its new points a
  contiguous id range `[new_points_start_id, new_points_start_id + count)` and
  every replica stores point `i` at the same slot.
- **Atomic application:** a consumer applies a `MapTransaction` in one step, in
  the fixed order of "Map Quality" (new map, add Frames, add points, add
  observations, update points, update poses, remove observations, remove
  points, remove Frames) — the Tracker never tracks against a half-applied
  change.
- **Priority events and the inbox counter.** A producer sets
  `Event::priority` before publishing (the Tracker: on urgent `FrameTracked`).
  `AsyncObserver::Push` increments the inbox's `pending_priority` counter
  before the enqueue; **only `AsyncObserver::TryPop` (and a `Listener` from
  `GetListener()`) decrement it** — popping from the raw `queue()` bypasses
  the counter and leaves it too high (TODO: make `queue()` private). The
  counter is event infrastructure, not map data, so it does not break the
  no-locks-on-map-data rule.
- **Older event names.** Sections written before 2026-10-07 name
  `KeyFrameCreated`, `MapCreated`, `MapPointsUpdated` / `MapPointsRemoved` and
  `KeyFramePoseUpdated`; read them as the corresponding `MapTransaction`
  parts (added Frame, `new_map`, updated / removed points, updated poses).
- **Pointer rule:** an event may point only at data that never changes. Raw
  pointers are allowed only to data that lives for the whole run
  (configuration); immutable data that may die earlier is shared-owned. A
  serializer turns such references into values (inline, or sent once and
  referenced). Everything mutable or short-lived travels by value — e.g. a
  pointer to a reused Tracker buffer is never allowed. Two payloads are shared,
  immutable and owned by `shared_ptr<const ...>`, so they live as long as any
  holder and a serializer sends them by value:
  - a Frame's measurement (since 2026-10-10): immutable after the Tracker
    creates it;
  - a `DistortionModel` (since 2026-10-12): `TDistortionModelPtr` in every
    `geometry::Camera`, `StereoRig` and measurement, so the app need not keep
    its calibration objects alive. Code that only uses a model during a call
    takes `const DistortionModel*` / `&`.
- **Measurement variant (resolves OQ 24).** `features/measurement.h` defines
  `TMonocularMeasurement<FE>`, `TBinocularMeasurement<FE>` and
  `TMeasurement<FE> = std::variant<mono, binocular>`, so mono and stereo
  share one event type each. `InitializationMeasurement` holds a
  `TMeasurement<FE>` by value. `FrameTracked` and `MapTransaction` (per added
  Frame) hold a **`TSharedMeasurement<FE> = std::shared_ptr<const
  TMeasurement<FE>>`** (since 2026-10-10, map-merging spec section 1): the
  Tracker creates it with `std::make_shared` once per tracked frame (after
  stereo matching, in `AcceptPose`), and the Mapper's Frame, its `NewFrame`
  and every replica's `ReplicaKeyFrame` keep that same object; nothing writes
  to it afterwards (`const` by type). The only other creator is the Mapper,
  for a new map's initial Frames, whose measurements come from
  `InitializationMeasurement` copies. A pooled event releases its pointer on
  reuse (`Reset()`).
- **`map_id`.** `map::MapId` (`map/map_id.h`, strong `uint32_t`; ids start at
  1, only increase, `MapId{0}` = not tied to a map). `Event` has a public
  `map_id` field that **producers must set on every publish** (pooled events
  are reused without reset). It is the map the Mapper is building
  (`MapTransaction`), the map the Tracker tracks against (`FrameTracked`,
  `TrackingLost`) or 0 (`InitializationMeasurement`).
  **Replica rule** (since 2026-10-10, map merging Task 2; it replaced "newest
  map id wins"). Every replica — the Tracker's, the LoopCloser's and the
  GlobalBA's `tracking::AtlasReplica` (`tracking/atlas_replica.h`) and the
  viewer's `SceneState` — mirrors the Mapper's Atlas: every live map by id,
  and which one is active. Per `MapTransaction` of map `map_id`: map 0 is no
  map (ignored); `new_map` creates the map; any other transaction applies to
  the live map it names, and one naming an unknown or removed map is ignored
  whole; then the maps in `removed_maps` are dropped and their ids remembered
  (never admitted again); then the active map is taken from `active_map`
  only, never from the newest id. The Mapper sets `active_map` on every
  transaction. Other events (`FrameTracked`, `TrackingLost`, ...) change no
  replica: a final `TrackingLost` is answered by the Mapper's transaction
  that deactivates the map (`removed_maps` too if it is discarded); a kept
  map stays live, inactive, in every replica. A merge (`absorbed_map` set,
  since map merging Task 5) first moves the absorbed map's Frames,
  observations and points into the map it names by the Mapper's id rule
  (`MapReplica::Absorb`, `map::AbsorbedPointIds`), then applies the
  transaction, then drops the absorbed map (it is in `removed_maps`).
  `AtlasReplica::TranslatePoint(absorbed, old)` gives an absorbed point's id
  in the merged map, after the merge's fusions (nullopt if it was not alive or
  is gone): the Tracker's last-frame matches follow it (Task 6, "Loop
  Closing", Tracker across a merge). The Mapper builds map n+1 only after a
  final `TrackingLost(n)`, so a newer map never appears while the Tracker
  tracks an older one.
- **No subscription filtering:** an `Observable` delivers every event to every
  connected observer (the viewer also receives `InitializationMeasurement` and
  just counts it). A future serializing observer chooses what to send.

### 6.1 Event Lifecycle: Pooled, Reference-Counted, Acknowledged

**Goal:** publishing an event must not hit the general-purpose heap allocator —
neither for the event object nor for its payload. Events are produced at frame
rate on every thread; `new`/`delete` per event means allocator contention
between threads and unpredictable latency. Instead, every concrete event type
has a pre-allocated **`ObjectPool`**, each event is a **`Recyclable`** object
that knows how to return itself to its pool, and pooled objects are **kept
alive** between uses so their payload buffers are reused (steady state: zero
allocations per event).

**The building blocks (`libs/common/include/voyahslam/memory/`):**

- `ObjectPool<T, SyncPolicy>` — one contiguous slab of `max_count` slots
  allocated once in the constructor. Free slots form a stack of slot indices
  whose links live in a separate `atomic<uint32_t>` array, so slot memory is
  never read by the allocator and is not touched (or committed by the OS) until
  first use. `Create()` pops a slot (asserts when exhausted); `TryCreate()`
  returns `nullptr` instead. The first time a slot is used its `T` is
  default-constructed; on every later reuse the pool resets the ref count to 1
  and calls `T::Reset()` if `T` defines one. `Recycle(T*)` just pushes the slot
  back — **the object is not destroyed**, so vectors and Eigen buffers inside it
  keep their allocations. Objects are destroyed only in `~ObjectPool`. All O(1).
- `ColumnBuffer<TScalar, Rows>` — grow-only `Rows × N` Eigen buffer used for
  event payloads. `Resize(n)` sets the visible column count and reallocates
  (geometrically, preserving data) only when `n` exceeds capacity; `matrix()`
  returns the `leftCols(n)` view. Plain `Eigen::Matrix` can't be used here
  because it reallocates on every size change.
- `Recyclable<T, SyncPolicy>` — CRTP mixin that gives an object an atomic
  reference count (starts at **1**) and a back-pointer to the pool that created it.
  `Recycle()` decrements; whoever drops the count to 0 returns the object to its
  pool. `SetRefCount(n)` sets the count in one store (used for fan-out).
- `SyncPolicy` — `ThreadSafePolicy`: the free-list is a lock-free Treiber stack
  (CAS on a 64-bit head packing `tag << 32 | index`; the tag is bumped on every
  push/pop to defeat ABA). Required for event pools, because `Create` runs on the
  producer thread and `Recycle` on whichever consumer acknowledges last.
  `NoSyncPolicy`: same structure with plain loads/stores, for single-thread pools.
- `Event` — non-virtual base with a type tag and two function pointers
  (`recycle_`, `set_ref_count_`) wired by each concrete event's constructor, so
  code holding only an `Event*` can manage the lifetime without virtual dispatch
  and without knowing the concrete type.

**The lifecycle of one async event:**

```
 producer thread                         consumer threads (N async observers)
 ───────────────                         ────────────────────────────────────
 1. e = pool.Create()        ref = 1
 2. fill payload
 3. observable.Notify(e)
      a. sync handlers run inline (e alive, producer still owns it)
      b. SetRefCount(N)       ref = N    ◄── ownership transferred to consumers
      c. enqueue e on each of the N queues
 4. producer forgets e (must not touch it again)
                                          5. listener.GetNextEvent() → e
                                             consumer reads the payload
                                          6. next GetNextEvent() / ~Listener()
                                             = acknowledgement: e->Recycle()
                                             ref = ref - 1
                                          7. last consumer to acknowledge
                                             (ref → 0) returns e to the pool
```

**Acknowledgement is implicit.** A consumer never calls `Recycle()` itself. The
`Listener` treats "asking for the next event" as "I am done with the previous
one", and its destructor acknowledges the last event it handed out. So the
pointer returned by `GetNextEvent()` is a **borrow**, valid until the next
`GetNextEvent()` call or until the `Listener` is destroyed.

**Rules for consumers:**
1. Do not keep an `Event*` (or references/`Map`s into its payload) beyond the
   next `GetNextEvent()`. Copy whatever must outlive it into the local replica.
2. Treat the payload as **read-only**. With fan-out the same object is read
   concurrently by all N consumers, so moving data *out* of an event (e.g.
   `std::move(e->positions)`) is a data race.
3. Don't sit on events: until every consumer has acknowledged, the slot is not
   back in the pool. A stalled consumer drains the pool and the producer's
   next `Create()` asserts. Pool sizes must cover the worst-case backlog.
4. A `Listener` is move-only; moving it transfers the borrowed event, so it is
   acknowledged exactly once.

**Rules for producers:**
1. **`Notify()` always takes ownership.** After it returns the producer must not
   touch the event. With N > 0 async observers, the count is set to N and the
   consumers release it; with none, `Notify()` recycles it right after the sync
   handlers. The producer doesn't need to know how it is wired.
2. An event with a null `recycle_` is never recycled — this is how a
   stack-allocated event is published to sync-only observers.
3. Sync handlers run inline before the async enqueue and must not retain the
   event or `AddRef()` it — the subsequent `SetRefCount(N)` would overwrite that
   reference.
4. Use `TryCreate()` where exhaustion is an expected, recoverable condition
   (back-pressure) and `Create()` where it indicates a bug.
5. A created event may be a reused one. Events with a `Reset()` come back
   cleared (counts 0, ids empty, capacity kept). Events **without** `Reset()`
   (`LoopDetected`, `TrackingLost`, `InitializationMeasurement`) still hold
   the previous values: the producer must set every field. `FrameTracked` and
   `MapTransaction` have `Reset()` (every part cleared, capacity kept).
6. Fill batched payloads with `e->Resize(n)` and then write into
   `e->positions.matrix()` / `.col(i)` etc. — never assign a freshly built
   `Eigen::Matrix` to a buffer's storage, that would throw away the reuse.

**Adding a new event type:** inherit `Event` and
`memory::Recyclable<Self, memory::ThreadSafePolicy>`, wire `recycle_` /
`set_ref_count_` in the constructor (see e.g. `events/frame_tracked.h`), make it default
constructible, store variable-size payloads in `ColumnBuffer`s / `std::vector`s,
and give it `Reset()` if it has any. Put it in its own header
`events/<snake_case_name>.h`, add the `EventType` value, and add it to
`events/events.h`.

## Example Applications (`examples/`)

Configuration lives in the user application (§9 Q18), so `examples/` holds the
reference applications:

| Target | Kind | Purpose |
|---|---|---|
| `voyahslam_dataset_reader` | static lib (`examples/dataset_reader/`) | `EurocDatasetReader`, `PlaybackClock` |
| `monocular_full_slam` | executable (`examples/full_slam/`) | cam0 → `MonocularMeasurement` → `Tracker::Track()` |
| `binocular_full_slam` | executable (`examples/full_slam/`) | cam0+cam1 → `BinocularMeasurement` → `Tracker::Track()` |
| `dataset_reader_tests` | gtest | Synthetic-dataset tests for the reader and clock |

Usage: `<app> <dataset_root> [--rate=R] [--max-frames=N]` (rate 1 = real
time, 0 = as fast as possible). Both apps print lateness and load/extract
timings. Further options: `--show`, `--viewer`, `--trajectory_out <file>`,
`--vocabulary <file.bin>`, `--loop_closing` (opt-in, needs the vocabulary;
off by default, see "Loop Closing"), `--drop_frames a:b`.

**`EurocDatasetReader`** reads the EuRoC layout (`mav0/cam{0,1}/data.csv`,
`data/<ns>.png`) used by EuRoC MAV and TUM-VI. Calibration: Kalibr
`dso/camchain.yaml` (TUM-VI) if present, else `mav0/camN/sensor.yaml` (EuRoC;
the OpenCV `%YAML:1.0` directive is stripped for yaml-cpp). It owns the
distortion models and builds the `StereoRig` from both cameras' `T_cam_imu`
(`T_rl = T_right_imu · T_left_imu⁻¹` with `T_cam_imu` mapping IMU to camera, so `p_right = T_rl p_left`: `StereoRig::T_rl` maps left → right).
Images: decoded to 8 bit (16-bit PNGs reduced) and returned as
`images::TWorkingImage` (8-bit grayscale).
Distortion models: `equidistant` (Kannala-Brandt), Brown-Conrady under its
names `radial-tangential` (EuRoC), `radtan` (Kalibr) and `plumb_bob` (ROS)
with 4 (`[k1, k2, p1, p2]`) or 5 (`[..., k3]`) coefficients (any other count is
rejected), and `none`. See "Camera models (lens distortion)".

**Timing — no reader thread.** The reader is synchronous and index-based. The
app's main loop paces itself with `PlaybackClock::WaitUntil(timestamp)`:
absolute deadlines (`anchor + (t - t0) / rate`), so per-frame work doesn't
accumulate drift as long as it fits in the frame gap; if it doesn't, the clock
returns the lateness rather than drifting silently. Measured on TUM-VI
corridor1 (Release, 1024², 1500 ORB features): load ≈ 10 ms, ORB extraction
≈ 220 ms per image — so a separate reader thread would buy nothing; the
extractor is the bottleneck (Open Question 21).

**Reference dataset:** TUM-VI `dataset-corridor1_1024_16` (20 Hz stereo,
1024×1024 16-bit, equidistant fisheye, baseline 0.101 m).

## Visualization Library (`vizualization/`, `voyahviz`)

Development tool, never part of the production build. Implemented with
OpenCV (HighGUI, imgproc) and Pangolin (3D map viewer), but — like every
library of ours — both are private: the public API uses only voyahslam types
and OpenCV and Pangolin are linked `PRIVATE` (principle 8). Namespace / include prefix: `voyahviz`.

| Header | Contents |
|---|---|
| `color_image.h` | `TColorImage` = `images::Image<uint8_t, 3>` (interleaved B, G, R); `Color` (RGB); `ToColor(TImageU8Gray)`; `HorizontalConcat(left, right)` |
| `keypoint_drawing.h` | `KeypointStyle`; `DrawKeypoints(TColorImage& out_canvas, points, angles*)` and a gray-image overload returning a new canvas; `DrawFeatures(image, ImageFeatures)` for raw extractor output; `ProjectToImage(measurement, distortion)` / `DrawMeasurement(...)` |
| `image_window.h` | `ImageWindow`: named window, `Show(TImageU8Gray / TColorImage)`, `WaitKey()`, `GuiBackend()`; throws if no GUI backend |

Internally `vizualization/src/opencv_wrapping.h` wraps our images as `cv::Mat` without
copying (channels interleaved, stride in pixels).

**Pitfall:** `MonocularMeasurement::points()` are *undistorted* pixel
coordinates — on a fisheye they don't line up with the raw image.
`DrawFeatures()` is deleted for measurements; use `DrawMeasurement()`, which
maps features back via `K · Distort(normalized)`. Since 2026-10-05 measurements
also keep the raw keypoints (`distorted_points()`), which line up with the raw
image directly.

**Threading:** HighGUI isn't thread-safe; create the window and call
`Show()`/`WaitKey()` from one thread (the apps' main thread). A future live
viewer fed by Tracker/Mapper events must marshal frames to that thread.

`monocular_full_slam` / `binocular_full_slam --show` display the live frame(s)
with keypoints (stereo side by side); `q`/`Esc` quits.

### Map Viewer

Spec: `docs/superpowers/specs/2026-10-03-map-viewer-design.md`.

| Header | Contents |
|---|---|
| `scene_state.h` | `SceneState<FE>`: `Apply(const Event&)`, per-`EventType` counters, and the decoded scene: map points (dense, indexed by per-map point id), keyframe poses (`T_cw`), current camera pose, trajectory, and from the latest `FrameTracked` the tracked and visible point ids and the local Frames. Built only from events; no threads/GUI |
| `camera_frustum.h` | Camera frustum geometry helper (corner points of a frustum of a given size) |
| `map_viewer.h` | `MapViewer<FE>(MapViewerOptions{fps, show_window, width, height, camera_size})`: one `AsyncObserver` inbox, `Start()` / `Stop()` / `RunOnCurrentThread()` / `RequestStop()`, `inbox()`, `events_processed()`, `window_closed()`, `WaitForWindowClose()` |
| `pangolin_renderer.h` (private, `vizualization/src/`) | Pangolin window: points (tracked green, visible blue, others grey), keyframe frustums (local-map Frames orange, others blue), current camera (green, red when lost), trajectory, control panel (toggles, map id, counts, status) |

Specs: `docs/superpowers/specs/2026-10-04-pangolin-viewer-design.md`
(Pangolin viewer), `docs/superpowers/specs/2026-10-03-map-viewer-design.md`.

- **`SceneState` is the scene model.** It is rebuilt purely from events and
  follows the replica rule (§6; voyahviz cannot include
  `tracking::AtlasReplica`, so it applies the same rule to its own per-map
  scenes): it keeps every live map's points and keyframes and shows the
  active map (nothing if none). The camera pose, trajectory and tracked sets
  come from `FrameTracked` / `TrackingLost` of the active map; a change of
  the active map clears them, and resets the status unless no map is left
  active (after a final loss it stays Lost). Events with map 0 are counted
  only. Showing inactive maps is TODO (map merging spec).
- **Loop-thread invariant:** only the thread running the loop touches the
  `SceneState`, the renderer and the GL context. The window is created at loop
  entry and destroyed at loop exit (or on close), on that thread. Other threads
  only use the atomics (`window_closed()`, `events_processed()`) and `inbox()`.
- Loop: drain inbox through a local `Listener` (acknowledges each event) ->
  `SceneState::Apply` -> render (if the window is open) -> sleep to the next
  absolute deadline (no catch-up when late). No busy-waiting.
- **`Start()` vs `RunOnCurrentThread()`:** `Start()` runs the loop on the
  viewer's own thread (Linux). `RunOnCurrentThread()` runs it on the caller's
  thread, because on macOS the GUI (Pangolin/Cocoa) must live on the main
  thread; the app would then run playback on a worker thread. The apps use
  `Start()` today; the macOS setup is in TODO.
- **Stop semantics:** `Stop()` in `Start()` mode joins and drains once more so
  queued events return to their pools; idempotent, safe without `Start()`,
  restart allowed, destructor stops. During `RunOnCurrentThread()`, `Stop()`
  only requests the stop and the loop drains itself on its own thread; other
  threads end that run with `RequestStop()` only (a `Stop()` before the loop is
  entered would consume the request and a later `RunOnCurrentThread()` would
  block), and `Stop()` / the destructor belong on the loop's thread after it
  returns. `Start()`/`Stop()` must be called from one controlling thread
  (externally ordered). A
  `RequestStop()` made before the loop is entered is kept (the run drains once
  and returns). `fps <= 0` throws.
- **Window close does not end the loop.** It keeps draining headless (so the
  producers' event pools never stall); only `RequestStop()` / `Stop()` end it.
  `window_closed()` is also true if the window could not be created (logged,
  runs headless). Apps stop playback when it turns true and, after the last
  frame, wait with `WaitForWindowClose()`.
- **Wayland:** on Linux under Wayland with `PANGOLIN_WINDOW_URI` unset, the
  `MapViewer` constructor sets it to `x11://` (XWayland): Pangolin's Wayland
  backend blocks in `eglSwapBuffers` on GNOME, which would stall the drain and
  the producers' pools. Construct the viewer before other threads start.
- **Ordering:** connect with `AddObserver(viewer.inbox())` before the producers
  start; stop `Slam` before the viewer.
- Pools feeding the viewer must cover the backlog published between two ticks
  (about 33 ms at 30 fps) - relevant to OQ 14.
- **Pangolin is a private dependency:** linked `PRIVATE` via `pango_display`
  (installed in `/usr/local`, 0.9); no Pangolin types in public headers
  (principle 8).
- The full_slam apps always attach a `MapViewer` (headless by default) and
  print `viewer: N events processed`; `--viewer` opens the window (also stops
  playback when the window is closed and, at the end of playback, waits for it
  to be closed).
- Verified with ThreadSanitizer (`MapViewer*:SceneState*`, 2026-10-04: clean).

## FAST Corner Detector (`features/fast.h`, `voyah_algs`)

**Since 2026-10-11** the ORB extractor detects corners with our own port of
OpenCV's FAST instead of `cv::FAST`; with the SIFT extractor removed the same
day, no `voyah_*` library uses OpenCV.

- **Source.** `FAST_t<16>` (9-of-16 segment test with non-maximum
  suppression, `modules/features/src/fast.cpp`), `makeOffsets` and
  `cornerScore<16>` (`fast_score.cpp`) and the AVX2 kernel
  (`fast.avx2.cpp`) from OpenCV 5, branch `5.x` at commit `20e367198c`
  (version 5.1.0-dev: the OpenCV built from source into `/usr/local` that the
  tests compare with; there is no `5.1.0` tag yet, and these functions are
  unchanged since tag `5.0.0`). The local checkout `~/git/opencv` was checked
  byte-for-byte against GitHub at that commit.
- **Licence.** OpenCV 5 is Apache-2.0; `libs/algs/src/features/fast/fast.cpp`
  carries OpenCV's copyright list and the Apache-2.0 notice plus Edward
  Rosten's original BSD notice for the FAST code, and states the changes;
  the SIMD files carry a short form pointing to it.
- **Interface.** `DetectFast(data, width, height, stride, threshold,
  nonmax_suppression[, out_corners][, simd])` (and an `images::TWorkingImage`
  overload) returns `std::vector<FastCorner{x, y, response}>`, row-major.
  Stride-aware: only pixels inside the view are read, so the extractor passes
  each grid cell as a sub-view without copying. `threshold` is clamped to
  [0, 255] (OpenCV's SIMD paths would use it unclamped; out-of-range
  thresholds are not part of the contract). Scratch rows are per thread
  (`thread_local`), so per-cell calls do not allocate.
- **Bit-exact with `cv::FAST(img, kps, t, true)` (TYPE_9_16):** same corners,
  order and responses (OpenCV's float positions and scores are integers).
  Every path runs OpenCV's algorithm for its width: scalar = OpenCV's scalar
  loop and scalar `cornerScore`; SSE2 / NEON = the `CV_SIMD128` block (16 px)
  with its vector score; AVX2 = OpenCV's AVX2 block (32 px), then the 16-px
  block, then the scalar tail, as OpenCV's dispatch does. The scores of the
  scalar and vector formulas agree on every detected corner (its score is at
  least the threshold), so all paths give identical output.
- **SIMD paths and dispatch** (`FastSimd`, `IsFastSimdAvailable`,
  `ResolveFastSimd`; `kAuto` by default). x86-64: SSE2 is the baseline
  (`fast_sse2.cpp`, no flags); AVX2 lives in `fast_avx2.cpp`, compiled with
  `-mavx2` (GCC / Clang) and chosen at run time with
  `__builtin_cpu_supports("avx2")`, so the library still runs on CPUs without
  AVX2 and needs no `-march`. arm64: NEON (`fast_neon.cpp`, baseline, no
  dispatch). Other targets: scalar. The installed OpenCV runs AVX2 here
  (baseline SSE3, dispatch list SSE4_1 … AVX512_SKX; FAST has only an AVX2
  dispatch file), so AVX2 is the speed bar. A kernel whose block cannot fit
  the view is skipped (ORB cells are about 35 px wide).
- **Verification.** `FastTest.*` (unit, `voyah_algs_tests`) compares every
  runnable path with live `cv::FAST`: noise at widths around the 16 / 32-px
  blocks, sparse corners (the block-restart branch), stride ≠ width, flat
  images, images smaller than 7×7, corners at the 3-px border, thresholds 0
  and 255. `FastCorridor1Test` (integration, `voyah_algs_dataset_tests`)
  does the same on three corridor1 frames at all 8 pyramid levels, both
  extractor thresholds (20, 7), with and without suppression, and on every
  ORB grid cell as a sub-view. Scalar, SSE2 and AVX2 are verified on this
  machine (and a deliberate score bug in either SIMD kernel fails both
  tests). **NEON has not run on arm64 hardware** (no cross compiler or
  emulator installed): it passed both test files on x86-64 through SIMDe's
  NEON emulation (v0.8.2, a scratch build outside the repo), which checks
  the logic but not the real compiler or CPU (TODO).
- **ORB extractor unchanged.** `OrbExtractorGoldenTest` (same dataset
  binary) holds per-frame, per-level digests (FNV-1a) of keypoints,
  responses, angles and descriptors on five corridor1 frames, recorded with
  the `cv::FAST` extractor before the switch
  (`libs/algs/tests/features/data/orb_corridor1_golden.txt`); the port
  reproduces it byte for byte (Debug and Release trees).
- **Dataset tests** read corridor1 from `$VOYAHSLAM_CORRIDOR1`, else
  `~/Downloads/dataset-corridor1_1024_16`, and skip without it.
- **Benchmark** (`libs/algs/benchmarks/fast_benchmark`, built with `all`, not
  part of `check` / `check_all`; Release, single thread, 20 corridor1 frames
  × 50 repetitions, median per frame, ratio = ours / OpenCV, 2026-10-11;
  OpenCV and ours alternate who runs first per frame and repetition, so neither
  always gets cache-warm data):

  | level (size) | t | OpenCV µs | ours (AVX2) µs | ratio | SSE2 only | scalar only |
  |---|---|---|---|---|---|---|
  | 0 (1024²) | 20 / 7 | 262 / 523 | 237 / 498 | 0.90 / 0.95 | 1.18 / 1.14 | 3.97 / 3.77 |
  | 1 (853²) | 20 / 7 | 204 / 401 | 183 / 384 | 0.90 / 0.96 | 1.15 / 1.15 | 3.80 / 3.67 |
  | 2 (711²) | 20 / 7 | 146 / 300 | 134 / 290 | 0.92 / 0.97 | 1.19 / 1.17 | 3.94 / 3.76 |
  | 3 (593²) | 20 / 7 | 122 / 248 | 110 / 240 | 0.90 / 0.96 | 1.11 / 1.10 | 3.59 / 3.50 |
  | 4 (494²) | 20 / 7 | 96 / 192 | 90 / 185 | 0.94 / 0.96 | 1.15 / 1.17 | 3.43 / 3.57 |
  | 5 (412²) | 20 / 7 | 73 / 151 | 68 / 144 | 0.94 / 0.95 | 1.14 / 1.15 | 3.43 / 3.56 |
  | 6 (343²) | 20 / 7 | 56 / 114 | 53 / 110 | 0.95 / 0.96 | 1.14 / 1.15 | 3.46 / 3.66 |
  | 7 (286²) | 20 / 7 | 49 / 99 | 46 / 95 | 0.94 / 0.96 | 1.10 / 1.12 | 3.06 / 3.31 |
  | all levels, both t | | 3037 | 2867 | **0.94** | 1.15 | 3.68 |
  | ORB cells, all levels | | 13886 | 7933 | **0.57** | 0.57 | 0.92 |

  Whole levels: ours (auto = AVX2) is 3–10% faster than `cv::FAST` at every
  level and threshold; SSE2 alone is ~15% slower than OpenCV's AVX2. The
  extractor's real workload, one call per 35-px cell (plus the threshold-7
  retry), is 1.75× faster: `cv::FAST` resizes its output to ≥ 1000 KeyPoints
  and tries HAL hooks on every call; ours reuses thread-local rows.
- The other OQ 21 items (FAST once per level instead of per cell, blind
  strips, buffers, border padding) are separate; this port keeps the
  extractor's behaviour.

## Geometry Toolkit (`estimators/`, `geometry/`)

Spec: `docs/superpowers/specs/2026-10-04-two-view-geometry-design.md` (part A of
the real monocular initializer; matching and `MapInitializer` integration are
spec B+C, now done: see "Monocular Initialization"). Pure math on
**normalized, undistorted points** (z = 1), so the calibrated model is the essential matrix E, not F + K. Sigma is in normalized
units (σ_px / f). Verified on synthetic scenes only. This is the "rays stay
rays" half of the residual-space rule (see "Camera Model and Raw-Pixel
Residuals"): two-view RANSAC, triangulation and the 85° ray test work on rays;
every reprojection residual and gate after them is in raw pixels.

| Component | Header | Summary |
|---|---|---|
| `Camera` | `geometry/camera.h` | K + `TDistortionModelPtr` (`shared_ptr<const DistortionModel>`, null = pinhole); `Project` (camera point → raw pixel), `ProjectJacobian` (2×3), `Unproject` (raw pixel → z = 1 ray). See "Camera Model and Raw-Pixel Residuals". |
| `DistortionModel` (+ Dummy, Kannala-Brandt, Brown-Conrady) | `geometry/distortion_models/*.h` | `Distort` / `Undistort` on normalized points, analytic `DistortJacobian`. Forwarding headers at `features/distortion_models/`. |
| `RigidTransformation` | `geometry/rigid_transformation.h` | Value type, hidden representation (R, t now). `T_ab` maps frame b to frame a: `p_a = T_ab * p_b`, `T_ac = T_ab * T_bc`. `Inverse()`, `Transform()`, `ToMatrix34/44()`. |
| `TwoViewCorrespondences` | `geometry/two_view_correspondences.h` | `x1`, `x2` (2xN normalized) + `sigma`. |
| `TwoViewEstimator` concept, `ModelScore` | `estimators/two_view_estimator.h` | `TModel`, `kSampleSize`, `Estimate(corr, sample)` -> 0..n candidates, `Score(model, corr)` -> score + inlier mask. |
| `HomographyEstimator` | `estimators/homography_estimator.h` | 4-point DLT (Hartley-normalized), `H_21`; score = ORB-SLAM3 `CheckHomography` (symmetric transfer, χ² < 5.991). |
| `EssentialEstimator` | `estimators/essential_estimator.h` | Normalized 8-point + projection to the essential manifold, `E_21`; score = ORB-SLAM3 `CheckFundamental` (χ² < 3.841 per direction, adds 5.991 - χ²). |
| `Ransac<TEstimator>` | `estimators/ransac.h` | Generic, header-only, deterministic (own `mt19937` from `seed`). |
| `DecomposeEssential`, `DecomposeHomography` | `geometry/two_view_decomposition.h` | 4 (R, ±t) poses from E; up to 8 Faugeras candidates (+ plane normal) from H. `‖t‖ = 1`, `det R = +1`. |
| `SelectPose` | `geometry/two_view_reconstruction.h` | ORB-SLAM3 `CheckRT` + acceptance rules. |
| `TwoViewReconstructor` | `geometry/two_view_reconstruction.h` | Fits H and E, picks the model, decomposes, selects the pose. |

**Estimator contract.** `Estimate` and `Score` are required; scores of all
estimators are on one scale (truncated χ², σ in normalized units). An estimator
may also provide `Refine(model, corr, inliers)`; `Ransac` detects it with a
`requires` expression (optional, not part of the concept). Both estimators
implement it: Levenberg-Marquardt over the inliers (E: R update + t on the unit
sphere, Sampson error; H: 8 parameters, symmetric transfer error); at most 20
accepted steps / 40 trials, early stop at relative decrease < 1e-6; a non-finite
or ill-conditioned result returns the input model.

**Ransac.** Per iteration: sample `kSampleSize` distinct indices (partial
Fisher-Yates), score every candidate, keep the best. Adaptive stop from the
inlier ratio (`confidence`, clamped to `[min_iterations, max_iterations]`).
Afterwards up to `refinement_rounds` (5) of local refinement: refine on the
current inliers, re-score, keep only strict improvements. `success = false` for
N < `kSampleSize` or no positive score.

**SelectPose / CheckRT.** For every inlier, triangulate with `P1 = [I|0]`,
`P2 = T_21`; skip non-finite points, points behind a camera (unless the point
has almost no parallax) and points with reprojection error > 4σ² in either view.
Accept the candidate with the most good points if it has
`> max(0.9 x inliers, 50)` good points, no other candidate has more than
`uniqueness x best` (E: 0.7, H: 0.75) and parallax >= 1°, the parallax being
the angle at the 51st-smallest cosine, i.e. the 51st-largest parallax angle
(index `min(50, n - 1)`, as ORB-SLAM3). Far points count as good but are not returned as
triangulated.

**Model selection (COLMAP rule).** `Reconstruct` runs `Ransac` for H and for E
(seeds `seed`, `seed + 1`) and picks H iff `inliers_H >= 0.8 x inliers_E`
(a homography explains nearly everything E does: planar scene or near-pure
rotation); otherwise E. ORB-SLAM3's score ratio `S_H / (S_H + S_E) > 0.4` does
not transfer: it relies on the *uncalibrated* F degenerating on planes, whereas
a calibrated E fits planar points as well as general ones (measured: even the
ground-truth H vs E gives a ratio of about 0.41). Both scores are still
reported. A non-positive sigma fails the reconstruction. The default narrow,
near-frontal planar scene is rejected as ambiguous by design.

**Measured accuracy limits.** On a wide-FOV general scene the
translation-direction error is up to 9.9° (about half of 21 seeds under 2°);
rotation < 1°. The cause is inlier-set lock-in, not LM convergence: LM reaches
the same answer from the true E and from the returned E on the same inlier set,
but that set is self-selected by a hard χ² threshold (3.841) — least squares
over it drops true inliers and keeps 2-3 outliers as levers. On the worst seeds
the true E scores higher than the returned model (2079 vs 1920 at seed 108,
1891 vs 1670 at 113, 2111 vs 1862 at 116); more refinement rounds (30) or a
looser re-collection threshold do not fix it. Only ~58 % of true inliers are flagged at
σ = noise (noise in both views). Planar wide-FOV: 21/21 seeds < 1° / < 2°.
Further accuracy is left to bundle adjustment right after initialization (as in
ORB-SLAM3).

## Camera Model and Raw-Pixel Residuals (`geometry::Camera`)

Spec: `docs/superpowers/specs/2026-10-05-camera-model-design.md` (branch
`feat/camera-model`).

**Why.** Two diagnostic spikes on corridor1 (2026-10-05, branch
`spike/mono-tracking-diag`, not merged) found that mono tracking lost its map
because search windows and noise models lived in *undistorted* pixel or
normalized space, where the fisheye periphery is stretched (median 5.8× at
60–85°). Only 29 % of findable points landed in the 2.5 px window, and at
60–85° `OptimizePose` rejected about half of the correct matches. ORB-SLAM3
projects, searches and measures residuals in the raw image through its camera
model; we now do the same.

**Residual-space rule.** Every reprojection residual and gate compares **raw
(distorted) pixels**, with σ = `sigma_px · 1.2^level` pixels (the extractor's
level scale); the χ² thresholds are unchanged (5.991 for 2 dof, 9.488 for
4 dof). **Rays stay rays:** two-view RANSAC, triangulation and the 85° ray
test work on undistorted normalized points (z = 1) as before.

**`geometry::Camera`** (`geometry/camera.h`, approach A: a small value type,
not camera-model classes and not free functions):
- `K` plus `distortion_models::TDistortionModelPtr distortion`
  (`std::shared_ptr<const DistortionModel>`, null = pinhole; shared by every
  copy, since 2026-10-12). A copy costs one atomic increment and decrement
  when a model is set, so hot loops take `const Camera&`: the g2o edges
  (BA, pose and Sim3 optimization) and the projection matcher's
  `ProjectionGate` refer to a camera owned by the call's arguments.
- `Project(Xc)`: camera-frame point → raw pixel, `K · [Distort(Xc.xy / Xc.z); 1]`.
  Precondition `Xc.z() > 0`.
- `ProjectJacobian(Xc)` (2×3): `K₂ₓ₂ · DistortJacobian(Xc.xy / Xc.z) ·
  [[1/z, 0, −x/z²], [0, 1/z, −y/z²]]`; `K₂ₓ₂` includes the skew.
- `Unproject(pixel)`: raw pixel → undistorted normalized ray via K⁻¹ and
  `Undistort`.

**`DistortionModel` in `geometry/`.** The models moved from
`features/distortion_models/` to `include/voyahslam/geometry/distortion_models/`
(namespace `voyahslam::geometry::distortion_models`, sources in
`src/geometry/distortion_models/`, tests in `tests/geometry/distortion_models/`;
since the restructure all three under `libs/common/`),
so `geometry` stays at the bottom of the dependency stack. The old headers are
forwarding headers (`namespace features { namespace distortion_models =
geometry::distortion_models; }`), so old includes and qualified names still
compile; in-repo users use the new path. New pure virtual `DistortJacobian(p)`
= d(distorted normalized) / d(undistorted normalized):
- Dummy: identity.
- Kannala-Brandt: with r = ‖p‖, θ = atan r, d(θ) = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ +
  k4θ⁸) and p̂ = p / r, **J = (d/r)·I + (d′(θ)·θ′(r) − d/r)·p̂p̂ᵀ**, θ′(r) =
  1 / (1 + r²), d′(θ) = 1 + 3k1θ² + 5k2θ⁴ + 7k3θ⁶ + 9k4θ⁸; J = I for r < 1e-8
  (the limit). Verified against central finite differences (Ruling U: the
  first spec draft had an extra 1/r).
- Brown-Conrady: the analytic (symmetric) 2×2 Jacobian of the OpenCV
  polynomial below.

### Camera models (lens distortion)

| Model | Class | Calibration names | Coefficients |
|---|---|---|---|
| Kannala-Brandt (fisheye) | `KannalaBrandtDistortionModel` | `equidistant` (Kalibr, EuRoC/TUM-VI) | k1..k4 |
| Brown-Conrady (radial-tangential) | `BrownConradyDistortionModel` | OpenCV standard model = ROS `plumb_bob` = Kalibr `radtan` = EuRoC `radial-tangential` | k1, k2, p1, p2[, k3] (OpenCV order; EuRoC ships 4, ROS `plumb_bob` 5) |

**Brown-Conrady** (`geometry/distortion_models/brown_conrady_distortion_model.h`,
added 2026-10-11 for EuRoC MAV). On the z = 1 plane, r² = x² + y²,
x_d = x(1 + k1r² + k2r⁴ + k3r⁶) + 2p1xy + p2(r² + 2x²) and
y_d = y(1 + k1r² + k2r⁴ + k3r⁶) + p1(r² + 2y²) + 2p2xy, exactly OpenCV's
`projectPoints` (tests compare against `cv::projectPoints` and
`cv::undistortPoints` live, plus hard-coded values; OpenCV stays in the test
target only). `Distort` returns z = 1; `Undistort` keeps the input's z;
|z| < 1e-8 columns are left unchanged. `Undistort` is Newton's method on the
analytic Jacobian from the distorted point, residual < 1e-12, ≤ 20 iterations
(OpenCV's fixed-point `undistortPoints` stops after 5 by default).

**Failed undistortion.** A point Newton does not invert (no convergence, or a
root on the folded branch where the Jacobian has det ≤ 0 or trace ≤ 0, e.g. the
mirrored preimage k1 < 0 has beyond its turning radius) is **left unchanged**:
it stays the distorted normalized point, with no flag. Kannala-Brandt behaves
alike (it keeps its best θ, or the input if tan θ is not finite). No caller
checks: `MonocularMeasurement` (→ `normalized_points()`, `points()`, used by
two-view RANSAC, triangulation, the stereo matcher and the Mapper's stereo
points), `Camera::Unproject` (→ PnP bearings) all take the result as a valid
ray. A failure therefore yields a silently wrong ray, off by the local
distortion. It only happens far outside the calibrated field of view (for
EuRoC cam0 the whole 752×480 image inverts to 1e-10), so it is accepted;
a per-point validity flag is a TODO.

**Measurements keep both** the distortion model (shared) and the raw keypoints:
`MonocularMeasurement::distorted_points()` are the keypoints as passed to the
constructor (compacted with every other array when features beyond 85° are
dropped), and `camera()` returns `{K, distortion_model}`. `points()`
(undistorted pixels) and `normalized_points()` are unchanged.
`BinocularMeasurement` uses `left().camera()`, `right().camera()` and
`rig().T_rl`. Each measurement shares ownership of its model (§6 pointer rule),
so it may outlive the app's calibration objects.

**The five consumers** (all in raw pixels):
1. Projection matcher: projects with `current.camera().Project(Xc)`; the
   bounding box and `FeatureGrid` use `distorted_points()`; window values
   unchanged (now raw px).
2. `OptimizePose`: `OptimizePose(initial_T_cw, left Camera,
   optional<StereoCameras{right, T_rl}>, observations, options)`;
   `PoseObservation::left/right` are raw pixels and `sigma` is in pixels. Mono
   e = `left.Project(T_cw·X) − z`, J = `left.ProjectJacobian(Xc)·[−[Xc]× | I]`;
   stereo rows through `right.Project(T_rl·Xc)`. A `right` observation without
   `stereo` → `success = false`. The helpers in `libs/algs/src/optimization/projection.h`
   (`ProjectPixel`, `PixelJacobians`, `ProjectPixelRight`,
   `PixelJacobianRightPose`) take a `Camera`; the normalized-plane helpers were
   removed.
3. `BundleAdjust`: `BaProblem::keyframe_cameras` (one per keyframe,
   validated); `BaObservation::measurement` in raw pixels, `sigma` in pixels;
   point block `ProjectJacobian · R`. Two-stage logic and rulings N, O, O′
   unchanged. `MapInitializer` builds observations from `distorted_points()`
   and each view's `camera()`.
4. Stereo matcher gate: triangulation stays on rays; the gate projects the
   triangulated point with each image's `camera()` and compares it with
   `distorted_points()` (`PixelChi2`, σ = `sigma_px · level_scale^level`).
5. Initialization matcher: grid on the current frame's `distorted_points()`;
   `last_matched` starts as the reference's `distorted_points()` and is updated
   in raw pixels; the 100 px window is ORB-SLAM3's raw-pixel value.

The Tracker builds `PoseObservation`s from `distorted_points()` and passes
`left().camera()` (plus `StereoCameras{right().camera(), rig().T_rl}` for
stereo); `TrackerOptions::sigma_px` keeps its meaning, now in raw pixels.

**Pinhole invariance (Ruling V).** With a null distortion every consumer's
χ², inlier decisions and results equal the old normalized code (1e-9) **only
for fx = fy**: the old code divided σ by a single f, while `OptimizePose`,
`BundleAdjust` and the stereo gate now weight x and y separately (σ in pixels
on each axis). On fx ≠ fy rigs (e.g. EuRoC) χ² changes slightly; this is the
intended behaviour.

**Spike numbers** (corridor1, mono, 400 frames, tracked frames /
re-initializations; spike code, not this branch):

| Variant | Result |
|---|---|
| Baseline (undistorted windows and σ) | 32 / 7 |
| Window and σ scaled by the Kannala-Brandt stretch | 53 / 5 |
| …plus local-map gate 20 | 108 / 5 |
| Angle-only allowance σ·1.5·(1 + r²), window ×(1 + r²) | 85 tracked |
| …plus local-map gate 20 | 99 tracked |
| Uniform σ 3 | effectively disables outlier rejection |

The calibration-free angle allowance (within 0.6 % of the Kannala-Brandt
stretch up to 60°) is recorded as a fallback for lenses without a model; it is
not implemented (TODO).

**Smoke runs on this branch** (2026-10-06, Release, `dataset-corridor1_1024_16`,
ORB, `--rate=0`, apps at debug level):

| Run | Result |
|---|---|
| Mono, 400 frames (default gate 30) | **82 tracked / 18 re-initializations** (19 maps; per map 0–14, median 4); median local-map inliers 41 (30–61); 16 losses at the local-map gate (23–29 inliers), 3 in the zero-motion step. Repeat run identical. |
| Mono, 400 frames, local-map gate 20 (temporary, measurement only) | 137 tracked / 14 re-initializations (15 maps) |
| Mono initialization | first map at about frame 118 with 71 points; the 19 maps have 62–109 points |
| Stereo, 200 frames | initialized on frame 1 with 358 points; 197 local-map successes, no loss, 1 map; median 232 inliers (68–269), was 189 |

Against the success criterion (≥ 53 tracked and ≤ 5 re-initializations) mono
passes on tracked frames and **misses on re-initializations**. The cause is
initialization acceptance, not shorter-lived maps: the initialization
matcher's 100 px window is now in raw pixels, which is effectively wider in the
fisheye periphery, so the reference frame is reset about half as often (94 vs
181 resets) and many more pairs initialize (19 maps vs 6; 11 of them from
adjacent frames, tracked for about 2.5 frames on average). Every accepted
init still passes the unchanged 1° parallax gate (1.02–2.34°). Maps from
comparable inits live as long as before; the 82 / 19 ≈ 4.3 frames per map
average (baseline 32 / 8 = 4.0, spike KB 53 / 6 ≈ 8.8) is diluted by the extra
short inits. Causal check (final review, scratch worktree): reverting only the
init matcher to undistorted pixels gives 6 maps / 56 tracked / 9.3 frames per
map, matching the spike. The branch keeps the raw-pixel init matcher: more
tracked frames (82 vs 56) at the cost of more, mostly short, maps. Every map
still dies at the 30-inlier local-map gate because nothing grows the
62–109-point initial map (sub-project 2). Three adjacent-frame maps
(≈ frames 311–318) died on their first tracked frame with 0–1 pose inliers out
of 39–47 matches; this is not diagnosed (TODO). No parameter was tuned (gate,
init acceptance and init-map tuning are in TODO).

## Optimization (`optimization/`, g2o behind Eigen types)

Spec: `docs/superpowers/specs/2026-10-05-init-bundle-adjustment-design.md`
(amended during implementation; the decisions log has the changes).

`optimization::BundleAdjust(BaProblem, BundleAdjustmentOptions) -> BaResult`
(`libs/algs/include/voyahslam/optimization/bundle_adjustment.h`) jointly refines keyframe
poses (`T_cw`, some may be fixed) and world points from **raw (distorted)
pixel observations**, each projected through its keyframe's `geometry::Camera`
(`BaProblem::keyframe_cameras`, one per keyframe; per-observation sigma in
pixels; normalized-plane residuals until 2026-10-05). The public
header uses only our types and Eigen; g2o is linked `PRIVATE` and never appears
in a public header (principle 8 applies to g2o as well). Internals: `libs/algs/src/optimization/`
(`projection.h` has the pixel projection helpers and analytic Jacobians through
`Camera::ProjectJacobian`; the g2o
edge/vertex glue lives in `bundle_adjustment.cpp`). Levenberg-Marquardt with
`BlockSolver_6_3` + `LinearSolverEigen`, points marginalized.

**Two stages like ORB-SLAM3 LocalBA:** `robust_iterations` (5) with the Huber
kernel (delta^2 = `huber_chi2` = 5.991), then edges with chi2 > `huber_chi2` or
camera-frame depth <= 0 are deactivated and `iterations` (default 10) plain
iterations run. `BaResult` has per-observation `chi2` and `inlier` (for all
observations, deactivated ones included), initial/final cost and the iteration
count. Invalid input gives `success == false` without optimizing.

**Vendored g2o.** `third_party/g2o/src/` is pristine upstream (tag
`20241228_git`, commit `eec325a1`); the local Conan recipe
`third_party/g2o/conanfile.py` (reference `g2o/20241228.1@voyah/stable` since
2026-10-10, recipe revision `.1` for `types_sim3`; user/channel set in the recipe) builds only the BSD-2-Clause core (`stuff`, `core`,
`types_slam3d`, `types_sba`, `solver_eigen`) and exposes them as components
(`g2o::core`, ...). The one-time `conan create` is part of the README build steps.
Details: `third_party/README.md`.

## Monocular Initialization (Mapper, `MapInitializer`, `InitializationMatcher`)

Spec: `docs/superpowers/specs/2026-10-04-mono-initialization-design.md`.
Verified on synthetic sequences; the app smoke run is recorded in `docs/TODO.md`.

**Pipeline** (ORB-SLAM3 `MonocularInitialization`). The Mapper feeds each
`InitializationMeasurement` to `MapInitializer::TryInitialize`:
1. A frame with more than `min_features` features becomes the **reference**
   (`last_matched` = its raw keypoints, `distorted_points()`).
2. Each later frame: `FeatureGrid` over the current frame's raw keypoints, then
   `MatchForInitialization` (ORB-SLAM3 `SearchForInitialization`: window around
   the last matched position (100 px, raw pixels), level restriction, best/second-best ratio,
   one-to-one with replacement, orientation histogram keeping the three largest
   bins). Fewer than `min_matches` matches: the current frame becomes the
   reference.
3. `TwoViewReconstructor` on the normalized matched points. On failure the
   reference **and** the matcher's updated `last_matched` positions are kept
   (ORB-SLAM3 `mvbPrevMatched`). Zero triangulated points or a non-finite /
   non-positive median depth also count as failure.
4. **Bundle adjustment** (`bundle_adjust`, default on): a two-keyframe
   `BundleAdjust` on the *unscaled* reconstruction (keyframe 1 fixed at identity,
   keyframe 2 and the points free; observations are `distorted_points()` through
   each view's `camera()`, sigma = `sigma_px * level_scale^level` pixels per
   observation). 5 + 40 iterations (baseline 1, depths ~40, so the plain stage
   needs ~40 iterations; LM stops at convergence). Points with any outlier
   observation are dropped; fewer than `min_points_after_ba` (50) survivors, or an
   unsuccessful adjustment, fail the attempt (reference kept, like any failure).
5. **Scaling:** positions and the translation of `T_21` are divided by the median
   depth of the surviving points in camera 1, so the median depth is 1.
6. **Publication order** (the Mapper sets state and map id first): a
   `KeyFrameCreated` for keyframe 1 carrying every initial point (ids from 0)
   and keyframe 1's observations, a `KeyFrameCreated` for keyframe 2 with its
   observations, then `MapCreated` (since 2026-10-06; before, a separate
   `MapPointsAdded` came first). Replicas therefore have the points before the
   keyframes that observe them.

**Conventions.** Keyframe/frame poses are `T_cw` (world -> camera,
`RigidTransformation`); the world frame is camera 1 (first keyframe =
identity). Observations travel on `KeyFrameCreated` as parallel
`observed_features` / `observed_points` vectors. Map point ids restart at 0
**per map**; `(map_id, MapPointId)` identifies a point. Frame ids stay
monotonic across maps.

**Keypoint levels.** `ImageFeatures` stores a pyramid level per keypoint
(`levels()`); each extractor defines `ScaleFactor(level)`: ORB `scale_factor^level`
(1.2). The matcher restricts candidates to the reference level
(`max_reference_level`, ORB default 0). (The OpenCV-based SIFT extractor,
with levels decoded from OpenCV's packed octave/layer, was removed on
2026-10-11.)

## Stereo Initialization (Tracker stereo matching, stereo `MapInitializer`)

Spec: `docs/superpowers/specs/2026-10-04-stereo-initialization-design.md`.
Verified on synthetic stereo frames and a TUM-VI smoke run (see `docs/TODO.md`).

**Pipeline.**
1. The Tracker thread, on dequeuing a `BinocularMeasurement`, calls
   `features::MatchStereo` (options in `TrackerOptions::stereo_matcher`) and
   stores the result with `SetStereoMatches` before the measurement is tracked
   or forwarded as `InitializationMeasurement`. Mono measurements are
   untouched.
2. `MatchStereo` is modelled on ORB-SLAM3 `ComputeStereoFishEyeMatches`:
   brute-force descriptor matching (best < 0.7 x second best), one-to-one (the
   closer claim wins), DLT triangulation of the undistorted normalized points,
   then checks: positive depth in both cameras, per-image reprojection
   chi-square (sigma scaled by pyramid level), and a parallax-cosine limit
   (`max_parallax_cos` 0.99998). The reprojection test projects the point with
   each image's `camera()` and compares raw pixels with `distorted_points()`, as
   ORB-SLAM3 does (undistorted normalized coordinates until 2026-10-05). A rectified row search does not apply to
   fisheye images; epipolar-guided search is a deferred optimization.
3. `StereoMatches` (`right_indices`, `depths`, `num_matches`) lives in its own
   header (`features/stereo_matches.h`) so that `BinocularMeasurement` and the
   matcher do not include each other.
4. Stereo `MapInitializer` (ORB-SLAM3): needs more than
   `stereo_min_features` (500) left features; every left feature with a valid
   depth becomes a map point at **metric scale** (`depth * normalized_point`,
   no median normalization); at least `stereo_min_points` (100) required. The
   result is **one** keyframe with an identity `T_cw`.
   `InitialMapPoint::feature_in_second == -1` is the sentinel for maps with a
   single keyframe. A binocular measurement drops any mono reference.
5. The Mapper publishes one `KeyFrameCreated` (the keyframe with every
   initial point) -> `MapCreated`. Point descriptors come from the left image;
   the normal is the viewing direction of the single keyframe.

**`StereoRig::T_rl`.** A `RigidTransformation` mapping left-camera coordinates
to right-camera coordinates (`p_r = T_rl p_l`); `baseline = ‖T_rl.translation()‖`.
It replaces `R_rl`/`t_rl`; the dataset reader previously computed the inverse.
Test rigs put the right camera at +x, so the translation is `(-b, 0, 0)`.

**Observation storage.** Each map keeps a mono `ObservationStorage` and a
stereo one; stereo keyframes store `depth` and `right_feature_index`
(`int16_t`) there. Both are cleared on a new map and on `TrackingLost`. Keeping
both allows mixed mono/stereo sessions later (TODO). *(Superseded
2026-10-07: the Mapper's storages are removed; the `CovisibilityIndex` is its
observation record and the depth / right index come from each Frame's
measurement — see "Map Quality".)*

**Depth limit.** `max_parallax_cos = 0.99998` rejects rays closer than about
0.36 degrees; with the 0.11 m baseline this caps stereo depth at about 17 m.
ORB-SLAM3's `KannalaBrandt8::TriangulateMatches` is stricter: it rejects
`cosParallaxRays > 0.9998` (about 1.15 degrees, a cap of about 5.5 m at
0.11 m), so our default is more permissive; tuning comes last (TODO).

**Smoke run** (TUM-VI `dataset-corridor1_1024_16`, Release, ORB):
initialized on the first frame with 355 points; median point depth 1.96 m
(quartiles 1.50 / 2.13 m, range 0.25-9.50 m).

## Tracking Against the Map (Tracker, `MapReplica`, `ProjectionMatcher`, `OptimizePose`)

Spec: `docs/superpowers/specs/2026-10-05-tracking-design.md` (sub-project 1 of
4; keyframe creation / map growth is sub-project 2, see "Keyframes and Map
Growth"; local BA and culling, relocalization are sub-projects 3-4, deferred;
the user has their own designs for them).

**Replica.** `tracking::MapReplica<FE>` (`tracking/map_replica.h`) is a
private copy of one map, touched only on its owner's thread; the Tracker's
`tracking::AtlasReplica<FE>` holds one per live map and routes the Mapper's
transactions by the replica rule (§6), and the Tracker works on the active
one. A `MapReplica` holds one map only (a `new_map` transaction clears it and
gives it its id; a transaction of another map is asserted against). It stores point position/descriptor/normal/alive,
keyframes (`T_cw`, the shared measurement, observations) and, per point, the first
keyframe observation as a `map::PointReference` (keyframe, feature, pyramid
level, distance to the camera centre). It does not count events (decision R).

**Flow per measurement in `Tracking`** (stereo matching has run on dequeue):
1. **Motion model** (when a velocity exists): predict `T_cw = velocity * T_last`,
   `SearchFromLastFrame` with a 15 px (mono) / 7 px (binocular) window scaled
   by `level_scale^level` of the last-frame feature; fewer than 20 matches
   retries with the window x2; `OptimizePose`; the step needs >= 10 inliers,
   and since 2026-10-08 >= `min_motion_model_inliers` (30) final inliers, else
   it counts as failed and zero motion runs (see "Relocalization and
   Reference-Frame Tracking", motion-model gate).
2. **Zero-motion fallback** when step 1 failed or there is no velocity: same
   with the predicted pose = `T_last` and the window x4 (the BoW-less stand-in
   for ORB-SLAM3's `TrackReferenceKeyFrame`; see limitation below).
3. **Local map** (since 2026-10-06 built from covisibility, see "Keyframes and
   Map Growth"): `SearchFromLocalMap` over the alive, unmatched, visible points
   of the local Frames (`th` = 1), then `OptimizePose` on all matches; success
   needs >= 30 inliers.
4. **Success**: `FrameTracked` (pose, inlier pairs, visible points, reference
   Frame, local Frames, stage, counts), `velocity_ = T_cur * T_last^-1`,
   `last_frame_` = this frame with its inlier matches; info log at most 1 Hz
   ("Tracking: frame N, M inliers"), debug log per step. **Failure**:
   `TrackingLost`; the debug log names the step and counts.

**Thresholds** (`TrackingOptions`): window 15 px mono / 7 px stereo; motion
model needs 20 matches (else x2 retry) and 10 inliers (30 since 2026-10-08);
zero-motion window x4 and 10 inliers; reference Frame ratio 0.7, 15 matches,
10 inliers; local-map `th` 1.0; 30 local-map inliers.

**Projection matching** (`features/projection_matcher.h`, header-only
template on the extractor; raw (distorted) pixel space: projection through
`current.camera()`, grid and bounding box on `distorted_points()`). `SearchFromLastFrame`:
project the last frame's matched points, candidates within the level-scaled
window on levels [l-1, l+1], best descriptor <= 100, one-to-one claims
(closer wins), rotation-consistency histogram (12 bins, three maxima).
`SearchFromLocalMap`: visibility (z > 0, within 85 deg, view/normal cosine >
0.5), predicted level from the reference observation, radius `th * (cos > 0.998
? 2.5 : 4.0) * level_scale^l`, levels [l-1, l], ratio test 0.8 unless the two
best are on different levels. Deviations from ORB-SLAM3: the histogram is built
from the final claims; points without a reference observation are skipped by
the local-map search; there is no scale-invariance distance check (the level is
clamped instead).

**`OptimizePose`** (`optimization/pose_optimization.h`, g2o private). Unary
edges on one `VertexSE3Expmap`: mono (2 rows, raw pixels through the left
`Camera`) and stereo (4 rows: left and right via `StereoCameras{right, T_rl}`,
with `PixelJacobianRightPose`).
Schedule as ORB-SLAM3: 4 rounds x 10 iterations, every round restarts from the
initial pose, Huber kernel in rounds 0-1 and plain after, edges classified per
round by chi2 > 5.991 (mono) / 9.488 (stereo, 4 dof) or depth <= 0; success
needs >= 3 inliers (decision S). `sigma = sigma_px * level_scale^level` pixels.

**BoW-less fallback limitation.** There is no vocabulary / place recognition,
so a lost frame cannot be tracked against the reference keyframe; the wide
zero-motion projection search replaces it. It recovers only small
frame-to-frame motion; real recovery is relocalization (sub-project 4).
*(2026-10-07: a vocabulary exists now, see "Bag of Words", but only the
Mapper uses it; the Tracker gets BoW with relocalization, roadmap step 3.)*
*(2026-10-08: with a vocabulary, reference-Frame tracking runs after
zero motion, and a failure relocalizes; see "Relocalization and
Reference-Frame Tracking". Without one, this fallback is still all there is.)*

**Measured behaviour** (TUM-VI `dataset-corridor1_1024_16`, Release, ORB,
2026-10-05; details in `docs/TODO.md` smoke note): binocular tracks all 200
frames of the smoke run (median 189 inliers). Monocular initializes at about
frame 120 but is lost within 1-11 frames, repeatedly (8 maps in 400 frames):
the initialization map has 67-94 points, so the local map drops below 30
inliers quickly, and without map growth (sub-project 2) there is nothing to
refill it. Mono translation accuracy is also limited by the initial map
(decision T).

After the camera-model change (raw-pixel residuals, 2026-10-06 smoke run; see
"Camera Model and Raw-Pixel Residuals"): binocular still tracks the whole run
(median 232 inliers); monocular tracks 82 frames in 400 with 18
re-initializations (19 maps, 62-109 points each, median 41 local-map inliers).
Maps are still short-lived for the same reason (no map growth). Map growth
arrived with sub-project 2: see "Keyframes and Map Growth" for its smoke run.

## Keyframes and Map Growth (sub-project 2: Mapper decides, `KeyFrameCreated` transaction, covisibility local map)

Spec: `docs/superpowers/specs/2026-10-06-keyframes-design.md` (amended in
place for rulings F, G, H). A `Frame` is what ORB-SLAM3 calls a KeyFrame;
event names keep "KeyFrame".

**Who decides: the Mapper.** The Tracker publishes `FrameTracked` for every
tracked measurement (§6) and decides nothing about keyframes;
`NewKeyFrameCandidate` and `PositionUpdated` are deleted. The Mapper, the only
map writer, has the authoritative observation counts, its own backlog and the
Frames the Tracker's replica has not received yet, so it decides.

**Mapper inbox.** `FrameTracked` is considered only in `Mapping` and only for
the current map. *Drain to latest:* a `FrameTracked` is skipped when a newer
`FrameTracked` **or a `TrackingLost`** of the same map follows it in the
drained batch (ruling I); all other events keep their order. *Backlog* = the
`FrameTracked` events skipped since the last evaluated one; 0 means idle.

**Signals and rule** (`mapping/keyframe_decision.h`, pure
`ShouldCreateFrame(KeyFrameSignals, KeyFrameOptions)`):

| Signal | Definition |
|---|---|
| `ref_points` | points of the reference Frame (in the Mapper's map) with >= `min_obs` observations, `min_obs` = 2 while the map has <= 2 Frames, else 3; **a stereo-matched observation counts 2** (ruling H, ORB-SLAM3 `AddObservation`) |
| `since_last` | timestamp − the last Frame's timestamp |
| `inliers` | `num_inliers` |
| `close_tracked` / `close_untracked` | stereo: stereo-matched left features with depth < 40 baselines, inside / outside the inlier pairs |
| `need_close` | stereo: `close_tracked < 100 && close_untracked > 70` |

```
c1a = since_last >= max_interval (1 s)       c1b = backlog == 0
c1c = stereo && (inliers < 0.25 · ref_points || need_close)
ratio = mono ? 0.9 : (num_frames < 2 ? 0.4 : 0.75)
c2  = (inliers < ratio · ref_points || need_close) && inliers > 15
create = (c1a || c1b || c1c) && c2
```

The reference Frame comes from `FrameTracked::reference_frame_id`, a
`std::optional` (ruling A); when it is absent or no longer in the Mapper's map
the newest Frame is used. Every evaluation is logged at debug level with all
signals and the result.

**Creating a Frame** (`Mapper::CreateFrame`, one `KeyFrameCreated`):
1. Frame id from `next_frame_id_` (never reused), pose and the copied
   measurement.
2. Tracked observations of alive points (every id checked with `IsValid`);
   each touched point gets its normal (`MeanViewingDirection`) and
   representative descriptor (`DistinctiveDescriptor`: least median Hamming
   distance; even counts take the mean of the two middle distances, ties the
   lowest index — ruling E) recomputed, published as updated points.
3. Stereo close points (binocular Frames): untracked stereo-matched left
   features by depth; every one closer than 40 baselines and at least the 100
   closest; one stereo observation each.
4. Triangulation against neighbour Frames (`mapping/triangulation.h`): the top
   30 (mono) / 10 (stereo) Frames by covisibility; a neighbour is skipped when
   mono baseline / its median scene depth < 0.01, or stereo baseline < the rig
   baseline. Matching is **brute force** over the features without a point in
   either Frame: distance <= 50, ratio 0.6, the epipolar test on rays, the
   rotation histogram (12 bins), one-to-one. **Epipolar band (ruling F):** the
   angle between Frame b's ray and the epipolar plane (normal n) must satisfy
   `asin(|n·ray_b|) <= k·σ·‖(JJᵀ)⁻¹J n‖` with J = `Camera::ProjectJacobian` at
   the unit ray, k = 3 and σ = `sigma_px · level_scale^level` px — the exact
   directional 3σ bound (σ/f at a pinhole centre), not the norm of J.
   Acceptance: DLT on rays, ray parallax cos < 0.9998 (or ORB-SLAM3's stereo
   fallback when a stereo depth has more parallax), positive depth in both,
   raw-pixel χ² < 5.991 in both, and **scale consistency (ruling G):**
   dist_b / dist_a within [1/f, f] · level_scale^(level_a − level_b), f = 1.5 ·
   level_scale (ORB-SLAM3 order). Each new point has the new Frame's
   observation and the neighbour's (`added_observations`).
5. Publish. `sigma_px`, `level_scale` and the matching thresholds live in
   `MapperOptions::triangulation` (`TriangulationOptions`, ruling B); the
   neighbour counts, `min_close_points` and the baseline ratio in
   `MapperOptions`.

The Mapper keeps a `map::CovisibilityIndex` (per point: observing Frames; per
Frame: points), fed by initialization and `CreateFrame`, readable through
`Mapper::covisibility()` on the Mapper thread / in tests only (ruling C). The
replica keeps its own, fed by `KeyFrameCreated`.

**Tracker local map** (replaces "every alive point of the replica"):
- **K1:** the Frames observing a current match; **K2:** for each K1 Frame its
  top `local_neighbours` (10) covisible Frames not yet included (ruling K:
  each K1 Frame adds up to 10, as the spec words it; ORB-SLAM3 adds only the
  first new one); cap `max_local_frames` (80), K1 first by shared points.
- **Local points:** the alive points of the local Frames
  (`CovisibilityIndex` does not purge removed points, so `IsAlive` filters).
- **Reference Frame:** after the final `OptimizePose`, the Frame sharing the
  most inliers among **all** observing Frames (ruling L; the spec said K1).
- **`visible_points`:** local points passing the projection gate, **plus every
  already-matched point** without re-projection (ruling K, ORB-SLAM3
  `SearchLocalPoints`), so `visible ⊇ matched`.
- **Fast pickup:** a `KeyFrameCreated` whose timestamp equals `last_frame_`'s
  merges its observations into `last_frame_.matches`.

**Viewer.** `SceneState` consumes `FrameTracked` (pose, trajectory, tracked and
visible point sets, local Frames) and the combined `KeyFrameCreated`. Colours:
tracked points **green**, visible-but-unmatched points **blue**, other points
**grey**; local-map Frames **orange**, other Frames blue; current camera green
(red when lost). Checked on corridor1 mono with `--viewer` (200 frames,
2026-10-07): green, blue and orange all appear.

**Knowingly deferred** (sub-project 3/4, `docs/TODO.md`): fusion, point and
Frame culling, local BA (duplicates accumulate; tracking tolerates them through
the pose outlier rejection), `ObservationsRemoved`, interruptible BA and the
busy-Mapper branch, the post-relocalization Frame guard, the Faiss vs
DBoW2/FBoW index for matching. *(2026-10-07: fusion, culling, observation
removal, local BA and its urgent interruption are done — see "Map Quality";
the busy-Mapper branch, the relocalization guard and the index remain.)*

**Synthetic end-to-end tests** (`SlamTest.{Mono,Stereo}MapGrowsWhileLeavingInitialView`):
120 frames, 6 m along +x through a 10 m room (`surround_scene`: points on the
walls, up to 1 m relief), turning 90° towards +x (`turn_deg`), so the last
frames see only a wall the first frames never saw (mono 1000 scene points;
stereo 700 with `stereo_min_features` 150, to keep the Debug run near 45 s).
Both track every frame
with no `TrackingLost`, create Frames and grow the map. The error bounds are
looser than planned (30 % of the path mono, 0.3 m stereo instead of 5 % and
2 cm): without local BA the error random-walks with the 0.5 px pixel noise
(measured 20 % / 0.15 m; 0 / 0 without noise). *(Tightened 2026-10-07 with
local BA to 0.249 m mono (4.2 % of the path) / 0.037 m stereo — see "Map
Quality".)*
ThreadSanitizer (`Tracker*:Mapper*:Slam*`, 55 tests, 2026-10-07): clean.

**Smoke (corridor1 1024, Release, ORB 1500, debug logs, headless, 2026-10-07).**
Lifetimes are tracked frames per map; the last map of a run is cut by its end.

| Run | Maps (tracked frames) | First map | Median lifetime | Re-inits | Frames created | Points | Inliers (median) | Bar |
|---|---|---|---|---|---|---|---|---|
| Mono, 400 frames | 2 (129, 142+) | 129, lost | 135.5 | 1 | 1 / 2 (beside 2 init Frames per map) | 329 / 582 | 83 / 141 | first map ≥ 150: **missed** (129); median ≥ 50: met |
| Stereo, 1000 frames (run 1) | 2 (360, 637+) | 360, lost | 498.5 | 1 | 17 / 252 (beside 1 init Frame per map) | 6 931 / 67 879 | 498 / 434 | no `TrackingLost`, one map: **missed** |
| Stereo, 1000 frames (run 2) | 3 (354, 0, 638+) | 354, lost | 354 | 2 | 16 / 0 / 251 | 6 207 / 334 / 68 833 | 491 / – / 433 | **missed**; map 2 lost on its first frame (zero motion, 5 inliers, fast rotation) |
| *Before (master, 2026-10-06)* | mono 19 maps (82 tracked in 400), stereo 1 map in 200 | | ≈ 4 (mono) | 18 (mono) | – | 62-109 per mono map | 41 mono / 232 stereo | |

Mono initializes at about frame 120 (as before); map 1 then tracks 129 frames
(about 6.5 s) — the user-visible symptom (losing the map within a few frames)
is gone, but the first-map bar is missed. Diagnosis from the decision log: of
129 evaluations on map 1, 1 created a Frame (`c2` via the ratio, `c1b`) and
128 had `c2` false. After Frame 2, `ref_points` stays at 36: the 258
triangulated points have only 2 observations (< `min_obs` 3) and nothing adds
a third until the next Frame, so `inliers < 0.9 · ref_points` holds only when
inliers fall to 32, and they dropped from 41 to 27 (< 30 gate) in one frame
while the camera rotated at 50-70 deg/s (mocap). ORB-SLAM3 avoids the stall
through fusion (`SearchInNeighbors`), sub-project 3. Stereo creates Frames
mostly through `need_close` (215 of 252 in map 2) and is lost once, at
frame ~360, by the motion model (17 matches < 20, zero motion 6 inliers)
during a 120-170 deg/s rotation: a missing reference-Frame / relocalization
fallback (sub-project 4), not map starvation (a Frame had been created on each
of the three previous frames). Mapper cost per `CreateFrame` (temporary
timer, not committed): triangulation is ≥ 98 % of it, about 25 ms per mono
neighbour (brute force, 1500 features); mono 49-76 ms with 2-3 neighbours
(0.75 s expected with 30), stereo 88 ms mean (max 124 ms) with 8.8 neighbours, about 10 ms each. A second stereo run lost map 1 at frame ~354 and map 2 right after its initialization (3 maps): the stereo result varies between runs (thread timing). ATE vs mocap: not measured —
the apps do not write a trajectory (TODO).

**Restructure re-run (2026-10-07, same apps after the library split, Release,
idle machine).** Mono 400: 2 maps (129, 142 tracked), 1 re-initialization,
3 Frames created (1 / 2), 329 / 582 points — identical to the table. Stereo
200: 1 map, 198 tracked, no loss, median 478 inliers. Stereo 1000 (two runs):
1 map, 999 / 998 tracked, no `TrackingLost`, 260 / 259 Frames created — better
than the table's runs 1 and 2, but `master` rebuilt and run the same day gives
the same (1 map, 999 tracked, 264 Frames; mono 400 identical too), so this is
the known run-to-run variation (thread timing), not a change from the split.

## Map Quality (sub-project 3: `MapTransaction`, fusion, culling, interruptible local BA)

Spec: `docs/superpowers/specs/2026-10-07-map-quality-design.md` (amended in
place for rulings A, C, D, E, H, I, J); plan
`docs/superpowers/plans/2026-10-07-map-quality.md`. Roadmap step 1.

**`MapTransaction<FE>` — the only Mapper event** (`events/map_transaction.h`;
parts in §6). Every consumer — the Mapper's own writes, the Tracker's
`MapReplica`, the viewer's `SceneState` — applies the parts in one fixed
order:

1. new map (clears the replica);
2. add Frames;
3. add points;
4. add observations;
5. update points;
6. update poses;
7. remove observations;
8. remove points (replaced losers included);
9. remove Frames (their observations go implicitly).

**Contract** (checked by `MapReplica::CheckConsistency()` in tests): after
every transaction each observation refers to an existing Frame and an alive
point, the covisibility index agrees with the observations, and every alive
point has at least one observation. **Every point the Mapper removes is listed
in `removed_points`**, including implicit ones (no observation left after
observation removals, Frame culling or BA outliers): consumers without
observation data, such as the viewer, rely on it (ruling C). **Point
references:** when the observation a point's scale prediction refers to is
removed, the reference moves to the remaining observation with the smallest
Frame id; Mapper and replica apply the same rule, computed after the
transaction's pose and point updates.

**Initialization** is one transaction: `new_map`, `start_frame`, both mono
Frames (or the one stereo Frame) and every point.

**Pipeline per created Frame** (Mapper thread; ORB-SLAM3 LocalMapping order,
ruling H):

1. the new Frame and its tracked observations (`ProcessNewKeyFrame`);
2. recent-point culling (`MapPointCulling`);
3. new points: stereo close points and triangulation (see "Keyframes and Map
   Growth");
4. fusion (`SearchInNeighbors`);
5. local BA, interruptible;
6. Frame culling (`KeyFrameCulling`).

**Transaction 1** (steps 1–4) carries the Frame, its observations, the culled
recent points, the new points and the fusion results (added observations,
replaced and removed points, updated points). **Transaction 2** (steps 5–6)
carries the BA poses and points, the outlier observations removed, the culled
Frames and the points they leave behind. The Tracker can use the new points
while BA runs.

**Observation record (ruling G).** The Mapper's `ObservationStorage`s are
removed: its `CovisibilityIndex` (per point the observing Frames and feature
indices, per Frame its points, with `Remove` / `RemovePoint` / `RemoveFrame`)
is the only observation record; a stereo observation's depth and right index
come from the Frame's measurement. This reverses the 2026-10-04 decision to
store stereo observations in an `ObservationStorage<Stereo>`.

**Visible / found counters.** Every `FrameTracked` of the current map,
including those drain-to-latest skips, adds 1 to `visible` for each of its
`visible_points` and to `found` for each inlier point.

**Recent-point culling** (`JudgeRecentPoint`, `mapping/map_quality.h`). A
point is recent for the 3 Frames after its creation. It is removed when
`found / visible < 0.25`, or when, 2 Frames after its creation, its weighted
observations (stereo counts 2) are ≤ 2 (mono) / ≤ 3 (stereo); after 3 Frames
it graduates.

**Fusion** (`features::FuseMatches`, pure, in algs; the Mapper applies its
decisions). Targets: the new Frame's top 30 (mono) / 10 (stereo) covisible
Frames plus each one's top 20. Forward pass: the new Frame's points into each
target; backward pass: the targets' points the new Frame does not observe into
the new Frame. Gate per point and Frame: not already observed; positive depth
and inside the image; ray ≤ 85°; cos(view, normal) ≥ 0.5; distance within the
reference-implied scale range ±20 % (gives the predicted level); grid search
within 3 · 1.2^level raw px at the predicted level or one below; raw-pixel
χ² < 5.991 (mono) / < 7.815 (stereo, with the right x); best descriptor
distance ≤ 50. A match on a free feature adds an observation; on a feature
with another point it merges: the point with more weighted observations wins
(lower id on a tie), the loser's observations move to the winner (dropped in
Frames that already observe it), the loser is listed in `replaced_points` and
removed. Points already replaced in this pass are skipped. Touched points get
their normal and descriptor recomputed. The Tracker remaps its `last_frame_`
matches from loser to winner.

**Frame culling** (`IsRedundantFrame`). Candidates are the new Frame's
covisible Frames, never the map's first Frame or the new one. A Frame is
redundant when ≥ 90 % of its points (stereo: only its close points) are seen
by ≥ 3 other Frames at the same or a finer level (level ≤ its level + 1).
Removing it drops its observations and removes points left with < 2.
Each candidate is judged on the map as the earlier ones left it. Culled Frames
leave an empty slot in the Mapper's Frame array (lookup by id stays O(1);
TODO).

**Local BA** (`optimization::BundleAdjust`, 5 robust + 10 plain iterations).
Optimized: the new Frame plus covisible Frames with ≥ 15 shared points,
**capped at 20 including the new Frame** (ruling I). Points: all points of the
optimized Frames. Fixed: the other Frames observing them; the map's first
Frame is always fixed when present. BA is **skipped** when no free Frame has
an observation **or no Frame would be fixed** (ruling J; ORB-SLAM3's gauge
guard — mono needs a fixed Frame). Stereo observations get a full stereo
residual (left u, v and right u) with `optimization::StereoCameras`; σ =
`sigma_px · 1.2^level` raw px from `MapperOptions::fuse` (ruling K; a second σ
source beside the Tracker's — TODO). Poses and points are always applied;
outlier observations (χ² > 5.991 mono / 7.815 stereo, or depth ≤ 0) are
removed only when `stage_reached ≥ 1`; points left without observations are
removed.

**Stopping BA (rulings A, D, E).** `BundleAdjustmentOptions::should_stop` is a
`std::function<bool()>` (null = never), evaluated **once before the first
iteration and after every iteration**; a stop before iterating leaves the
poses unchanged. `BaResult` reports `interrupted` and `stage_reached` (0 =
stopped in the robust stage, 1 = in the plain stage, 2 = finished). A stop
right after the last robust iteration but before outlier deactivation reports
stage 0, so outliers are kept (conservative). Initialization BA passes no
callback and is unchanged. A callback instead of the spec's
`const std::atomic<bool>*`: the Mapper thread itself runs BA, so nobody could
set a flag; the callback reads the inbox counter instead.

**Urgent interruption.** The Tracker publishes a `FrameTracked` with
`urgent` = `priority` = true when its final inliers are below
`urgent_inlier_factor` (1.5) × `min_local_map_inliers` (30), i.e. 45. The
Mapper's inbox counts pending priority events (§6 "Priority events");
the local-BA callback is `inbox().pending_priority() > 0`, so an urgent event
stops BA within one iteration. Ordinary `FrameTracked` never stop BA;
drain-to-latest absorbs the backlog. Only `TryPop` decrements the counter, so
the Mapper drains through it.

**Options.** `MapperOptions::map_quality` (`MapQualityOptions`: fusion
targets, recent-point window / ratio / observations, Frame redundancy, local
BA sizes and iterations) and `MapperOptions::fuse` (`FuseOptions`: the match
gate and σ), ORB-SLAM3 defaults; `TrackerOptions::urgent_inlier_factor`.

**Synthetic end-to-end tests.** `SlamTest.{Mono,Stereo}MapGrowsWhileLeavingInitialView`
bounds tightened to 1.5 × the largest of three runs with local BA: mono
0.159 / 0.129 / 0.166 m → 0.249 m (4.2 % of the 5.95 m path; 1.21 m without
local BA), stereo 0.0247 / 0.0248 / 0.0248 m → 0.037 m (0.15 m without).
`SlamTest.DuplicatesFusedInLongSequence` (stereo leave-the-view run,
0.5 px noise): ≥ 50 % of the alive points have ≥ 3 observations (78–86 % with
fusion, 43 % without) and no two alive points lie within 1 mm with
descriptors within 10 bits. With pixel noise the twins fusion leaves lie
centimetres apart (about 18 pairs within 10 bits remain; 112 without fusion),
so the 1 mm test only catches exact copies (TODO). ThreadSanitizer
(`voyah_slam_tests` 113 tests, `voyah_integration_tests` 11, 2026-10-07):
clean.

**Smoke (corridor1 1024, Release, ORB 1500, debug logs, headless, real-time
playback, 2026-10-07; two runs each).** Feature extraction (69 ms mono /
149 ms stereo per frame) is slower than the 50 ms frame gap, so playback runs
behind real time, as in earlier smoke runs. ATE: `tools/ate.py` on the
`--trajectory_out` file against `mav0/mocap0/data.csv` (20 ms association,
Umeyama; the mocap covers only the first 26.8 s, the room). Frames
created / culled are after initialization; points created = triangulated +
stereo close points.

| Run | Maps (tracked) | Lost | Frames created / culled | Points created / recent-culled / merged | Local BA ms mean / median / max, interrupted | Mapper ms per pass mean / max | ATE | Must |
|---|---|---|---|---|---|---|---|---|
| Mono 400, run 1 | 1 (282: frames 118–399) | 0 | 64 / 41 (max gap 10 frames) | 28 034 / 6 490 / 18 572 | 104 / 120 / 157, 0 | 255 / 413 | Sim3 0.033 m (282 / 282 matched, scale 1.856) | **met** |
| Mono 400, run 2 | 1 (281: 118–398) | 0 | 68 / 45 (max gap 13) | 29 378 / 6 505 / 19 763 | 105 / 121 / 141, 0 | 249 / 401 | Sim3 0.032 m (281 / 281) | **met** |
| Stereo 1000, run 1 | 1 (999: 1–999) | 0 | 297 / 89 | 107 971 / 34 530 / 57 356 | 134 / 138 / 245, 1 (stage 1) | 251 / 405 | SE3 0.036 m (536 / 999) | **met** |
| Stereo 1000, run 2 | 1 (999) | 0 | 280 / 73 | 102 481 / 34 463 / 52 637 | 133 / 136 / 215, 1 (stage 1) | 259 / 380 | SE3 0.037 m (536 / 999) | **met** |
| *Before (keyframes table above)* | mono 2 (129, 142+); stereo 1–3 | mono 1; stereo 0–2 | mono 3; stereo 260–270 | no culling or fusion | – | – | not measured | |

Mono initializes at frame 118 (as before) and now keeps that map to the end
of the run: every frame after it is tracked, a Frame is created at least every
13 frames, and 41–45 of the 64–68 created Frames are culled as redundant.
Mono run 2 did not track frame 399: the measurement was still queued when the
app stopped the Tracker (known, `Tracker::Stop()` TODO), not a loss. Every
local BA ran (none skipped); median 19 free / 1 fixed Frames mono, 20 / 36–39
stereo. Only two `FrameTracked` per run fell below the urgent threshold (45),
so BA was interrupted once per stereo run and never in mono. A Mapper pass
(create + fuse + BA + Frame culling) costs about 250 ms, of which triangulation
is about 130 ms (mono) and local BA about 105–135 ms, so drain-to-latest
evaluates about 60 % of the `FrameTracked` (mono 164–187 of 281). Fusion
merges about two thirds of the triangulated points (mono) / half of all new
points (stereo).

## Bag of Words (`bow/`, roadmap step 2)

Spec: `docs/superpowers/specs/2026-10-07-bow-design.md` (amended in place for
rulings B, D, F); plan `docs/superpowers/plans/2026-10-07-bow.md`.

**Module** (`voyah_algs`, `bow/`, namespace `voyahslam::bow`, no OpenCV): an
OpenCV-free port of DBoW2's `TemplatedVocabulary` (ORB-SLAM3
`Thirdparty/DBoW2`), TF-IDF weighting and L1 scoring only.

| Type | Header | Role |
|---|---|---|
| `WordId`, `NodeId`, `BowVector`, `FeatureVector`, free `Score` | `bow/bow_vector.h` (in `voyah_common` since 2026-10-08, ruling D: plain data the events carry; same include path) | `BowVector`: (word, weight) sorted by word, L1-normalised. `FeatureVector`: feature indices grouped by node in CSR form (`nodes` sorted, `offsets`, `features`). |
| `Vocabulary<FE>` | `bow/vocabulary.h` (I/O in `src/bow/vocabulary_io.cpp`) | `LoadText`, `LoadBinary`, `SaveBinary`, `FromTree`, `Transform(descriptors, levels_up, out_bow, out_features)`, `Score`, `num_words()`, `k()`, `L()` |

- **Generic over the extractor:** descriptors are `FE::TDescriptors` (one
  column per node), distances `FE::Distance`. It compiles for every
  `INSTANTIATE_CLASS` extractor (ruling A; only ORB since SIFT's removal on
  2026-10-11); the loaders reject a descriptor size that does not match `FE`
  at run time.
- **Storage:** nodes in id order as flat arrays (descriptor, parent, weight);
  children (CSR) and word ids are derived from the parents at load. Node 0 is
  the root.
- **Transform:** each descriptor descends from the root to the closest child
  (ties: first child in stored order) down to a leaf, its word; each
  occurrence adds the word's idf weight, words of weight 0 are skipped (DBoW2),
  then the vector is L1-normalised. The FeatureVector files the feature under
  the node its descent passes at depth `L − levels_up` (DBoW2's definition): the
  root when `levels_up ≥ L`, and **the leaf itself when the leaf is shallower
  than that** (ruling B; DBoW2 leaves the node uninitialised there, which is
  UB). ORBvoc (k 10, L 6) has no leaf shallower than depth 4, so this never
  applies with `levels_up` 4.
- **Score:** DBoW2 `L1Scoring`, `Σ (|a| + |b| − |a − b|) / 2` over shared
  words, in [0, 1].
- **Loaders never throw:** on any malformed or inconsistent file (other
  scoring / weighting types, `is_leaf` disagreeing with childlessness, a wrong
  size) they log the reason and return `std::nullopt`.
- **Equivalence with DBoW2:** `VocabularyEquivalenceTest` (fixture
  `libs/algs/tests/bow/data/dbow2_reference.bin`, made by the original DBoW2
  on 3 corridor1 frames; `tools/dbow2_reference/README.md`) matches exactly:
  the same words, the same feature filing, and bit-identical weights.

**Text format** (DBoW2 / ORB-SLAM): a header `k L scoring weighting`, then one
line per node from id 1: `parent is_leaf d0 … d31 weight`. Trailing blank
lines are ignored. DBoW2's loader reads one more, empty line at the end of
`ORBvoc.txt` and creates a phantom leaf from uninitialised values (UB): DBoW2
has 971 815 words, ours 971 814. **Our tree omits the phantom** (ruling D); a
feature DBoW2 would drop at that node gets a real sibling's word with us.

**Binary format** (ours, little-endian):

```
magic "VOYBOW\0\1" | u32 k | u32 L | u32 descriptor_bytes | u64 node_count
| u32 parent[node_count] | u8 descriptor[node_count][descriptor_bytes] | f64 weight[node_count]
| u64 checksum (FNV-1a 64 of all preceding bytes)
```

A wrong magic or descriptor size, truncation, trailing bytes or a bad checksum
fails the load. `tools/convert_vocabulary <in.txt> <out.bin>` converts, then
reloads and verifies that the trees are identical. **The vocabulary file is
never committed** (`ORBvoc.txt` licence: TODO).

**Load times** (Release, ORBvoc: k 10, L 6, 1 082 073 nodes, 971 814 words):
`LoadText` of the 145 MB text 0.51–0.55 s (DBoW2's `loadFromTextFile`
2.7 s); `LoadBinary` of the 47.6 MB binary 0.13–0.14 s; `convert_vocabulary`
end to end 0.75–0.77 s.

**Ownership.** The app loads the vocabulary once (`--vocabulary <file.bin>`,
binary only) and passes a `std::shared_ptr<const bow::Vocabulary<FE>>` to
`Slam`, which passes it to the Mapper. It is immutable after load, so threads
may share it read-only without locks (principle 1 is about mutable map data).
The pointer may be null: then nothing computes a bag of words and the Mapper
behaves exactly as before.

**Mapper.** Frames are kept in a private `KeyFrameRecord { frame; bow;
features; }` (ruling G): `map::Frame` is unchanged. The bags of words were
never published until 2026-10-08; since then every added Frame carries them in
its `MapTransaction` (see "Relocalization and Reference-Frame Tracking"). With a vocabulary, `AddKeyFrame` (initial Frames included)
runs `Transform` on the left descriptors with `kBowLevelsUp` = 4 (ORB-SLAM3);
about 5–8 ms per Frame. `BowVectorOf(frame)` exposes it to tests.

**`SearchByBoW` for triangulation.** `mapping::SearchForTriangulation` takes
two optional `const bow::FeatureVector*`. With both, candidate pairs come only
from features filed under the same node in both Frames (the sorted node lists
are walked side by side); every other check is unchanged (no point yet,
distance ≤ 50, ratio 0.6, epipolar band, rotation histogram, one-to-one).
The matches are therefore **drawn only from shared-node pairs**; they are not
in general a subset of brute force's, because fewer candidates can let a
match pass the ratio test that brute force rejected (ruling F). With every
feature under one node the result equals brute force exactly (tested, and the
Mapper's one-node-vocabulary test gives transactions identical to the
null-vocabulary run, ruling H). Without FeatureVectors brute force runs as
before. Fusion and the Tracker's projection matching are unchanged; the
Tracker uses BoW since 2026-10-08 (reference-Frame tracking and
relocalization).

**ORB rounding.** The rotated BRIEF offsets use `std::lround` (ORB-SLAM3's
`cvRound`) instead of truncation, so our descriptors are bit-compatible with
ORB-SLAM3's vocabulary.

**Tests.** Unit tests on hand-built vocabularies (k 3, L 2 and others):
transform, ties, weights, filing depth, score, binary round trip and
rejections, text fixtures (one with trailing newlines); `SearchByBoW` against
brute force; Mapper BoW per Frame, the BoW path taken, the one-node
equivalence. The equivalence test has ctest label **`vocabulary`** (ruling E):
`voyah_add_test_binary` accepts it, `check` (`-L unit`) excludes it,
`check_all` runs it and it **skips unless `$VOYAHSLAM_ORBVOC`** points to
`ORBvoc.txt`.

**Smoke (corridor1 1024, Release, ORB 1500, debug logs, headless, real-time
playback, 2026-10-07; each configuration twice, plus a third stereo run with
the vocabulary).** Same apps and flags as the "Map Quality" table, with and
without `--vocabulary ORBvoc.bin`. Triangulation time is
`TriangulateWithNeighbours` (all neighbours: matching plus triangulation),
measured with a temporary timer that was not committed; the BoW transform
(about 7 ms per Frame) is outside it and inside "create". Feature extraction
ran about 13 % slower on the day than in the map-quality runs (78 vs 69 ms
mono), so compare against the "without" rows.

| Run | Maps (tracked) | Lost | Frames created / culled | Points created / recent-culled / merged | Triangulation ms per Frame mean (median / max) | Local BA ms mean / median / max, interrupted | Mapper ms per pass mean / max | ATE | Must |
|---|---|---|---|---|---|---|---|---|---|
| Mono 400, without, run 1 | 1 (278: frames 122–399) | 0 | 62 / 40 | 26 683 / 6 302 / 17 448 | 178.3 (178.7 / 313.7) | 101 / 117 / 139, 0 | 292 / 455 | Sim3 0.034 m (278 / 278) | |
| Mono 400, without, run 2 | 1 (277: 122–398) | 0 | 59 / 37 | 25 255 / 6 119 / 16 215 | 182.4 (183.3 / 316.7) | 100 / 117 / 137, 0 | 294 / 450 | Sim3 0.034 m (277 / 277) | |
| Mono 400, **with**, run 1 | 1 (278: 122–399) | 0 | 81 / 56 | 28 281 / 5 411 / 19 585 | **6.8** (7.0 / 13.4) | 115 / 127 / 204, 0 | 143 / 243 | Sim3 0.031 m (278 / 278) | **met** (26×) |
| Mono 400, **with**, run 2 | 1 (278: 122–399) | 0 | 81 / 55 | 28 438 / 5 443 / 19 660 | **6.5** (7.0 / 9.8) | 113 / 128 / 174, 0 | 140 / 212 | Sim3 0.033 m (278 / 278) | **met** (28×) |
| Stereo 1000, without, run 1 | 1 (998: 1–998) | 0 | 280 / 70 | 104 569 / 36 602 / 52 832 | 149.0 (151.9 / 252.5) | 126 / 130 / 206, 2 (stage 0) | 287 / 443 | SE3 0.037 m (536 / 998) | |
| Stereo 1000, without, run 2 | 1 (998) | 0 | 288 / 73 | 106 927 / 36 375 / 54 766 | 146.0 (151.9 / 228.9) | 128 / 132 / 202, 1 (stage 0) | 286 / 386 | SE3 0.038 m (536 / 998) | |
| Stereo 1000, **with**, run 1 | 1 (998) | 0 | 269 / 59 | 80 732 / 30 711 / 35 112 | **3.8** (4.0 / 6.0) | 117 / 125 / 220, 0 | 138 / 245 | SE3 **0.189 m** (536 / 998) | **met** (39×); ATE worse |
| Stereo 1000, **with**, run 2 | 1 (999) | 0 | 261 / 60 | 76 895 / 28 536 / 34 018 | **3.7** (3.9 / 5.1) | 116 / 125 / 214, 0 | 137 / 239 | SE3 **0.056 m** (536 / 999) | **met** (39×); ATE worse |
| Stereo 1000, **with**, run 3 (extra) | **2** (324 + 673) | **1** (frame 325) | 283 / 58 | 85 437 / 31 784 / 37 071 | 3.7 (3.9 / 7.5) | 113 / 123 / 227, 0 | 134 / 253 | SE3 0.206 m (map 2) | missed (loss) |

- **Speed:** triangulation per Frame is 26–28× faster (mono) and 39× faster
  (stereo); with the BoW transform counted it is still about 12× (mono) /
  14× (stereo). A Mapper pass halves (about 290 → 140 ms), so drain-to-latest
  evaluates more `FrameTracked` (mono 241–242 of 278 instead of 163–171;
  stereo 993–999 instead of 844–866).
- **Fewer matches per pair:** BoW finds fewer triangulation matches (stereo
  median 145 vs 204 new points per Frame; first mono Frame 291 vs 365), as
  expected from restricting candidates to shared nodes. In mono this is offset
  by more Frames (81 vs 59–62, with more culled); mono initializes at frame
  122 in all runs (118 in the map-quality runs, before the ORB rounding
  changed the descriptors) and tracks to the end. Both
  mono BoW runs created no Frame for 27 frames early in the map (dataset
  136–163; 11 without), while tracking stayed above 400 inliers.
- **Stereo accuracy regressed:** SE3 ATE 0.056–0.206 m with the vocabulary
  vs 0.037–0.038 m without (0.036–0.037 in the map-quality table); Sim3
  scale 0.91–0.97 vs 0.97. The error grows over the run (run 1: 0.07 m in the
  first 50 matched poses, 0.30 m at 400–500), i.e. drift, not an offset.
  Tracking itself is stronger (median inliers about 400, minimum 190–216 vs
  38–52 without). Nothing was tuned. The final review ruled out
  fewer triangulated points as the cause: BoW with a 150 ms sleep after
  triangulation gives ATE 0.037 / 0.037 m with 157–161 points per Frame;
  restricting BoW matches to those brute force also accepts gives
  0.036 / 0.037 m with only 116 points; turning the ratio test off keeps the
  Mapper fast and gives 170 points but ATE 0.050 / 0.235 m. The regression
  follows Mapper speed: a faster Mapper exposes timing-dependent behaviour
  elsewhere, not a BoW matching defect. Also ruled out: Frame/FeatureVector
  pairing, the level-4 filing depth, the left-descriptor choice, and BoW
  reused after a slot is reused. Our descriptors are as compatible with
  ORBvoc as ORB-SLAM3's own extractor's (60.4 % vs 61.1 % of matches share a
  depth-2 node). See TODO.
- **The extra stereo run lost tracking at frame 325**, the known fast
  rotation (120–170°/s, TODO "Stereo loss at fast rotation"): motion model 64
  then 41 matches, local map 23 inliers (< 30). The runs without the
  vocabulary dip to 38 and 52 inliers at frames 325 / 355, so the margin was
  already thin; the Tracker does not use BoW.

## Relocalization and Reference-Frame Tracking (roadmap step 3, sub-project 4)

Spec: `docs/superpowers/specs/2026-10-07-relocalization-design.md` (amended
in place for rulings C, E–G, F and I); plan
`docs/superpowers/plans/2026-10-08-relocalization.md`; the stereo-drift spike
`.superpowers/spikes/2026-10-07-stereo-drift.md`. Goal: a short tracking
loss is recovered **into the same map** instead of clearing it. Relocalization
into other maps waits for multi-map (roadmap step 5).

**State machine** (`TrackerState`, ORB-SLAM3 visual-only):

```
Tracking ─(motion model, zero motion and reference Frame fail)→ RecentlyLost
RecentlyLost: relocalize every measurement
   success → Tracking (same map; velocity reset; the matched Frame is the reference;
             1 s of the post-relocalization guard, ruling J)
   timeout → Lost
Lost: TrackingLost(final = true) → the Mapper clears its map (one transaction,
      removed_maps = {it}, no active map) → initialization (as before)
```

- **`RecentlyLost`** needs a vocabulary and a map with more than
  `min_map_frames_to_relocalize` (10) Frames; otherwise a failure goes
  straight to `Lost` (as before without a vocabulary). The
  timeout is `recently_lost_seconds` (5.0) of **measurement timestamps**, not
  wall-clock time (backwards timestamps never time out: TODO).
- While `RecentlyLost` the Tracker publishes **no `FrameTracked` and no
  `InitializationMeasurement`**; empty measurements still advance the timeout.
- **`TrackingLost::final`.** `false` is published on entering
  `RecentlyLost`: the Mapper ignores it, keeps its map, and the viewer shows
  "recently lost". `true` is published on entering `Lost`; only this one
  clears the Mapper's map. A non-final `TrackingLost` does not supersede an
  earlier `FrameTracked` of the same map in the Mapper's drained batch (ruling
  H: the map survives, so the last tracked frame is still evaluated); a final
  one does (decision I of 2026-10-06).
- Options: `TrackerOptions::relocalization` (`RelocalizationOptions`,
  `tracking/relocalizer.h`) holds the loss protocol and every §4 value below;
  the §3 reference-Frame values live in `TrackingOptions` (ruling B, amended).

**`TrackingStage`** (ruling C). `FrameTracked::stage` reports the **source of
the initial pose** that the local-map step refined: `MotionModel`,
`ZeroMotion`, `ReferenceFrame` or `Relocalization`. The old `LocalMap` value
is gone: the local-map step always runs last, so it said nothing.

**BoW in `MapTransaction`.** Each added Frame (`events::NewFrame`) carries
`bow::BowVector bow` and `bow::FeatureVector features`, filled by the Mapper
from its per-Frame computation (`levels_up` 4) and empty without a
vocabulary. The Tracker's `MapReplica` stores them per Frame. The Tracker gets
the same immutable `std::shared_ptr<const bow::Vocabulary<FE>>` as the Mapper
through `Slam`, and uses it only for the current frame at loss time
(reference Frame, relocalization). `bow_vector.h` (plain data and the free
`bow::Score`) moved down to `voyah_common` at the same include path so the
events can carry it (ruling D; ruling A for the free `Score`). `Slam` copies
the vocabulary pointer into the Tracker before the Mapper takes it, which
relies on `tracker_` being declared before `mapper_` (fragile: TODO).

**`bow::FrameDatabase`** (`voyah_algs`, `bow/frame_database.h`,
non-templated): an inverted index word → Frames that keeps each Frame's
`BowVector`, so a removal edits exactly its word lists. The replica owns one
and keeps it in sync with its transactions (`Add` on an added Frame with a
non-empty BoW, `Remove` on a removed Frame, `Clear` on a map switch).
`Query(bow, min_common_ratio)` returns the Frames sharing at least one word
and at least 0.8 × the best Frame's common words, with their L1 score
(`bow::Score`), by score descending, ties by ascending `FrameId`. A removed
Frame is never returned. Single-threaded: each owner has its own copy, as
every other replica structure.

**Reference-Frame tracking** (ORB-SLAM3 `TrackReferenceKeyFrame`; vocabulary
only). After zero motion fails: the current frame's BoW (`levels_up` 4),
`features::SearchByBoW` against the last `FrameTracked`'s reference Frame
(ratio 0.7, fewer than 15 matches fail), `OptimizePose` from the last pose
(≥ 10 inliers after dropping outliers), then the usual local-map step and its
30-inlier gate; stage `ReferenceFrame`. A reference Frame the Mapper has
culled makes it fail (ORB-SLAM3 follows the parent: TODO).

**`features::SearchByBoW`** (`voyah_algs`, `features/bow_matcher.h`;
ORB-SLAM3 `SearchByBoW(KeyFrame*, Frame&)`): current features against one
Frame's features that observe a live point, only under shared
`FeatureVector` nodes; best distance ≤ 50, best < ratio × second best,
rotation histogram (12 bins, top 3), one-to-one (an already matched current
feature is skipped, as ORB-SLAM3). Returns current feature → `MapPointId`.

**`geometry::SolvePnPRansac`** (`voyah_algs`, `geometry/pnp.h`). Minimal
solver: Lambda Twist P3P (Persson & Nordberg 2018) on unit bearing rays from
`Camera::Unproject`, so fisheye works without undistortion; a 4th point picks
among the up-to-4 solutions. RANSAC: up to 300 iterations, confidence 0.99
(adaptive stop), ≥ 10 inliers; inliers by raw-pixel χ² < 5.991 through
`Camera::Project` with σ = `sigma_px · 1.2^level`, depth ≤ 0 is an outlier.
Output: `T_cw`, inlier mask, success; degenerate input (fewer than 4 points,
exactly collinear points, no valid solution) fails cleanly, never NaN. It is a
**raw hypothesis generator**: accuracy comes from the refinement (ruling F).
Test contracts (rulings E–G, spec §5 amended): Lambda Twist recovers 3
noise-free rays to 1e-9; noise-free fisheye up to 80° to 1e-5 in all 50
seeded scenes (the `Unproject` tolerance bounds it); with 50 % outliers and
1 px noise, over 200 seeded scenes, ≥ 95 % succeed with an inlier mask ≥ 90 %
correct and ≥ 95 % are within 1 cm / 0.5° **after `OptimizePose` over all
matches** from the RANSAC pose (measured 199/200 and 198/200). Raw RANSAC
poses meet 1 cm / 0.5° in only about 34–59 % of such scenes, so no test
depends on a lucky seed.

**Relocalization** (`tracking/relocalizer.h`, `Relocalize`; ORB-SLAM3
`Relocalization`), on every measurement while `RecentlyLost`:
1. The current frame's BoW, queried in the replica's database (common ≥
   0.8 × max); hits whose Frame is gone are skipped.
2. Candidates grouped by covisibility: a hit's group score is its own plus
   those of its top 10 covisible Frames that are hits too; groups ≥ 0.75 ×
   the best keep their best-scoring Frame.
3. `SearchByBoW` against each candidate (ratio 0.75); fewer than 15 matches
   drop it.
4. Rounds of P3P RANSAC: each round gives every live candidate up to 5 more
   iterations, as a fresh `SolvePnPRansac` call with its own seed (from the
   measurement timestamp, the Frame and the round; deterministic), until 300.
5. **Refinement (ruling F, spec §4.5 step 2 amended):** a RANSAC pose with
   ≥ 10 inliers is refined by `OptimizePose` over **all** the candidate's
   SearchByBoW matches, starting from the RANSAC pose (its robust rounds
   reject the outliers), not only over the RANSAC inliers; fewer than 10
   survivors drop the candidate. Measured on half-outlier scenes: 97.6 %
   within 1 cm / 0.5° this way, 93 % refining the inliers only.
6. **Projection passes (ruling I, spec §4.5 amended):** below 50 inliers the
   candidate Frame's other points are projected with
   `features::SearchFromFrame`, **ORB-exact** (`SearchByProjection(Frame&,
   KeyFrame*, set, th, ORBdist)`): fixed radius `th · 1.2^level` px over levels
   [l−1, l+1], best distance ≤ `ORBdist`, **no ratio test**, first claim wins,
   rotation histogram; `th` 10 / distance 100, optimize; then, when 30 < n <
   50, `th` 3 / distance 64, optimize again. The spec's 10 / 100 and 3 / 64
   are ORB-SLAM3's semantics; the local-map search's ×2.5–4 window and 0.8
   ratio test (an earlier draft) are not used here.
7. The first candidate reaching **50 inliers** wins; then the local-map step
   runs (its 50-inlier post-relocalization gate decides, below) and Tracking
   resumes in the same map with stage `Relocalization`, the velocity reset
   and the matched Frame as the reference. The debug log prints every
   attempt's candidates, candidates with BoW matches, hypotheses and best
   verified inliers.

**Post-relocalization guard (ruling J, ORB-SLAM3's `mnLastRelocFrameId +
mMaxFrames`; final review Important 1).** A relocalized pose is the least
confirmed of the run, and the Mapper's Frame rule would almost always make it
a Frame (`since_last ≥ 1 s` after any loss of 1 s or more; in stereo
`need_close` then places about 250 close points from it). Two sides:
- **Tracker:** for `post_relocalization_seconds` (1.0) of measurement
  timestamps from the relocalized measurement on, that one included, the
  local-map step needs `post_relocalization_min_inliers` (50) instead of
  `min_local_map_inliers` (30) (`RelocalizationOptions`). A new map resets it.
- **Mapper:** no Frame from a `FrameTracked` within
  `relocalization_guard_seconds` (1.0) of the latest one with stage
  `Relocalization`, that one included, superseded ones too
  (`KeyFrameOptions`; logged at debug level). As ORB-SLAM3 (`nKFs >
  mMaxFrames`, `mMaxFrames` = fps), it applies only while the map has more
  than `relocalization_guard_min_frames` (20) Frames: a smaller map must keep
  growing. Without that condition stereo corridor1 (18 Frames at the
  recovery) lost tracking again 12 frames after recovering in 2 of 2 runs (the
  camera left the relocalized view with no new Frame; inliers 148 → 43, below
  the 50 gate), then a second map. A map clear resets the guard.

**Motion-model gate** (spec §0, the spike's fix). `min_motion_model_inliers`
= 30: a motion-model pose with fewer final inliers counts as failed, so zero
motion (whose minimum stays 10) and then the reference Frame run. The spike
found the stereo "drift" with `--vocabulary` was not drift: in the fast
rotations (corridor1 frames 324–378) the motion model was accepted with
10–29 inliers, `need_close` made the wrong pose a Frame with ~320 close
points, and with BoW the Mapper published it before the next frame was
tracked, so the Tracker locked onto the misplaced points (a jump or a loss).
With the gate, stereo 1000 with the vocabulary measured 0.0361 / 0.0363 /
0.0369 m in the spike and 0.0361 / 0.0358 m at Task 1 (was 0.056 / 0.189 m
and losses), one map each; mono 400 unchanged (0.0315 m).

**Deviations from ORB-SLAM3** (all in TODO): RANSAC is more permissive than
ORB's effective settings (ORB: minimum inliers max(10, 0.5 · N), about 35
iterations from ε 0.5; ours 10 and 300); the post-relocalization guard
(ruling J) counts 1 s of measurement timestamps, not `mMaxFrames` (= fps)
frames;
`SearchFromFrame` has no scale-distance (min / max invariance distance) gate;
the RANSAC state is not kept between rounds (fresh calls); a near-collinear
PnP sample can pass RANSAC (only exact collinearity is rejected; the
50-inlier verification is the guard).

**Apps.** `--drop_frames a:b` feeds frames a..b (inclusive, 0-based) as
measurements without features at their own timestamps, to simulate a
blackout.

**Tests.** `voyah_algs`: `FrameDatabase`, `SearchByBoW`, `SolvePnPRansac`
(above). `voyah_slam`: the motion-model gate, reference-Frame tracking of a
synthetic jump, RecentlyLost and `TrackingLost(final = false)` on a blackout
with the Mapper's map kept, relocalization into the same `map_id`, the 5 s
timeout (`final = true`, map cleared), small maps straight to Lost, no
vocabulary unchanged, database consistency after Frame removals, aliased
places rejected at BoW / RANSAC / the 50-inlier gate, candidate grouping,
the post-relocalization guard (`RelocalizationTest.StricterGateAfterRelocalization`;
`MapperRelocalizationGuardTest.*`, `MapperFrameCreationTest.NoRelocalizationGuardOnSmallMap`).
Integration: `SlamTest.{Mono,Stereo}RelocalizesAfterBlackout` run the
threaded `Slam` on the leave-the-view sequence with a 1 s blackout (frames
31–50, right after the Mapper's latest Frame) and a one-node vocabulary
(`min_map_frames_to_relocalize` 0: the synthetic map has 3–5 Frames): one
non-final `TrackingLost`, map 1 only, the first frame tracked after the
blackout has stage `Relocalization`, and the centre error from the recovery
on stays within 1.5 × the largest of three runs (mono 0.0931 m, recovery at
frame 52; stereo 0.0217 m, recovery at frame 51).

**Smoke (corridor1 1024, Release, ORB 1500, `--vocabulary ORBvoc.bin`,
`--rate=0`, debug logs, headless, sequential, 2026-10-08; two runs each).**
Recovery = the first frame tracked after the drop (stage `Relocalization`:
the only way out of `RecentlyLost`). "Reloc" = successful relocalizations
(Frame id, inliers); no run used reference-Frame tracking successfully (it
fails on the first empty frame of a drop, as it should). ATE by
`tools/ate.py` (mono Sim3, stereo SE3; matched poses / tracked).

| Run | Maps | Tracked | RecentlyLost / Lost | Recovery frame | Reloc | Reference-Frame successes | ATE | Must |
|---|---|---|---|---|---|---|---|---|
| Mono 400, drop 250–270, run 1 | 1 | 256 (122–398) | 1 / 0 | **271** | Frame 9, 136 | 0 | Sim3 0.0325 m (256 / 256) | **met** |
| Mono 400, drop 250–270, run 2 | 1 | 256 (122–398) | 1 / 0 | **271** | Frame 9, 139 | 0 | Sim3 0.0336 m (256 / 256) | **met** |
| Stereo 1000, drop 500–520, run 1 | 1 | 978 (1–999) | 1 / 0 | **521** | Frame 0, 81 | 0 | SE3 0.0351 m (515 / 978) | **met**, no other loss |
| Stereo 1000, drop 500–520, run 2 | 1 | 975 (1–999) | 1 / 0 | **524** (521–523: 1 candidate, best 3 / 9 / 0 inliers) | Frame 13, 99 | 0 | SE3 0.0349 m (512 / 975) | **met**, no other loss |
| Mono 400, no drop, run 1 | 1 | 278 (122–399) | 0 / 0 | – | – | 0 | Sim3 0.0324 m (278 / 278) | **met** (≈ 0.032) |
| Mono 400, no drop, run 2 | 1 | 277 (122–398) | 0 / 0 | – | – | 0 | Sim3 0.0323 m (277 / 277) | **met** |
| Stereo 1000, no drop, run 1 | 1 | 999 | 0 / 0 | – | – | 0 | SE3 0.0365 m (536 / 999) | **met** (≤ 0.04) |
| Stereo 1000, no drop, run 2 | 1 | 999 | 0 / 0 | – | – | 0 | SE3 0.0363 m (536 / 999) | **met** |

- **Recovery is immediate** in 3 of 4 drop runs (the first frame after the
  drop) and 3 frames later in the fourth; both drop ranges are in mapped
  parts of the run (mono: the map had Frames there; stereo run 1 matched the
  map's first Frame, Frame 0), so nothing recovered in unseen corridor. The
  ATE with a drop equals the no-drop ATE within 0.001–0.002 m (fewer poses).
- **Time per relocalization attempt** (a separate timed run each, with a
  temporary timer that was not committed): an attempt that succeeded took
  10.3 ms (mono, 82 inliers) and 9.3 ms (stereo, 86 inliers), BoW transform
  included; an empty measurement 0.000 ms. The failed real attempts (stereo
  run 2, frames 521–523) were not timed.
- The motion-model gate fired 2–3 times per stereo run (15–23 inliers;
  zero motion took over each time), never in mono; no run lost tracking without a
  drop. Stereo no-drop ATE 0.036 m (was 0.056 / 0.189 m with the vocabulary
  before the gate).
- **With ruling J** (same setup, one run each, 2026-10-08): mono drop
  recovers at 271 (Frame 9, 139 inliers) into map 1, 21 Frames, so the guard
  applies and **no Frame is created within 1 s** of the recovery (21 skips
  logged); Sim3 0.0333 m. Stereo drop recovers at 523 (Frame 0, 77 inliers)
  into map 1 with no other loss; the map has 18 Frames, so the guard does not
  apply at first: the recovered frame becomes a Frame (as in ORB-SLAM3, 263
  close points), 3 more follow in 0.55 s until the map passes 20 Frames, and
  the remaining 8 `FrameTracked` of that second are skipped;
  SE3 0.0350 m. Stereo no drop: one map, SE3 0.0362 m. Without the 20-Frame
  condition both stereo drop runs lost tracking again at frame 532–533 and
  made a second map (above). Two first-try runs (one drop, one no drop, both
  before the condition) had SE3 0.27 / 0.060 m from errors **before** frame 500,
  where no ruling-J code runs: run-to-run variance of the stereo run, not
  this change (their reruns: 0.0349 / 0.0362 m).

## Loop Closing (roadmap step 4a+4b: detection and correction)

Spec: `docs/superpowers/specs/2026-10-09-loop-closing-design.md`; plan
`docs/superpowers/plans/2026-10-09-loop-closing.md`. Goal: when the camera
returns to a mapped place, detect it and correct the accumulated drift (pose,
and scale for mono), following ORB-SLAM3's visual-only `LoopClosing`
(`NewDetectCommonRegions`, `CorrectLoop`) and `Optimizer::OptimizeEssentialGraph`
for the algorithm, its thresholds (checked against the ORB-SLAM3 source) and
its order. Global BA is step 4c (see "Global BA"), map merging step 5 (the merge itself: "Mapper").

**Status (2026-10-10): implemented, opt-in, off by default.**
- *What works:* loops on synthetic scenes, and revisits of nearby views. In
  the integration tests the end error with loop closing is 0.04–0.15 of the
  error without, with the scale drift corrected. On corridor1, stereo loops
  between nearby views correct in 231–688 ms with a clean next BA.
- *What fails:* on corridor1 it makes the ATE worse. Same-heading
  look-alikes (a corridor pattern repeating 7–30 m apart) pass every gate,
  and correcting one breaks the map. Mono goes from 0.341 / 0.500 m without
  to 0.679 / 0.669 m with; stereo from 0.098 / 0.138 m to 1.295 / 18.16 m.
- *Normal gate weakness:* once a bad correction has rotated the map, the
  gate also blocks the true return (below).
- *Default:* off. Enable it with `LoopCloserOptions::enabled` or the apps'
  `--loop_closing`.
- *Next:* look-alike rejection is the first item of roadmap step 6 (TUM-VI
  tuning).

| ORB-SLAM3 | Ours |
|---|---|
| LoopClosing detects and corrects, pausing LocalMapping | The LoopCloser only detects, on its own replica; the Mapper corrects and does no local mapping meanwhile |
| Shared map objects corrected in place, under locks | One `MapTransaction` at a new map epoch, applied by every replica |
| The Tracker reads the corrected map | The Tracker applies the transaction and shifts its `last_frame_` |
| Global BA in a thread after the correction | Step 4c, done 2026-10-10: see "Global BA" |

**Map graph.**
- **Spanning tree** (ORB-SLAM3 `UpdateConnections`, `SetBadFlag`): each Frame
  has a parent, the Frame sharing the most points with it at creation; the
  first Frame of a map has none. A culled Frame's children are re-parented
  one by one to the candidate (the culled Frame's parent or an already
  re-parented sibling) with which they share the most points. Ruling B
  deviations (TODO): a new Frame sharing no points takes the previous newest
  Frame as parent (ORB leaves none; the tree stays connected), and
  re-parenting uses live shared-point counts with lower-id ties (ORB: cached
  weights ≥ 15).
- **Loop edges:** each Frame keeps its loop partners; a Frame with a loop edge
  is never culled.
- **`MapTransaction`** carries `parent` per `NewFrame`, `parent_changes`
  (frame, new parent), `added_loop_edges` (frame, frame) and `map_epoch`
  (`uint32_t`, +1 per loop correction); every replica stores them
  (`MapReplica::parent()`, `children()`, `loop_edges()`, `map_epoch()`). **`FrameTracked`**
  carries the epoch of the Tracker's replica.

**Similarity transforms.** `geometry::Sim3` (scale, R, t; `Inverse`,
`Compose`, `Apply`) is plain data in `voyah_common`
(`voyahslam/geometry/sim3.h`), so `LoopDetected` can carry one; Horn /
Umeyama alignment and `SolveSim3Ransac` live in `voyah_algs`
(`geometry/sim3_solver.h`) (ruling A). Directions follow ORB-SLAM3:
`S_query_match` is Scm, x_query = S · x_match in camera coordinates; a pose
S_cw folds its scale into the translation as `[R, t/s]` (ORB-SLAM3).
- **`SolveSim3Ransac`:** seeded RANSAC over 3-point samples, with or without
  scale, inliers by raw-pixel χ² 9.21 in both images (ORB `Sim3Solver`), the
  pixel of a point being its projection in its own Frame (ORB
  `mvP1im1`/`mvP2im2`); adaptive iterations capped at 300 with confidence
  0.99, best hypothesis kept (ORB stops at the first hypothesis over the
  inlier count; both are RANSAC, ours is stricter).
- **`optimization::OptimizeSim3`** (`sim3_optimization.h`, g2o
  `types_sim3`): ORB `OptimizeSim3`, projection residuals in both Frames in
  raw pixels with σ = sigma_px · 1.2^level, Huber, 5 + 5 iterations with
  outliers removed between; scale fixed for stereo.
- **`features::SearchPointsBySim3`** (ORB `SearchByProjection(KF, Scw, ...)`)
  and `SearchBySim3Projection` (ORB-SLAM2 `SearchBySim3`, mutual): radius
  th · 1.2^level, scale-invariance and viewing-angle gates. Both compare the
  observing feature's descriptor and level, not a map point's distinctive
  descriptor (the interface has none; TODO).
- The g2o Conan package gained the `types_sim3` component, and its version
  became `20241228.1` (2026-10-10) so that a stale cache fails to resolve
  instead of silently lacking it: other machines re-run `conan create
  third_party/g2o` for Debug and Release, then `conan install` (README).

**Detection (LoopCloser).** The LoopCloser keeps its own `AtlasReplica` (a
`MapReplica` per map, each with its `FrameDatabase` and covisibility; it
detects in the active map) fed by the Mapper's transactions, and
gets the immutable vocabulary through `Slam`. Loop closing is **opt-in**:
unless `LoopCloserOptions::enabled` is set (default false; the apps'
`--loop_closing`), or without a vocabulary, the LoopCloser only acknowledges
events and everything else behaves as before (the Tracker and the Mapper
still use the vocabulary). Every Frame of
`added_frames` is a query once the map has `min_map_frames` (12) Frames.
- **Queries wait, transactions do not (2026-10-10, final-review I1).** Each
  `ProcessPending()` first drains the inbox, applying every transaction to
  the replica (which releases the pooled event) and queueing its new Frames'
  ids, then detects the oldest query only. Before, a transaction was held
  until its Frames were detected, so a lagging LoopCloser could exhaust the
  Mapper's 256 transactions and abort it. Beyond `max_query_backlog` (8;
  ORB-SLAM3's queue is unbounded) the oldest queries are dropped, counted
  (`QueryStats::dropped`) and logged once per backlog episode. The Mapper
  never drops transactions and gets no back-pressure. A query culled while
  queued is skipped, and detection runs on the replica as it is then (later
  Frames, a correction already applied). One query per call also keeps
  `Stop()` prompt: it waits for the query in progress, not the backlog.
  `Stop()` logs the per-query detection time; on the synthetic loop scenes
  (Debug tree, `-O2`) it is 4.5–9.3 ms mean, 26 ms max; on corridor1
  (Release, `--rate=1`, 2026-10-10) mono 55 ms mean / 133 ms max over 1793
  queries (run beside three other apps and TSan), stereo 43.5 ms / 96.5 ms
  over 2258 queries, no query dropped in either.
- **A correction drops the pending region**: a transaction raising the
  epoch of the region's match map resets it, since its Sim3 is in the old
  coordinates; so does the removal of that map.
- **Candidates** (`DetectNBestCandidates`, ORB `KeyFrameDatabase::
  DetectNBestCandidates`): the `FrameDatabase::Query` hits of every live
  map (each `MapReplica` has its own database, dropped with the map on
  `removed_maps`) minus the query's covisible Frames, kept above
  int(0.8 · max) common words (max over all maps' hits), each grouped over
  its top 10 covisible Frames in its own map; by group score, the best 3 of
  the active map are the **loop** candidates and the best 3 of the other
  (inactive) maps the **merge** candidates.
- **Merge detection (map merging Task 4, ORB `NewDetectCommonRegions`).**
  Merge candidates go through the same `DetectCommonRegion` gates and
  options as loops, the match side read from the inactive map's replica
  (`RefineCommonRegion` likewise takes the match map's replica). A mono
  query solves a free-scale Sim3 (two mono maps have unrelated scales),
  stereo scale 1, as for loops. The merge region has its own pending state
  (`mnMergeNumCoincidences`, `mnMergeNumNotFound`): each pending region is
  refined against the query first; unless one then has 3 confirmations, the
  candidates are searched once and each kind not confirmed by this query is
  verified. A confirmed merge is published as `LoopDetected` with
  `match_map_id` the inactive map's; a loop confirmed at the same query is
  dropped unpublished (ORB handles `mbMergeDetected` first and resets the
  loop). ORB does not reset `mnMergeNumNotFound` on a confirmation; we do,
  as for loops (2 failures *in a row* drop either region). Synthetic
  two-map scene (`LoopCloserMergeTest`, seeds 1–5, map scale ratios 0.5,
  1.1, 2 mono and 1 stereo, first detection): BoW 400, RANSAC 164–332,
  coarse 396–400, optimized 209–368, projection 393–398 (thresholds 20 /
  15 / 50 / 20 / 80); the published Sim3 within 0.06° and 1.4 % of scale
  (mono) of the truth. *No normal gate for merges* (controller ruling
  2026-10-10): it compares normals in world coordinates, and two maps'
  worlds are related only by the merge hypothesis, so merges rely on
  ORB-SLAM3's gates; the disagreement share is still logged.
  `AcceptsMergeAcrossRotatedWorlds` (map 2's world turned 120°) fails if
  the gate is applied to merges.
- **Per candidate** (`DetectCommonRegionsFromBoW`, `common_region.cpp`): the
  window is the candidate plus its top 10 covisible Frames (dropped if it
  touches the query's neighbourhood); `SearchByBoW` (ratio 0.9) from the
  query's map points, ≥ 20; `SolveSim3Ransac`, ≥ 15 inliers (scale 1 for
  stereo); the window's points projected through the RANSAC Sim3 (th 8,
  ratio 1.5) into a fresh match list, ≥ 50; `OptimizeSim3`, ≥ 20 inliers;
  projection with the optimized Sim3 (th 5, ratio 1.0), ≥ 80.
- **Consistency** (`mnLoopNumCoincidences`): a region needs 3 confirmations,
  from the query's top 10 covisible Frames (≥ 30 projection matches, th 3)
  or from later queries (`RefineCommonRegion`, ORB
  `DetectAndReffineSim3FromLastKF`: 30 matches, > 50 `OptimizeSim3`
  inliers, ≥ 100 with the refined Sim3); 2 later queries in a row that fail
  drop it. The refinement re-projects with the optimized Sim3 (ORB's source
  uses the unoptimized guess, apparently a bug); ORB's widening of the match
  window for maps with fewer than 10 covisibles is skipped (TODO).
- **Normal-consistency gate — a deliberate deviation from ORB-SLAM3 (user
  decision, 2026-10-09).** It is applied to a candidate after step 5, to each
  covisible confirmation, and to each later query's refinement. The gate
  looks at the final matched pairs: the own map point of each feature, and
  the map point it matched. If more than `max_normal_disagreement` (0.5) of
  them have normals more than 60° apart, the region (or the confirmation) is
  rejected. Pairs whose normal is undefined count in neither the share nor
  its total. Rejections go into `CommonRegionStats::normal_rejections`, and
  the LoopCloser logs them ("Normal gate: …").
  - *Why:* the false-loop diagnosis
    (`.superpowers/sdd/2026-10-09-loop-closing/false-loop-diagnosis.md`)
    found that corridor1's opposite-facing false loops pass ORB-SLAM3's
    visual-only gates as such. An ORB-exact chain accepts the stereo
    2262→1587 loop with 81 final matches and a third false loop at exactly
    80. The gates are consistent with the hypothesis they test, but blind to
    the map's own geometry: the recovered Sim3 was 6.5° / 0.22 m where the
    truth is 176° / 9.2 m.
  - *Measured:* on the pairs of the false loops, 0.96–1.00 of the normals
    are more than 60° apart, against 0.00–0.05 on true loops.
  - *What it misses:* same-heading look-alikes, such as frames 2215–2236
    against 692–807 (a corridor pattern repeating 14.9 m apart), have
    consistent normals and are not caught (TODO).
  - *Map-frame weakness:* the normals are compared in map coordinates, so
    the gate assumes that the two ends of a loop are rotated by less than
    60° against each other in the map. Once a bad correction has rotated
    them (corridor1, after a same-heading false loop), the gate blocks the
    true return: in the fix-round-1 smoke it rejected 70–188 true-return
    candidates per run, 0.1–1.0 m apart. Large mono rotation drift would do
    the same (TODO).
- **`LoopDetected`** (priority: it interrupts the Mapper's local BA, unless a
  loop was already published at the replica's current epoch — a repeat found
  before its correction arrived, which the Mapper drops anyway; 2026-10-10):
  `query_map_id`, `match_map_id` (equal for a loop, the inactive map's for
  a merge), `query_frame`,
  `match_frame`, `S_query_match` (scale 1 for stereo) and the matched
  `(query point, match point)` pairs (features without points are dropped:
  TODO). Published from a pool of 16 with `TryCreate`: a loop found while
  all are in flight is dropped and found again. Duplicates are not
  suppressed; the Mapper ignores a closed loop. The LoopCloser is not wired
  to `TrackingLost` in `Slam`; the Mapper's map-id check rejects a lost
  map's late loops (TODO).

**Correction (Mapper, `mapper_loop.cpp`, ORB `CorrectLoop`).** A
`LoopDetected` is rejected if its maps are not the current one, the query or
the match was culled, its Sim3 is invalid, or the loop is already closed
(the match window is covisible with the query, or linked to its window by a
loop edge). Otherwise:
1. **Window:** the query and its Frames sharing ≥ 15 points
   (`min_connection_weight`, ORB `GetVectorCovisibleKeyFrames` after
   `UpdateConnections`).
2. **Window poses:** the query's corrected Sim3 is `S_query_match` composed
   with the match Frame's pose; every window Frame keeps its Sim3 relative
   to the query.
3. **Points:** each point the window observes moves with the first window
   Frame observing it (ORB `mnCorrectedReference`; the essential-graph pass
   then follows that Frame), X' = S_new⁻¹ · S_old · X; for mono the scale folds into each
   Frame's translation. Reference distances are not rescaled (ORB
   `UpdateNormalAndDepth`; TODO).
4. **Loop fusion:** the pairs replace the query side's points with the
   match side's; `SearchAndFuse` projects the match window's points into the
   corrected window Frames (Sim3, radius 4 · 1.2^level) and merges
   duplicates through the existing fusion and "points replaced" machinery.
5. **Connections:** covisibility updates, which creates the cross-loop links
   (the loop connections, every weight as ORB).
6. **Essential graph** (`optimization::OptimizeEssentialGraph`,
   `essential_graph.h`, g2o Sim3): a vertex per Frame (scale fixed for
   stereo; the match Frame fixed, as ORB-SLAM2 and the spec; ORB-SLAM3 fixes
   the map's first Frame); edges: the spanning tree, all loop edges and
   covisibility edges with ≥ 100 shared points (not with the parent or a
   child), measured from the uncorrected poses, plus the loop connections
   measured from the corrected ones (`EssentialGraphProblem::edges` and
   `loop_connections`, deduplicated); identity information; 20 LM
   iterations. Frames take the optimized poses and points follow their
   reference Frame. If the optimization fails, the rigid window correction
   is published with the new epoch.
7. **Loop edge** query ↔ match.
8. **One `MapTransaction`** at `map_epoch + 1` with every changed pose and
   point, the fused / replaced / removed points, the observations and the
   loop edge.

`MapperOptions::loop` (`LoopCorrectionOptions`) holds `min_connection_weight`
15, `fuse_radius` 4, `min_essential_weight` 100, `essential_iterations` 20.
Steps 3–6 are Mapper functions taking a window and a Sim3, for map merging.
**Stale epoch:** the Mapper creates no Frame from a `FrameTracked` of an
older epoch (its pose and observations are of the uncorrected map); its
visible / found counters are still updated; `stale_epoch_skipped()` counts
them.

**Tracker.** After applying a transaction with a new epoch, `last_frame_`
follows its reference Frame's world change, T_last ← T_last · T_ref_old⁻¹ ·
T_ref_new (the first Frame observing its matches if the reference is gone);
the velocity is unchanged. For mono the relative translation and the
velocity stay in the old scale for that one frame (TODO).

**Tracker across a merge** (since 2026-10-11, map merging Task 6, spec
section 4). A transaction merging the active map M into N (`absorbed_map` M,
`active_map` N) is followed like a correction, not adopted like a new map:
the Tracker switches to N (`current_map_id()`), keeps its state, velocity and
reference Frame (Frames keep their ids in N), and, while Tracking, shifts
`last_frame_` by the anchors taken in M before the transaction (the epoch path
above). `last_frame_.matches` then take their ids in N through
`AtlasReplica::TranslatePoint` (id rule, then the merge's fusions); a point
without one becomes unmatched. While RecentlyLost or Lost there is no last
frame: relocalization searches N. `FrameTracked` carries N and N's epoch.
The local map is rebuilt per frame from the replica, so it holds no ids
across a merge. A merge landing while the Tracker is **Lost** overtook its
final `TrackingLost(M)` in the Mapper, which ignores that loss as stale (M is
removed; had it read the loss first, it would have rejected the merge): the
Tracker publishes a final `TrackingLost(N)` with the first one's timestamp,
and the Mapper handles it as any final loss (N kept inactive or discarded,
the next initialization builds a new map). Any other change of the active
map resets (`AdoptMap`); a merge of the active map that leaves no map
active (a shape the Mapper never sends) resets too, as `AdoptMap` of the
target (map merging Task 7). A merge that does not absorb the active map
leaves the Tracker alone (`TrackerMergeTest.MergeOfAnotherMapLeavesTrackerAlone`).

**Map merging end to end** (map merging Task 7, spec section 6).
`SlamTest.{Mono,Stereo}MergesMaps` run MonoClosesLoop's loop extended to
600° (401 frames, so frames 240–380 pass the first lap's first 140 frames'
view again) with loop closing (and global BA) on, a blackout over frames
140–165 and `recently_lost_seconds` 1 s, over seeds 1–3, each against the
same run without the blackout. In every run map 1 is lost with 10–11
Frames and kept inactive, map 2 starts after the blackout (stereo at frame
166, mono about frame 174, once the parallax reaches 4°), the LoopCloser detects one merge on the
return (query frames 294–322) and the Mapper applies it; the run ends with
one live map, map 1, holding Frames of both parts; no `TrackingLost` after
the blackout and every frame from map 2's start tracked, the first one
after the merge by the motion model (`MotionModel` stage). The Mapper skips
one in-flight `FrameTracked` of map 2 per run (`other_map_skipped()`): the
real interplay of a merge with a frame tracked in M. Whole-run error (RMS of
the final map's Frame centres against truth, one alignment), merged /
baseline, measured 2026-10-11 (three runs; the numbers repeat to four
digits except where two values are given):

| Seed | Mono (m) | Ratio | Merge scale | Stereo (m) | Ratio |
|---|---|---|---|---|---|
| 1 | 0.0109–0.0130 / 0.0101 | 1.08–1.29 | 0.862 | 0.0046 / 0.0052 | 0.90 |
| 2 | 0.0054 / 0.0086 | 0.63 | 0.882 | 0.0041–0.0043 / 0.0031 | 1.33–1.40 |
| 3 | 0.0074 / 0.0056 | 1.31 | 0.919 | 0.0048–0.0050 / 0.0046 | 1.02–1.08 |

The bar is 1.5× (spec section 6). Margins are thin: map 1 holds 10–11
Frames at the loss against `min_frames_to_keep` 10, and the error ratio
depends on where the blackout falls. With the blackout at frames 160–185
(measured once, not adopted) the ratios were mono 0.81 / 4.84 / 3.30 and
stereo 1.13 / 1.61 / 0.85: map 2's part is distorted on its own (mono seed
2: 0.047 m RMS under its own alignment against 0.018 m for map 1's part).
The merge links the maps at one seam and nothing closes map 2's own drift
between its start and the seam, whereas the baseline closes its loop; the
error is map 2's open-loop drift, a random walk (TODO).

**Map merging smoke** (2026-10-11, corridor1 full, 5990 frames, Release,
`--rate=0 --vocabulary`, each run alone). Merge runs add `--loop_closing
--drop_frames 1500:1700` (global BA on). In every merge run the blackout
lost map 1 for good with 359–377 Frames (kept inactive), map 2 started
right after it and was merged back into map 1 about 4 s later (query at
t+89.1 s, frame ~1784, against a map-1 Frame at t+70.5–71.5 s, frame
~1410–1430). False merges are checked against the default stereo run's
trajectory (SE3 ATE 0.064 m against mocap; mocap covers only the first 27
s and the last 25 s): both camera positions 0.38–1.12 m apart, so every
merge was true (the first one with the camera turned about 95°, which the
fisheye views still share).

| Run | Merges found / applied / false | Maps over time | Final map | ATE of map 1 (`ate.py --map 1`) | Peak RSS | Wall |
|---|---|---|---|---|---|---|
| Mono 1 | 2 / 2 / 0 | 1 → lost at ~1600 (373 Frames, kept) → 2 at ~1700 → merged into 1 at ~1790; 1 lost at ~4600 (1220 Frames, kept) after a false loop → 3 → merged into 1 at ~5000 | 1 (1589 Frames) | Sim3 0.664 m | 2.08 GB | 9:41 |
| Mono 2 | 1 / 1 / 0 | 1 → lost (377, kept) → 2 → merged into 1 at ~1790 | 1 (1543) | Sim3 0.448 m | 2.23 GB | 9:36 |
| Stereo 1 | 1 / 1 / 0 | 1 → lost (359, kept) → 2 → merged into 1 at ~1784 | 1 (1696) | SE3 20.12 m | 2.73 GB | 18:58 |
| Stereo 2 | 1 / 1 / 0 | 1 → lost (364, kept) → 2 → merged into 1 at ~1784 | 1 (1636) | SE3 19.53 m | 2.69 GB | 18:36 |
| Stereo, default flags (memory) | — | one map | 1 (1808) | SE3 0.064 m | **1.23 GB** (master about 1.7 GB) | 18:27 |

No tracking loss followed a merge: the next loss came 4–8 min (wall) later, after
loop corrections. Against the reference trajectory the merged runs stay
within 0.02–0.13 m (stereo) and 0.2–0.6 m (mono, Sim3) until a later loop
correction: the large ATEs jump at the false loops of frames ~3100 (mono)
and ~5020 (stereo), corridor1's look-alikes (step 6), not at a merge. Merge
cost: 229–266 ms (windows 25–30 / 26 Frames), of which the weld 202–247 ms;
the second mono merge (1220 + 163 Frames) 465 ms, weld 220 ms. The
default-flags stereo run's peak RSS is 1.23 GB (1808 Frames), against
master's about 1.7 GB: the shared measurements (Task 1 measured 1.21 GB).

**Apps.** `--loop_closing` enables loop closing (`LoopCloserOptions::enabled`;
it needs `--vocabulary`). Without it the vocabulary serves relocalization and
triangulation only. The apps print whether loop closing is on.

**Tests.**
- **`voyah_algs`:**
  - `SolveSim3Ransac`: a noise-free Sim3 recovered to 1e-9 with or without
    scale; half outliers over seed ranges; degenerate input.
  - `OptimizeSim3`: finite-difference Jacobian; a perturbed Sim3 recovered;
    stereo scale fixed.
  - The Sim3 projection matchers' radii.
  - `OptimizeEssentialGraph`: a drifted ring with one loop edge corrected,
    the fixed vertex unmoved, stereo scale kept.
- **`voyah_slam`:**
  - The spanning tree, re-parenting and loop edges, in the Mapper and the
    replicas.
  - The LoopCloser:
    - a drifted return found only at the third confirmation, mono and stereo;
      the normal gate measures its pairs at ≤ 5 % disagreement;
    - a covisible revisit, and a window touching the query, are no loop;
    - an unmapped look-alike is rejected;
    - **an opposite-facing region** is rejected by the normal gate
      (`OppositeFacingRegionRejectedByNormals`, five seeds, mono and stereo).
      The scene is region A turned 180° about a vertical axis beyond it and
      seen from return poses turned alike. Without the gate it passes every
      ORB-SLAM3 gate and is published as a loop; with the gate, each
      candidate is rejected at ≥ 0.9 disagreement;
    - a later query's refinement is rejected by the gate too
      (`RefinementRejectedByNormals`);
    - failed confirmations drop the region; a new map clears it; without a
      vocabulary, or disabled, the LoopCloser does nothing.
  - The Mapper correction: the window, points and graph are corrected, loop
    points fused, one transaction published, the replica stays consistent; a
    stale epoch creates no Frame; culled Frames and other maps' loops are
    ignored.
  - The Tracker shift.
- **Integration:** `SlamTest.{Mono,Stereo}ClosesLoop` run the threaded `Slam`
  on a synthetic loop, once without and once with loop closing.
  - *Scene:* `SyntheticSequenceOptions::loop_radius` 1.5 m, with the camera
    looking outward and turning 450° at 1.5° per frame. The room is 10 m,
    its descriptors carry a place word per 45° sector, and the vocabulary has
    64 words.
  - *Without loop closing:* no loop.
  - *With loop closing:* a loop on the return (frame ≥ 210 of 301), one map,
    **no tracking loss across the correction**, and an end error at most a
    third of the run without.
  - *End error:* the mean, over the last 20 frames, of the centre error
    relative to the first-lap Frame of the same true pose, as the final map
    holds it. For mono it is taken in that map's local scale, which the
    correction changes. Both runs must have measured it (≥ 10 end frames with
    a reference, error > 0).
  - *Lock-step* (`RunSetup::lock_step`): each frame is fed once the previous
    one is tracked, the Mapper has handled every `FrameTracked`, and it has
    refined around every Frame it created. This takes the Tracker and the
    Mapper out of thread timing. The LoopCloser is not gated, so the frame at
    which a correction arrives still can vary; in practice three runs agree
    to four digits.
  - *Mono seeds:* `MonoClosesLoop` covers seeds 1–3, because mono drift is a
    random walk whose scale part varies by scene. It expects the end-of-run
    scale drift against truth (`end_scale_ratio`) to exceed 2 % without loop
    closing for some seed. For every such seed, loop closing must cut that
    drift to at most a third: the scale is checked as corrected, not just as
    detected.
  - *Measured:*

    | Run | End error without / with | Scale drift without / with |
    |---|---|---|
    | Mono seed 1 | 0.0676 / 0.0101 m | 5.5 / 0.4 % |
    | Mono seed 2 | 0.1139 / 0.0157 m | 0.7 / 1.4 % |
    | Mono seed 3 | 0.1429 / 0.0060 m | 4.6 / 0.5 % |
    | Stereo | 0.0157 / 0.0023 m | – |

    Over seeds 1–5, the ratio is at most 0.18 for mono and 0.25 for stereo.
- **ThreadSanitizer** (scratch tree, `-fsanitize=thread -O1 -g`): clean on
  2026-10-09 at e529d07, with 183 + 15 tests. Re-run on 2026-10-10 after the
  normal gate and the opt-in default: `voyah_slam_tests` (186) and
  `voyah_integration_tests` (15, the loop tests included) pass with no
  warnings.

**Smoke, fix round 1, with the normal gate.** Setup:
- corridor1 1024, the full sequence of 5990 frames;
- Release, ORB 1500, `--vocabulary ORBvoc.bin`, `--rate=0`;
- debug logs, headless, runs in sequence;
- 2026-10-09, two runs each with and without loop closing (then switched off
  with `--no_loop_closing`, since replaced by the opt-in `--loop_closing`);
- the previous round (before the gate) is summarized below the table.

How to read the table:
- **Loop frames** are dataset frame indices (query → match).
- **"ref"** is the distance and rotation between the two frames on a
  no-loop stereo trajectory (one map, SE3 0.077 m). A loop ≥ 2 m apart is
  false.
- **"Next BA"** is the first local BA after the correction: its initial
  cost, against a median of 1.4–1.8·10⁴.
- **"Gate"** counts the candidates and confirmations the normal gate
  rejected, as distinct (Frame, region) pairs, with their disagreement
  shares. "Opposite" counts those ≥ 2 m apart, "true" those ≤ 1 m apart.
- **ATE** is by `tools/ate.py` (mono Sim3, stereo SE3), shown as matched
  poses / poses. Mocap covers the room at the start and at the end.

| Run | Maps | Loops corrected: frames, ref, scale, ms, next BA | Gate | RecentlyLost / Lost | Frames, points | ATE |
|---|---|---|---|---|---|---|
| Mono, loop closing, 1 | 1 | **2217→702 (14.6 m, 0.952, 253 ms, 1.7·10⁸)**; **3216→2223 (7.6 m, 1.033, 1029 ms, 1.8·10⁸)**; 3336→3076 (0.6 m, 1.195, 487 ms, 7.7·10⁶); 4335→4203 (0.2 m, 0.548, 879 ms, 5.1·10⁸); 5360→4662 (0.2 m, 0.910, 857 ms, 1.3·10⁵) | 73: 1 opposite (3746→3507, 0.88); **70 true** (returns 5028–5281, 0.86–1.00, all after the false corrections) | 3 / 0 | 1637, 100 332 | Sim3 **0.679 m** (918 / 5735) |
| Mono, no loop closing, 1 | 1 | – | – | 2 / 0 | 1578, 95 958 | Sim3 0.341 m (917 / 5831) |
| Mono, loop closing, 2 | 1 | 1785→1416 (0.8 m, 0.992, 295 ms, 4.5·10⁴); **3210→792 (22.2 m, 0.960, 1324 ms, 7.2·10⁷)**; 3329→3087 (0.5 m, 1.070, 637 ms, 4.0·10⁵); 4335→4201 (0.2 m, 0.314, 1079 ms, 1.4·10⁶); **4669→4032 (22.2 m, 0.739, 1902 ms, 6.7·10⁶)**; 5363→4665 (0.2 m, 1.774, 1033 ms, 1.4·10⁶) | 6: 4 opposite (2266→1590, 2268→1592, 3740→3509, 3746→3509; 0.84–0.97); 2 others at 1.2 m / 103–115° (0.58–0.59) | 3 / 0 | 1650, 100 230 | Sim3 **0.669 m** (917 / 5817) |
| Mono, no loop closing, 2 | 1 | – | – | 2 / 0 | 1632, 99 745 | Sim3 0.500 m (918 / 5834) |
| Stereo, loop closing, 1 | 1 | 1784→1411 (1.1 m, 231 ms, 2.1·10⁴); 2026→1931 (0.5 m, 330 ms, 1.5·10⁴); 4145→1895 (1.0 m, 564 ms, 3.4·10⁴); **5017→2950 (30.1 m / 5°, 2452 ms, 1.8·10⁸)**; then 6 loops at 0.1–0.6 m (5338→1281, 5364→4662, 5387→4682, 5415→4708, 5429→4721, 5449→4735; 900–1422 ms, next BA 1.8·10⁶–1.7·10⁸) | 198: 2 opposite (2262→1587, 0.98; 4528→1366, 1.00); **178 true** (returns from 5033 on, 0.54–1.00, all after 5017); 18 others | 2 / 0 | 1754, 106 916 | SE3 **1.295 m** (1038 / 5949) |
| Stereo, no loop closing, 1 | 1 | – | – | 2 / 0 | 1781, 111 831 | SE3 0.098 m (1038 / 5935) |
| Stereo, loop closing, 2 | 1 | 2025→1932 (0.5 m, 353 ms, 1.5·10⁴); 4144→1896 (1.1 m, 688 ms, 3.1·10⁴); **4666→4029 (22.2 m / 5°, 2158 ms, 5.5·10⁸)**; 5359→4654 (0.4 m, 1050 ms, 3.0·10⁴) | 203: 2 opposite (2262/2265→1587, 0.93–0.98); **188 true** (returns from 5035 on, 0.71–1.00, all after 4666); 13 others | 3 / 0 (4222, 4429, 4682) | 1708, 105 627 | SE3 **18.16 m** (1039 / 5951) |
| Stereo, no loop closing, 2 | 1 | – | – | 2 / 0 | 1757, 109 278 | SE3 0.138 m (1038 / 5950) |

- **The gate does what it was added for.**
  - It rejected every opposite-facing false loop that reached it: 2–4 per
    run, at 0.84–1.00 disagreement. Among them is the stereo 2262→1587 loop,
    which cost map 1 in both runs of the previous round.
  - With the gate, every run keeps **one map**, and stereo has no loss other
    than those of the no-loop runs.
  - It rejected **no true loop before a false correction**.
- **Same-heading false loops are not caught, as decided.** Every loop-closing
  run corrected at least one loop between views 7–30 m apart with the same
  heading: a regular corridor pattern. Their normals agree.
  - Each such correction breaks the map: the next BA starts at 7·10⁶–5·10⁸
    with up to 60 % outliers.
  - The two ends of the map are then rotated against each other, and the
    normal gate, which compares normals in map coordinates, rejects the true
    returns: 70 (mono run 1), 178 and 188 candidates (stereo runs) at
    0.1–1.0 m, with disagreement 0.54–1.00.
  - In stereo run 2 the end of the map never reconnects (ATE 18.2 m).
- **Loops between nearby views** before any false correction correct
  cleanly in stereo: 231–688 ms, next BA ≤ 3.4·10⁴ (the median is 1.5·10⁴).
  In mono, even nearby loops carry extreme scales (0.314–1.774) and next-BA
  costs up to 5·10⁸, so mono corrections are not clean.
- **Musts: missed** for mono and stereo. Each run keeps one map with no
  extra loss, but its ATE with loop closing is above the run without (mono
  0.679 / 0.669 m against 0.341 / 0.500 m; stereo 1.295 / 18.16 m against
  0.098 / 0.138 m). The cause is the same-heading false loops,
  which are left for the user (TODO). Nothing was tuned.
- **Previous round** (e529d07, no gate), with the same setup:
  - mono ATE 0.652 m against 0.316 m without loop closing, and in run 2 two
    maps;
  - stereo four maps in both runs, against one map at 0.077 m without;
  - the cause was the opposite-facing loop 2262–2266 → 1587–1590. The next
    local BA after it started at 3.9–5.1·10⁸, and map 1 was lost 13–22
    frames after the correction.

## Global BA (roadmap step 4c)

Spec: `docs/superpowers/specs/2026-10-10-global-ba-design.md`; plan
`docs/superpowers/plans/2026-10-10-global-ba.md`. Goal: after each loop
correction, refine every Frame and point of the map with ORB-SLAM3's global
BA (`LoopClosing::RunGlobalBundleAdjustment`,
`Optimizer::GlobalBundleAdjustemnt(map, 10, &stop, loop_kf, bRobust = false)`)
in its own thread, without blocking local mapping, and merge the result into a
map that kept changing meanwhile.

**Status (2026-10-10): implemented.** It exists only with loop closing, which
is opt-in (off by default), and is on by default with it; `--no_global_ba`
turns it off. On the synthetic loops it lowers the map points' RMS error
against truth to 0.41–0.48× the essential graph's in stereo and 0.87–0.92× in
mono. On corridor1 the smoke runs had no crash and no tracking loss caused by
a merge (table below), but a run takes up to 85 s on a full stereo map and
raises the peak memory by up to 1.9 GB; three of the four runs with it lost
map 1 after a false-loop correction (the four without kept one map), and
the ATE is still dominated by the same-heading false loops (step 6).

| ORB-SLAM3 | Ours |
|---|---|
| The GBA thread reads the shared map under locks | `GlobalBundleAdjuster` owns a `MapReplica`, fed by the Mapper's transactions from the start of the run (like the LoopCloser); no late joining |
| `CorrectLoop` launches the thread, aborting a running one (`mbStopGBA`, `mnFullBAIdx`) | A transaction that adds a loop edge (the correction at epoch N) triggers a run; a newer correction or a new map arriving during a run interrupts it |
| Results parked in `mTcwGBA` / `mPosGBA` of the shared objects | One `GlobalBAFinished` event with plain data: `map_id`, `map_epoch` N, `frame_poses` (FrameId, T_cw), `point_positions` (MapPointId, position) |
| Local mapping stopped and the map locked for the merge | The merge is one Mapper handler call (the Mapper is the only writer) and one `MapTransaction` at epoch N + 1 |

**The component** (`global_ba/global_bundle_adjuster.h`):
- An `EventNode` with its own thread, replica and a pool of
  `GlobalBaOptions::max_events_in_flight` (2) `GlobalBAFinished`, published
  with `TryCreate` and `priority` (the Mapper's running local BA stops for the
  merge). An exhausted pool drops the result (`num_results_dropped`, logged)
  and is not retried: the map keeps the essential-graph correction until the
  next loop.
- **Trigger:** an applied transaction of the current map with
  `added_loop_edges`. There is no request event. After the inbox is drained,
  one run per `ProcessPending()`.
- **Snapshot contract** (`RunGlobalBa`, `global_ba/global_ba.h`): it builds a
  self-contained problem (every Frame, every alive point with an
  observation, all observations, mono and stereo) **before its first
  `should_stop` poll** and never reads the replica again. The replica may
  therefore change during the run; the result is the snapshot at epoch N and
  carries the run's captured `(map_id, N)`.
- **Apply-on-poll rule:** each `should_stop` poll drains the inbox; every
  transaction is first judged for interruption against the run's
  `(map_id, N)`, then applied to the replica, and so released. Why: the
  Mapper's transaction pool (256) aborts when exhausted, and a run on a large
  map lasts seconds at about 2 transactions per KeyFrame (the LoopCloser's
  apply-on-receipt rule, for the same reason). `should_stop` is sticky: once
  it returns true it stops polling.
- **Interruption:** `Stop()`, or a drained transaction that is a newer
  correction of the run's map (a larger `map_epoch` with a loop edge),
  recreates it (`new_map`), or leaves it not active (a new active map, or
  none after the Mapper drops a finally lost map; since 2026-10-10 the
  replica is an `AtlasReplica` and the run is on its active map). An
  interrupted run publishes nothing. A newer correction starts a new run
  next; a change of the active map drops the run. A run stopped by `Stop()`
  is not resumed after a restart.
- **Optimization** (`RunGlobalBa`): `optimization::BundleAdjust` as one plain
  stage of `GlobalBaOptions::iterations` (10), no robust kernel and no
  outlier removal (ORB `bRobust = false`), raw-pixel residuals through
  `geometry::Camera`, σ = `sigma_px · level_scale^level` (the
  `MapperOptions::fuse` defaults, ruling: a third σ source, TODO). Gauge:
  the spanning tree's root is fixed. Mono poses
  are SE3: the scale is the one the correction left.
- **Test-only seams:** `stop_requested()` (an atomic read, any thread: a
  test's run hook can see that `Stop()` has set the flag, which a test flag
  set before `Stop()` cannot do without racing), `set_run_hook()` (called
  before each run) and `set_poll_hook()` (called at each `should_stop` poll,
  before the inbox check; used to inject transactions mid-run). The hooks are
  set while the worker is stopped.

**The merge** (`Mapper::MergeGlobalBa`, ORB-SLAM3's post-GBA block):
- **Rejected** (counted in `num_global_ba_rejected`, nothing published) unless
  the Mapper is `Mapping`, the map is the current one and `map_epoch` equals
  the Mapper's current epoch (another correction in between: ORB
  `mnFullBAIdx`).
- **Frames:** a breadth-first walk of the spanning tree from the root,
  parents first. A Frame of the result takes its pose; a Frame created after
  N takes `T_cur · T_parent,cur⁻¹ · T_parent,new`, so the correction chains
  down the tree; culled Frames are skipped.
- **Points:** an alive point of the result takes its position (fused or
  culled ones are skipped); a point created after N moves rigidly with its
  reference Frame, unless that Frame was not corrected. Moved points get
  their normals recomputed (unlike ORB's GBA block; as our loop correction).
  Reference distances are not refreshed (TODO).
- **Publish:** one `MapTransaction` at `map_epoch + 1` with every changed
  pose and position, never with a loop edge (so it never triggers another
  run). The Tracker follows through the step 4b epoch path. Local BA work
  done since N on optimized Frames is overwritten, as in ORB-SLAM3. The merge
  takes the result as plain vectors, so map merging (step 5) can reuse the
  propagation.

**Epoch and the LoopCloser (deviation from ORB-SLAM3).** A global-BA merge
raises the map epoch (N + 1), and `LoopCloser::ApplyEvent` treats any higher
epoch as a correction, so a merge also drops the LoopCloser's pending region
(a partly confirmed loop restarts). ORB-SLAM3 does not reset it. Kept because
the pending `S_last_world` is in pre-merge coordinates.

**Wiring and options.** `Slam(..., LoopCloserOptions, GlobalBaOptions)` holds
a `std::optional<GlobalBundleAdjuster>`, created only with loop closing in
effect (a vocabulary and `LoopCloserOptions::enabled`) and
`GlobalBaOptions::enabled` (default true); `global_ba()` is null otherwise.
Mapper → GlobalBA → Mapper. Start: Mapper, GlobalBA, LoopCloser, Tracker;
Stop in reverse. With loop closing off nothing is created, so behaviour is
exactly as before. The apps print "Global BA: on/off" from `global_ba()` and
take `--no_global_ba`.

**Tests.**
- `GlobalBaProblemTest` (6): the problem covers every Frame and observed
  point; the spanning tree's root is fixed; a perturbed map is
  recovered; interrupted and empty runs return nothing.
- `GlobalBundleAdjusterTest` (11): the loop-edge trigger, interruption by a
  newer correction (also between LM iterations) and not by an ordinary
  transaction, a new map and a removed map (the final-loss transaction)
  dropping the run, removed maps ignored, `Stop()`, a full
  result pool, transactions released at each poll (a pool of 4 survives more
  polls than its size), changes during a run not affecting the result
  (bitwise equal to an undisturbed run).
- `MapperGlobalBa*` (9): optimized Frames and points, propagation down the
  tree (chains, re-parented chains), new points following their reference
  Frame, culled and fused objects skipped, one transaction at N + 1, a result
  interrupting local BA, stale-epoch and old-map rejection.
- `SlamTest.{Mono,Stereo}GlobalBaRefinesLoop` (seeds 1–3, lock-step; the
  harness waits for each correction's merge before the next frame, and fails
  fast with the counts if a result is rejected or not merged within 60 s):
  essential graph alone vs followed by global BA, both runs with one map, no
  loss and tracked to the end. **Stereo** bars: points RMS ≤ 0.8×, end error
  ≤ 1.0×; measured 0.41 / 0.48 / 0.44 and 0.76 / 0.67 / 0.18. **Mono** bars,
  revised after measurement (controller ruling; the 0.8× was an unmeasured
  guess): points RMS ≤ 0.95× per seed and ≤ 0.92× on average, end error
  ≤ 1.05× per seed; measured 0.89 / 0.87 / 0.92 (mean 0.89) and 0.58 / 0.35
  / 1.005. The mono RMS is dominated by low-parallax points of a 15–16 Frame
  map and global BA has converged (10 and 100 iterations agree).
  `SlamTest.GlobalBaCreatedOnlyWithLoopClosing` checks when the object
  exists; `*ClosesLoop` checks that a whole run without it merges nothing.
- **ThreadSanitizer** (2026-10-10, scratch tree `-fsanitize=thread -O1 -g`,
  under `setarch -R`): `voyah_slam_tests` 208 and, after the wiring, 217
  clean; `voyah_integration_tests` 18 with 0 warnings (the global-BA tests
  included; the mono test then failed only on its old bar).

**Smoke** (2026-10-10). Setup:
- corridor1 1024, the full sequence of 5990 frames;
- Release, ORB 1500, `--vocabulary ORBvoc.bin --loop_closing`, `--rate=0`;
- debug logs, headless, the runs one after another;
- two runs each with global BA (the default) and with `--no_global_ba`.

How to read the table:
- **RL / Lost** counts the Tracker's `Tracking -> RecentlyLost` and
  `RecentlyLost -> Lost` transitions.
- **GBA runs** are started / interrupted. No result was dropped or rejected
  in any run.
- **GBA s** are the completed runs, as (Frames, points) and seconds.
- **Merge ms** is the Mapper's merge; the result event is the payload of the
  pose and position vectors.
- **RSS** is the peak resident set (`/usr/bin/time -v`).
- **ATE** is by `tools/ate.py` (mono Sim3, stereo SE3) on the largest map,
  shown as matched poses / poses. Mocap covers the room at the start and at
  the end; a run whose map 1 was lost mid-run is measured on the start room
  only (415–536 matched poses), so its ATE is **not comparable** with a
  one-map run (917–1038).

| Run | Maps | Loops corrected | RL / Lost | GBA runs | GBA (Frames, points): s | Merge ms | Result MB | RSS GB | Wall | ATE |
|---|---|---|---|---|---|---|---|---|---|---|
| Mono, global BA, 1 | **2** | 4 | 4 / 1 | 4 / 0 | (461, 29.9k) 3.4; (592, 38.4k) 11.9; (1229, 78.2k) 28.2; map 2 (290, 17.4k) 4.2 | 6.7, 10.1, 17.8, 3.8 | 0.96, 1.23, 2.51, 0.56 | 2.24 | 9:59 | Sim3 0.034 m (415 / 4277, map 1) |
| Mono, essential graph, 1 | 1 | 5 | 2 / 0 | – | – | – | – | 1.51 | 9:29 | Sim3 0.707 m (917 / 5834) |
| Mono, global BA, 2 | **2** | 4 | 5 / 1 | 4 / **3** (after 15.1, 35.5, 16.7 s) | (899, 57.0k) 21.2 | 14.4 | 1.83 | 2.38 | 9:42 | Sim3 0.034 m (415 / 4516, map 1) |
| Mono, essential graph, 2 | 1 | 6 | 4 / 0 | – | – | – | – | 1.47 | 9:33 | Sim3 0.645 m (917 / 5812) |
| Stereo, global BA, 1 | 1 | 4 | 3 / 0 | 4 / 0 | (546, 36.2k) 5.0; (1160, 75.6k) 18.0; (1465, 91.9k) **82.5**; (1654, 103.1k) **85.2** | 13.0, 20.7, 30.1, 31.7 | 1.16, 2.42, 2.95, 3.31 | **4.23** | 19:37 | SE3 **18.68 m** (1038 / 5949) |
| Stereo, essential graph, 1 | 1 | 10 | 4 / 0 | – | – | – | – | 2.28 | 18:46 | SE3 0.836 m (1038 / 5770) |
| Stereo, global BA, 2 | **4** | 3 | 5 / 3 | 3 / 0 | (545, 36.1k) 4.8; (581, 37.7k) 12.4; map 4 (356, 22.8k) 3.3 | 10.5, 9.8, 7.7 | 1.15, 1.21, 0.73 | 1.56 | 18:32 | SE3 0.036 m (536 / 2220, map 1) |
| Stereo, essential graph, 2 | 1 | 4 | 3 / 0 | – | – | – | – | 2.33 | 18:49 | SE3 0.768 m (1038 / 5943) |

- **Musts met.** No run crashed (all exit 0), and **no tracking loss was
  caused by a merge**:
  - After every one of the 12 merges the Tracker followed the new epoch
    ("last frame follows Frame …") and kept tracking. The nearest loss after
    a merge (stereo 1, 2.8 s later) came 0.8 s after the *next* loop
    correction, with the frames between merge and correction tracked at
    284–392 inliers.
  - Every loss that started a new map began 0.1–1.0 s after an
    essential-graph correction (mono: 1504→1474 at scale 0.439, 1587→1371;
    stereo 2: 763→151) or minutes after any merge (stereo 2, maps 2–3, which
    never had a merge). Two merges landed while the Tracker was already
    `RecentlyLost` (mono 1, stereo 2); relocalization failed before and after
    them alike, and `Lost` came from the 5 s timeout.
- **More maps with global BA, not explained.** Both mono runs with global BA
  lost map 1, and so did stereo run 2 (4 maps), while the four
  essential-graph-only runs kept one map. Each loss follows a
  (mostly false, same-heading) loop correction, and which loops are detected
  differs from run to run (thread timing; the essential-graph runs corrected
  4–10 loops), so two runs per mode cannot separate the effect of global BA
  from that variance. Not tuned; a hypothesis to check with step 6: global
  BA makes the map consistent with a false loop's fused points, so the next
  false correction leaves the Tracker fewer inliers.
- **ATE:** no comparison is possible this round. Where global BA kept one map
  (stereo 1), a false loop (1463→674 or 1895→1036, Frame ids 800 apart) left
  SE3 18.68 m, as in the 2026-10-09 loop-closing smoke (18.16 m). The
  essential-graph runs: mono 0.707 / 0.645 m, stereo 0.836 / 0.768 m. Same-
  heading look-alikes still dominate (step 6).
- **Global BA is long on corridor1:** 3.3–28.2 s completed in mono and up to
  85.2 s in stereo (1654 Frames, 103k points), at 10 iterations; mono run 2
  had 3 of 4 runs interrupted by the next correction after 15–35 s. An
  interruption lands at the next LM iteration, up to about 6 s later on the
  largest mono map. TODO: an iteration or size cap.
- **Merge** 3.8–31.7 ms, growing with the map; the result event 0.56–3.31 MB
  (about 32 bytes per point; the points dominate).
- **Memory:** global BA raises the peak RSS from 2.28 / 2.33 to 4.23 GB in
  stereo (one map, 1758 Frames) and from 1.51 / 1.47 to 2.24 / 2.38 GB in
  mono: its replica (another full copy of the map with measurements) plus
  the g2o problem of a full BA. TODO "Memory".
- Mono run 1 with global BA overlapped Debug builds and tests during its
  first 7 minutes, so its first three GBA times may be inflated; the other
  seven runs ran alone.

## 7. Coding Conventions

**Naming (Google C++ style):**
- Classes, structs, functions, methods: `PascalCase` — `MapPointStorage`, `Add()`, `GetNextEvent()`.
- Trivial accessors may be `snake_case` and mirror the member — `size()`, `positions()`, `normalized_points()`.
- Local variables and parameters: `snake_case`.
- Member variables: `snake_case_` with trailing underscore.
- Parameters passed by non-const reference (outputs): `out_` prefix — `TPoint3DF& out_triangulated`.
- Constants: `kPascalCase` — `kInitialCapacity`, `kDescSize`.
- Template parameters and type aliases: `T` prefix — `TFeatureExtractor`, `TDescriptors`, `TMeasurement`.
- Enums: `enum class` with `PascalCase` values.
- Namespaces: `voyahslam::<module>` (`map`, `events`, `memory`, `features`, `geometry`, …).

**Library layout (since 2026-10-07, roadmap step 0):**

| Library (target) | Contents | Depends on |
|---|---|---|
| `voyah_common` | `defs.h`, ids, `memory/*`, `logging/*`, `algebra/*`, geometry types (`RigidTransformation`, `Camera`, distortion models), image types, feature/measurement data types, stereo rig, the extractor *declarations* (`OrbFeatureExtractor`, `orb/image_pyramid.h`), map storage (`Frame`, `MapPointStorage`, `ObservationStorage`, `CovisibilityIndex`) | Eigen, spdlog (PUBLIC) |
| `voyah_events` | `events/*`: event types, `Observable` / `AsyncObserver` / `EventNode`, `WakeSignal`, `WorkerThread` | common; concurrentqueue (PUBLIC) |
| `voyah_algs` | estimators, two-view geometry, image filters, `io/read_image`, FAST (`features/fast.h`), the extractor *definitions*, feature grid and matchers, point statistics, keyframe decision, triangulation, `optimization/*`, bag of words (`bow/*`) | common; stb, g2o (PRIVATE); no OpenCV since 2026-10-11 |
| `voyah_slam` | `tracking/*`, `mapping/{mapper, map_initializer}`, `loop_closing/*`, `slam/*`, `voyahslam.h` | common, events, algs |
| `voyahslam` (INTERFACE) | umbrella: links all four; the apps link it | all |
| `voyahviz` | viewer, image windows (dev tool) | common, events; OpenCV, Pangolin (PRIVATE) |

```
libs/
  CMakeLists.txt            the four libraries + the voyahslam umbrella
  common/  events/  algs/  slam/
    include/voyahslam/...   public headers (same relative paths as before)
    src/...                 sources and private headers
    tests/...               the library's unit tests (voyah_<lib>_tests)
  slam/tests/integration/   voyah_integration_tests (SlamTest.*, TrackerTest.TracksSyntheticStereo)
  test_support/             voyah_test_support (STATIC, links voyah_common only):
                            utils_test.*, support/*.h
cmake/VoyahTests.cmake      voyah_add_test_binary, voyah_optimize_target, check / check_all
cmake/voyahslamConfig.cmake.in  installed CMake package config (find_dependency)
vizualization/              voyahviz (+ voyahviz_tests)
conanfile.py                package recipe voyahslam/0.1.0 (also the dev installs)
test_package/               conan create's consumer test (links voyahslam::voyahslam)
profiles/                   linux-gcc13 (default + gnu20), android-arm64,
                            settings_user.yml (clang 21)
```

**Build options and packaging (2026-10-11):**
- CMake options `VOYAHSLAM_BUILD_TESTS` (tests, benchmarks,
  `voyah_test_support`, `check` / `check_all`; GTest, OpenCV) and
  `VOYAHSLAM_BUILD_EXAMPLES` (apps, `voyahviz`, `tools/`; OpenCV, Pangolin,
  yaml-cpp), both OFF by default. With both OFF configuring needs only Eigen,
  spdlog/fmt, concurrentqueue, g2o and stb. The dev trees turn both ON through
  the recipe's `with_tests` / `with_examples` options (cache variables in the
  Conan preset; README "Build and test").
- The four libraries follow `BUILD_SHARED_LIBS` (Conan `shared`, default
  static). `install()` puts them, the umbrella and the merged
  `include/voyahslam/` tree in the prefix, exported as `voyahslam::<target>`
  with `voyahslamConfig.cmake` (`find_dependency` Eigen3, spdlog,
  concurrentqueue; g2o only for a static build, whose `voyah_algs` exports g2o
  as `$<LINK_ONLY:...>`). stb is linked `$<BUILD_INTERFACE:...>` (header-only,
  never exported). `voyah_common` carries `cxx_std_20` as a PUBLIC compile
  feature.
- Conan consumers use the recipe's components (`cmake_target_name` =
  `voyahslam::voyah_common` ... `voyahslam::voyah_slam`); the package-level
  target is the umbrella `voyahslam::voyahslam` (a component of that name
  would collide with it). The package drops `lib/cmake`. PUBLIC deps are
  `transitive_headers` (spdlog also `transitive_libs`); g2o and stb are not
  (g2o's libraries still reach static consumers' link lines, never its
  include paths). `shared=True` also makes spdlog and fmt shared; shared
  libraries get `VERSION` / `SOVERSION` (0.1) and an `$ORIGIN` install
  runpath (not on Apple).
- `validate()` requires `compiler.cppstd` >= 20 (public headers are C++20;
  `profiles/linux-gcc13` = default + `gnu20` + Release, used by the dev
  installs), and rejects `shared=True` on Windows (no export macros) and with
  `compiler.libcxx=c++_static` (the NDK one-shared-library rule).
- Android arm64: `profiles/android-arm64` (API 24, clang 21 from NDK r30,
  `c++_static`); `voyah_algs` builds the NEON FAST kernel there. An app's
  CMake (NDK toolchain) finds the CMakeDeps folder through
  `CMAKE_FIND_ROOT_PATH` (the toolchain's `FIND_ROOT_PATH_MODE_PACKAGE=ONLY`
  re-roots `CMAKE_PREFIX_PATH`) and needs
  `CMAKE_MAP_IMPORTED_CONFIG_DEBUG=Release` for Gradle's debug variant
  (README "Using the package from an Android app").

- **Dependency rule:** a target's include path holds only its own root and
  the roots of the libraries it depends on, so a lower layer that includes an
  upper-layer header fails to compile ("No such file or directory"; verified
  by a one-off experiment: `map/frame.h` including
  `features/projection_matcher.h` breaks `voyah_common_tests`). `#include
  <voyahslam/...>` paths are unchanged by the split. common is the base;
  events and algs each depend only on common; slam depends on all three.
- **Placement:** a data type needed by a lower layer moves down; an algorithm
  that needs an upper layer moves up. The extractor classes are declared in
  common (every template, events and the viewer included, is parameterised on
  the extractor type) and defined in algs; code linking only common that calls
  `DetectAndExtract` gets a link error, not a compile error (descriptor-traits
  refactor in TODO).
- **Tests follow their code's library.** Each library has a `voyah_<lib>_tests`
  binary labelled `unit`; tests above 1 s in the optimized build and the
  whole-pipeline tests go to `voyah_integration_tests` (label `integration`).
  Tests that need a file outside the repository get their own binary and
  label (`vocabulary`: `voyah_vocabulary_tests`, the DBoW2 equivalence test;
  it skips unless `$VOYAHSLAM_ORBVOC` is set, `check` excludes it and
  `check_all` runs it; 2026-10-07).
  Test helpers that need slam (the Tracker fixture) live in that library's
  `tests/` directory, not in `voyah_test_support`.

**Structure:**
- C++20.
- Include guards `VOYAHSLAM_<PATH>_H` (no `#pragma once`).
- One event type per header; file name is the event name in `snake_case`.
- Every header is self-contained: it compiles when included first and alone.
- No OpenCV in public headers (principle 8): OpenCV usage lives in `.cpp` files
  and private headers under `src/` (`vizualization/src/opencv_wrapping.h`).
  The `voyah_*` libraries have none at all since 2026-10-11 (the old
  `libs/algs/src/opencv_bindings/` is gone); a test or benchmark that compares
  with OpenCV links it itself.
- Link-level note: until 2026-10-11 the static `voyah_algs` linked OpenCV
  `PRIVATE`, so its consumers (`voyah_slam`, the `voyahslam` umbrella) still
  linked OpenCV's shared libraries. They no longer do: a package export of the
  four libraries needs no `find_dependency(OpenCV)`; only `voyahviz` does.
- Public headers included as `#include <voyahslam/...>`.
- Template classes header-only; non-template classes have a `.cpp` in `libs/<lib>/src/` mirroring `libs/<lib>/include/`.
- `[[nodiscard]]` on const accessors.
- Move semantics throughout.
- Templates over the feature extractor are explicitly instantiated with
  `INSTANTIATE_CLASS` (`features/instantiation.h`) for ORB (SIFT removed
  2026-10-11).
- Formatting: `.clang-format` in repo root (4-space indent).
- New sources go in `libs/<lib>/CMakeLists.txt`, new tests in
  `libs/<lib>/tests/CMakeLists.txt` (mirroring the `include/` layout under
  `tests/`); a test above 1 s in the optimized build goes to
  `libs/slam/tests/integration/`.

**Build & test (Conan 2 + CMake):**
- Dependencies (`conanfile.py`; was `conanfile.txt` until 2026-10-11): eigen,
  stb, spdlog, concurrentqueue, g2o (local package, see below); gtest with
  `with_tests`, yaml-cpp with `with_examples` (see "Build options and
  packaging").
- **OpenCV is not from Conan:** OpenCV **5.x** built from source and installed
  in `/usr/local` (`find_package(OpenCV 5 ... core imgproc features highgui)`,
  config in `/usr/local/lib/cmake/opencv5`). Every dev machine needs it.
  Build it with a GUI backend for voyahviz (`WITH_GTK=ON`; GTK2 dev files are
  enough). OpenCV 5 renamed `features2d` → `features` and split `calib3d` into
  `calib` / `geometry` / `stereo` (compat headers `features2d.hpp`,
  `calib3d.hpp` exist). Linked as: `voyahviz` → core, imgproc, highgui (PRIVATE);
  `voyah_common_tests` → all (calib3d reference); `voyah_algs_tests`,
  `voyah_algs_dataset_tests` and `fast_benchmark` → core, features (the
  `cv::FAST` reference). No `voyah_*` library links OpenCV (2026-10-11).
- **g2o is a local Conan package:** vendored in `third_party/g2o/` and built with
  `conan create third_party/g2o --build=missing -s build_type=Debug|Release` once per
  machine, before the project's `conan install` (`conanfile.py` requires
  `g2o/20241228.1@voyah/stable`). Step-by-step build instructions: `README.md`
  (checked on a fresh `CONAN_HOME` on 2026-10-05).
- Debug: `cmake --build cmake-build-debug/build/Debug -j8`.
- **Tests:** `cmake --build cmake-build-debug/build/Debug --target check`
  runs the unit tests (`ctest -L unit -j8`; 521 tests, 2.1 s on 2026-10-07) — the
  edit-and-test loop. `--target check_all` runs every test (`unit` +
  `integration` + `vocabulary`, `ctest -j8`; 533 tests, 2.6 s) — at the end of each task and
  before every merge. Both build the test binaries first and run in the
  **Debug tree** with `VOYAHSLAM_OPTIMIZE_TESTS=ON` (default): the voyah
  libraries, `voyah_test_support`, `voyahviz` and the test binaries get
  `-O2` on top of `-g`, `NDEBUG` stays undefined (asserts on;
  `BuildConfigTest.AssertsEnabled` checks it), Conan dependencies (g2o) keep
  their Debug build. Configure with `-DVOYAHSLAM_OPTIMIZE_TESTS=OFF` to step
  through code in a debugger.
- Release: `conan install . --output-folder=build --build=missing -s build_type=Release`, then the `conan-release` preset.

**Workflow:** brainstorm → spec (`docs/superpowers/specs/`) → plan
(`docs/superpowers/plans/`) → implement task-by-task with tests on a feature
branch → review → merge to `master`. Update this document at the end.

## 8. Decisions Log

| Date | Decision | Rationale |
|---|---|---|
| 2026-09-30 | Event delivery layer (`Observable`/`AsyncObserver`/`Listener`) is non-template, carries `Event*`. | Avoid template bloat; concrete types only known at the consumer's `switch`. |
| 2026-09-30 | Sync handlers run before async enqueue; sync-only notify skips ref counting. | No race on the event pointer; stack-allocated events possible for sync. |
| 2026-09-30 | `Recyclable::Recycle()` uses `acq_rel`. | Correct cross-thread release of event payloads. |
| 2026-10-02 | Zero locks; each thread owns a map replica; pub/sub only. | Core design goal. |
| 2026-10-02 | Mapper is the only map writer, including all loop closure logic. LoopCloser only detects. | Single writer keeps replicas consistent by construction. |
| 2026-10-02 | Tracker optimizes only its own pose; never mutates map objects. Its replica changes only via Mapper diffs. | Same. |
| 2026-10-02 | `MapPointId` / `FrameId` are never reused. | Stale IDs from lagging replicas are detectable and can't alias new objects; replicas share slot layout. |
| 2026-10-05 | Scope of the id rule made precise: `FrameId` unique per run; `MapPointId` unique within a map and append-only (`MapPointStorage::Remove` only marks a point dead, the free-list slot reuse is removed; `num_ids()` = next id); point ids restart per map and `(map_id, id)` identifies a point. CLAUDE.md updated to match. | The free list could hand a removed id to a new point (aliasing for lagging replicas, non-contiguous `MapPointsAdded` ranges); restarting per map is safe because every event carries `map_id`. |
| 2026-10-02 | Tracker lag between `NewKeyFrameCandidate` and resulting diffs is accepted. (Since 2026-10-06: between `FrameTracked` and the resulting `KeyFrameCreated`.) | Matches ORB-SLAM3 behaviour; Tracker keeps tracking with its current replica. |
| 2026-10-02 | `ObjectPool` free-list is a lock-free Treiber stack with tagged head; links in a separate index array. | Removes the last mutex from the event path; slot memory never read concurrently with construction. Verified with ThreadSanitizer. |
| 2026-10-02 | `Observable::Notify()` always takes ownership of the event. | One rule for producers regardless of how observers are wired. |
| 2026-10-02 | Pooled objects are default-constructed once and kept alive; `Recycle()` doesn't destroy, reuse calls optional `Reset()`. `Create()` takes no arguments. | Payload buffers (the bulk of allocation) are reused, not just the event shell. |
| 2026-10-02 | Tracker has its own thread and queue; `Track()` only enqueues. | Map diffs and tracking are processed on one thread, so the Tracker's replica needs no locks. |
| 2026-10-02 | Tracker uses two queues: map diffs (inbox) drained first, then measurements. `Track()` takes the measurement by value and enqueues it. | Replica is current before every tracking step; allows dropping stale measurements; one moodycamel queue wouldn't order the two producers anyway. |
| 2026-10-02 | `CameraFrame` → `MonocularMeasurement`, `BinocularCameraFrame` → `BinocularMeasurement`; timestamp stored in the measurement (binocular uses the left's); removed from `Frame` and keyframe events. | Avoids `CameraFrame`/`Frame`/`MonoFrame` confusion; one source of truth for time. |
| 2026-10-02 | Long-term goal: mixed monocular/binocular keyframes in one map. Tracker accepts both measurement types now; binding decided later. | Extension beyond ORB-SLAM3. |
| 2026-10-02 | Configuration lives in the user application; reference app in `examples/full_slam/`. | Library stays free of config-file parsing. |
| 2026-10-03 | `images::TWorkingImage` is 8-bit (`TImageU8Gray`), not float. | FAST/BRIEF only compare intensities, float = 4× memory traffic. GPU backends read 8-bit textures as normalized floats, so float storage isn't needed for a future GPU extractor; the extractor interface (image in, features out) is the GPU boundary. Same feature count; mono 231 → 209 ms. |
| 2026-10-03 | Long-term goal: drop the OpenCV dependency. | User preference; own Gaussian blur already, FAST next. |
| 2026-10-03 | Link a source-built OpenCV 5.1 from `/usr/local` instead of Conan's OpenCV 4.12. | Conan's 4.12 `cv::FAST` resizes its output to 10 000 keypoints on every call (57% of extraction time). With 5.1: mono extraction 231 → 70 ms, stereo ~404 → 149 ms on TUM-VI 1024² (float images). Also gives a GTK GUI backend. |
| 2026-10-03 | Map viewer: `SceneState` (event-driven model) + `MapViewer` (own thread, single inbox, fixed-rate loop); attached by the apps, not by `Slam`. | Emulates the future remote viewer: everything comes from events, so the consuming code is unchanged behind a serializer. Own thread decouples rendering from the processing loop and suits Pangolin. |
| 2026-10-03 | Respect the event boundary now; serialization + transport (HTTP/ROS) later. | Format depends on the transport choice; the boundary is what must not change. |
| 2026-10-03 | No OpenCV types in public headers; OpenCV linked `PRIVATE`; APIs use our types. voyahviz reworked accordingly (`TColorImage`, `Color`). Small libs (Eigen, spdlog, concurrentqueue) may stay public. | User requirement: library users must not need OpenCV headers or `find_package(OpenCV)`; requiring the small libs is acceptable. |
| 2026-10-03 | Visualization in a separate `voyahviz` library (`vizualization/`), allowed to use OpenCV; images via `cv::imshow`, keypoints drawn with `cv::circle`/`cv::line` rather than `cv::drawKeypoints`. | Dev-only tool. Own drawing avoids converting to `cv::KeyPoint` (whose size/octave we don't have) and allows orientation ticks and future colouring by level/tracking state. |
| 2026-10-03 | Measurements drop features whose ray is > 85° off the optical axis (`kMaxRayAngle`), tested in distorted space via `Distort()` of an 85° ray. | z = 1 normalized points can't represent ≥ 90° rays (fisheye); model-agnostic check needs no new distortion-model API. |
| 2026-10-02 | Dataset reader is synchronous (no thread); apps pace with `PlaybackClock` (absolute deadlines). | Measured: load ≈ 10 ms vs extraction ≈ 220 ms per frame — a reader thread can't help; keeps the reader trivially testable. |
| 2026-10-02 | `ObjectPool::Create()` aborts on exhaustion in every build type (not `assert`). | Release builds silently returned `nullptr`. |
| 2026-10-02 | Orchestrator named `Slam<FE>`; components share a non-virtual `EventNode` base for their observable + inbox. | `Slam` is what the user instantiates; `System` is too generic. One base avoids repeating the observer API in three classes. |
| 2026-10-02 | One event type per header, plus `events/events.h` umbrella; `map_events.h` removed. | User preference: easier to navigate and to include only what is used. |
| 2026-10-02 | Event payloads use grow-only `ColumnBuffer` (capacity + count) instead of plain `Eigen::Matrix`. | Eigen reallocates on any size change; grow-only buffers reach a steady state with zero allocations. Chosen over fixed max-size buffers, which waste memory and need a hard cap. |
| 2026-10-03 | Scope A for the first protocol: state machines + component loops + initialization protocol, with geometry behind rule-based stubs (`PoseEstimator`, `MapInitializer`). | Runs the whole pipeline end to end now; the geometry spec replaces the stub bodies in place without touching the protocol. |
| 2026-10-03 | Mono/stereo measurements travel as `std::variant` (`features::TMeasurement<FE>`) inside events instead of a template parameter (OQ 24). | One `EventType` per event; consumers holding `Event*` can pick the concrete type; mixed keyframes stay possible. |
| 2026-10-03 | Idle wait via per-component `WakeSignal` (atomic counter, C++20 wait/notify) + `WorkerThread` (OQ 20). Snapshot is taken before the `running` check. | Lock-free sleep/wake from both `Track()` and queue pushes; the snapshot order avoids a lost `Stop()` wake-up. |
| 2026-10-03 | The Mapper (only map writer) performs initialization from `InitializationMeasurement`s forwarded by the Tracker. | Single-writer principle: the Tracker never creates map objects. |
| 2026-10-03 | Maps are identified by a monotonic `map_id` on every event; consumer rule: ignore older, reset-and-adopt newer, apply equal; 0 exempt. | Detects stale events across map restarts; the initial map's diffs precede `MapCreated`, so a newer id must adopt rather than drop. |
| 2026-10-03 | A component changes its state (and `current_map_id_`) **before** publishing the event that announces it (`MapCreated` -> `Mapping`, `TrackingLost` -> `Lost`). | Whoever observes the event, on any thread, then also observes the new `state()`; publishing first let `state()` lag the protocol. |
| 2026-10-03 | Monocular initialization needs many frames (reference + a later frame); stereo uses one measurement. | Mono needs parallax between views; stereo has depth from one pair. |
| 2026-10-04 | Essential solver: normalized 8-point now, 5-point later (TODO, raised priority), behind an estimator interface that returns several candidates per sample. | 8-point is simple and verifiable; narrow-FOV scenes are ambiguous for it, which 5-point fixes. |
| 2026-10-04 | One generic `Ransac<TEstimator>` per model (H, E), not ORB-SLAM3's shared sample loop. | Reusable for PnP, Sim3, stereo; the models are independent; cost is two sequential runs (parallel run optional, TODO). |
| 2026-10-04 | ORB-SLAM3 truncated-χ² scores with σ in normalized units (σ_px / f). | Calibrated normalized points; keeps scores comparable across estimators and independent of the camera. |
| 2026-10-04 | `RigidTransformation` with hidden representation and `T_ab` (b -> a) convention; replaces the empty placeholder; `TPose` users migrate later (TODO). | 4x4 or quaternion storage can be swapped without touching callers; `T_ac = T_ab * T_bc` reads naturally. |
| 2026-10-04 | Local refinement: optional `Refine` (Levenberg-Marquardt over inliers) in each estimator, 5 rounds in `Ransac`; LM counts only accepted steps (20 accepted / 40 trials). | Minimal-sample models recovered only ~55 % of true inliers with errors of several degrees; counting rejected trials starved the budget. |
| 2026-10-04 | H-vs-E selection uses the COLMAP rule (`inliers_H >= 0.8 x inliers_E`), not the ORB-SLAM3 score ratio. | A calibrated E fits planes as well as general scenes, so the ratio does not separate them (ground truth gave about 0.41). |
| 2026-10-04 | Recovery/accuracy tests use wide-FOV scenes (`half_fov_tan = 1.0`, close to the TUM-VI fisheye); narrow-FOV and near-frontal planar scenes are tested as rejected/weak. | Narrow FOV is ambiguous for the 8-point solver; the weakness is tracked as 5-point priority rather than hidden by loose bounds. |
| 2026-10-04 | Accepted accuracy limits: translation direction < 10° on general scenes (max 9.92° over 21 seeds), rotation < 1°, recovery >= 50 % (narrow) / >= 75 % (wide, 30 % outliers), `num_triangulated >= 0.5 x true inliers`. | Statistics of the scores (noise in both views) and inlier-set lock-in of the refinement (least squares over a hard-thresholded, self-selected inlier set; LM itself converges); bundle adjustment after initialization provides the final accuracy. |
| 2026-10-04 | `ImageFeatures` stores the pyramid level per keypoint (`TLevels`, uint8); each extractor exposes `ScaleFactor(level)` (ORB `scale^level`, SIFT `2^(level/3)/2`). SIFT keypoints are sorted by response, truncated to `number_of_features`, angles in radians. | Scale-aware matching and initialization need the level; SIFT previously returned degrees and uninitialized columns when over-detecting. |
| 2026-10-04 | Matcher `max_distance` is `double` (spec said `uint32`). | SIFT float L2 distances must be configurable; ORB distances convert exactly. |
| 2026-10-04 | Orientation histogram default: 12 bins of 30 degrees, not 30 bins of 12 degrees. | ORB-SLAM3's `HISTO_LENGTH = 30` is applied as factor `1/30` per degree, i.e. 30-degree bins; that is its effective resolution. |
| 2026-10-04 | A failed reconstruction keeps the reference and the matcher's updated `last_matched` positions. | ORB-SLAM3 `mvbPrevMatched` behaviour: the next frame is matched against the tracked positions. |
| 2026-10-04 | Zero triangulated points and a non-finite median depth count as initialization failure. | The Mapper uses the result's keyframes/points directly; an empty or unscalable map must not be created. |
| 2026-10-04 | SIFT radians / strongest-N fix: see the `ImageFeatures` level row above. | Same change; recorded there. |
| 2026-10-04 | Monocular initialization: ORB-SLAM3 matcher + two-view reconstruction, scaled to median depth 1, published as `MapPointsAdded` -> `KeyFrameCreated` (with observations) -> `MapCreated`; per-map point ids; keyframes store `T_cw`. | Replicas need points before the keyframes that observe them; per-map ids keep storage compact and `(map_id, id)` unique. |
| 2026-10-04 | Pangolin map viewer: `SceneState` decodes events into a scene model (newest map wins); `PangolinRenderer` is private to voyahviz. | Same code can later be fed by a deserializer on a remote machine; renderer stays swappable. |
| 2026-10-04 | `Stop()` during `RunOnCurrentThread()` only requests the stop; the loop drains itself on its own thread. | Keeps the loop-thread invariant: the drain touches the scene/renderer, which only the loop thread may do. |
| 2026-10-04 | Closing the window does not end the loop; only `RequestStop()` / `Stop()` do. The loop keeps draining headless. | Producers' event pools must never stall because a user closed a window. |
| 2026-10-04 | A `RequestStop()` made before the loop is entered is preserved (never lost). | A producer finishing first must not leave `RunOnCurrentThread()` blocked forever. |
| 2026-10-04 | On Linux under Wayland with `PANGOLIN_WINDOW_URI` unset, `MapViewer`'s constructor sets it to `x11://`. | Pangolin's Wayland backend blocks in `eglSwapBuffers` on GNOME, stalling the drain and the producers' pools. |
| 2026-10-04 | Pangolin linked `PRIVATE` via `pango_display`. | No Pangolin headers/types for library users (principle 8). |
| 2026-10-04 | Window-creation failure counts as `window_closed()`, so `--viewer` without a display stops playback after the first frame. | The viewer keeps draining headless; an app that asked for a window should not silently play on without one. |
| 2026-10-04 | `MapViewer::RunOnCurrentThread()` besides `Start()` (macOS hedge). | Cocoa requires the GUI on the main thread; the app would then play back on a worker thread. Not used by the apps yet. In this mode other threads end the run with `RequestStop()` only. |
| 2026-10-04 | Apps: `--viewer` opens the window; playback stops when it is closed; after the last frame the app waits for it to be closed. | The map is only worth inspecting once playback ends; closing the window is the user's exit. |
| 2026-10-04 | Stereo matcher follows ORB-SLAM3's fisheye method: brute-force descriptor match (ratio 0.7) plus triangulation checks (positive depth, chi-square, parallax). Epipolar-guided search deferred. | A rectified row search is unsuitable for fisheye images; brute force is simple and verifiable, guided search is an optimization (TODO). |
| 2026-10-04 | Stereo matching runs on the Tracker thread, at dequeue, before the measurement is tracked or forwarded. | The Mapper then receives matched measurements; the Mapper thread stays free for map work and the measurement is matched once. |
| 2026-10-04 | `StereoRig::T_rl` (left -> right, `RigidTransformation`) replaces `R_rl`/`t_rl`; the dataset reader computed the inverse and is fixed. | One documented convention; the old fields were used inconsistently. |
| 2026-10-04 | `StereoMatches` is in its own header (`features/stereo_matches.h`). | Avoids an include cycle between `BinocularMeasurement` and the matcher. |
| 2026-10-04 | Stereo initialization: one identity keyframe; every valid depth becomes a metric map point (ORB-SLAM3); thresholds 500 features / 100 points. | One stereo pair already carries scale and depth; no median normalization. |
| 2026-10-04 | `InitialMapPoint::feature_in_second = -1` means a single-keyframe map. | The result type serves both mono (two keyframes) and stereo (one). |
| 2026-10-04 | Stereo observations are stored per map in an `ObservationStorage<Stereo>` beside the mono one. (Reversed for the Mapper 2026-10-07: the `CovisibilityIndex` is its observation record.) | Keeps mixed mono/stereo sessions possible. |
| 2026-10-04 | Stereo depth is capped at about 17 m by `max_parallax_cos = 0.99998` at the 0.11 m baseline. | Accepted for now; the limit is a matcher option, tune later. |
| 2026-10-05 | g2o is vendored pristine in `third_party/g2o/src/` and packaged by a local Conan recipe; linked `PRIVATE`; the public API stays on Eigen types. | Reproducible build across machines (no system g2o), no g2o in public headers (principle 8), adaptations live in the recipe so upstream updates are a directory swap. |
| 2026-10-05 | Local Conan package `g2o/20241228@voyah/stable`: `user`/`channel` set in the recipe (not passed on the command line). | A plain `conan create third_party/g2o` yields the reference `conanfile.txt` requires; a missing package gives Conan's "ERROR: Package 'g2o/20241228@voyah/stable' not resolved: Unable to find 'g2o/20241228@voyah/stable' in remotes." error, which the README explains. |
| 2026-10-05 | BundleAdjust works on undistorted normalized points (z = 1) with sigma in normalized units; poses are `T_cw`; analytic Jacobians. | Same convention as the geometry toolkit (no camera model inside the optimizer; works for pinhole and fisheye). Pixel-space residuals deferred (TODO). Superseded later on 2026-10-05: raw-pixel residuals (camera-model rows below). |
| 2026-10-05 | BundleAdjust is two-stage like ORB-SLAM3 LocalBA: Huber pass (5 iterations), deactivate edges with chi2 > 5.991 or depth <= 0, plain pass (default 10 iterations). | A single robust pass drifted (baseline collapse, free scale) with 10 % gross outliers; deactivating outliers and re-converging fixed it. |
| 2026-10-05 | Monocular initialization runs a two-keyframe BA after the two-view reconstruction; outlier points are dropped; fewer than 50 survivors fail the attempt. | Accuracy (the reconstruction is limited by the RANSAC models; see TODO "refinement lock-in"); ORB-SLAM3 does the same (`Optimizer::GlobalBundleAdjustemnt` in `CreateInitialMapMonocular`). |
| 2026-10-05 | Init-time error at low parallax is noise-limited (synthetic seeds initialize at frame 3, baseline 0.15 vs depth 4-8; ML optimum 0.3-2.7 degrees), so tests check convergence to the maximum-likelihood optimum (oracle from ground truth) rather than an absolute error bound. Mean error 2.11 -> 1.34 degrees with BA. | An absolute bound would test the noise, not the optimizer. |
| 2026-10-05 | Init BA uses 5 + 40 iterations and runs on the unscaled problem (baseline 1, depths ~40), before the median-depth rescale (ORB-SLAM3 order). | The plain stage needs about 40 iterations to converge there; LM stops at convergence so runtime is barely affected. Rescaling first would converge faster (TODO, tuning). |
| 2026-10-05 | The root `CMakeLists.txt` picks the concurrentqueue target (`VOYAHSLAM_CONCURRENTQUEUE_TARGET`); `src/CMakeLists.txt` (now `libs/events/CMakeLists.txt`) only uses it. The choice is between `moodycamel::concurrentqueue` and `concurrentqueue::concurrentqueue`. | Found by the clean-machine build: the revision of concurrentqueue/1.0.4 that Conan Center serves today declares only `concurrentqueue::concurrentqueue`. |
| 2026-10-05 | Tracking against the map (sub-project 1): `MapReplica` fed by Mapper events; motion model -> zero-motion fallback (window x4) -> local map -> `OptimizePose`; the `PoseEstimator` stub is deleted. | Spec `2026-10-05-tracking-design.md`; keyframes, local BA, relocalization are sub-projects 2-4. |
| 2026-10-05 | (Q) The projection matcher takes the point source as a template parameter; `features/` does not depend on `tracking/`. `PointReference` lives in `map/point_reference.h`; the rotation histogram is shared in `features/rotation_histogram.h`. | No dependency cycle between features and tracking; one histogram implementation for the initialization and projection matchers. |
| 2026-10-05 | (R) The replica does not count events (spec said it would, as `SceneState`). | Nothing consumes the counters; the replica stays a pure map copy. |
| 2026-10-05 | (S) `OptimizePose`: 4 rounds x 10 iterations; every round restarts from the initial pose (ORB-SLAM3); Huber in rounds 0-1; chi2 5.991 mono / 9.488 stereo (4 dof); success >= 3 inliers. | ORB-SLAM3 `PoseOptimization` schedule; the stereo edge has 4 rows (left + right normalized; raw pixels since the camera-model change below). |
| 2026-10-05 | (T) Mono accuracy is limited by the noise-limited initial map at the default 1 deg parallax (about 12 % depth error, about 10 % translation error). Tests use 4 deg parallax for the accuracy bound and a separate default-init robustness test. | Same cause as the init-accuracy row above; tuning is deferred (TODO). |
| 2026-10-05 | Matcher deviations from ORB-SLAM3: the orientation histogram is built from the final claims; points without a reference observation are skipped by the local-map search; no scale-invariance distance check (the predicted level is clamped). | Simpler; the distance check is a TODO. |
| 2026-10-05 | BoW-less fallback: a wide-window (x4) zero-motion projection search replaces `TrackReferenceKeyFrame`. | No vocabulary yet; relocalization / place recognition is sub-project 4. |
| 2026-10-05 | Camera model, approach A: a `geometry::Camera{K, distortion}` value type (`Project`, `ProjectJacobian`, `Unproject`); not camera-model classes (approach B, TODO if ever needed) and not free functions. | Smallest change that gives every consumer one projection; ORB-SLAM3's `GeometricCamera` hierarchy is not needed for one lens model. |
| 2026-10-05 | All five consumers (projection matcher, `OptimizePose`, `BundleAdjust`, stereo matcher gate, initialization matcher windows) compare **raw (distorted) pixels** with σ = `sigma_px · 1.2^level` px; χ² thresholds unchanged (5.991 / 9.488). Rays stay rays (two-view RANSAC, triangulation, the 85° test). Replaces the normalized-plane residual space of the BA row, (S) and the stereo gate (their schedules and thresholds stand). | Spikes on corridor1 (mono, 400 frames, tracked / re-initializations): baseline 32 / 7; KB window + σ 53 / 5; + local-map gate 20 108 / 5; angle-only allowance σ·1.5·(1 + r²) with window ×(1 + r²) 85 tracked, 99 with gate 20; uniform σ 3 effectively disables outlier rejection. One noise model for the library, as ORB-SLAM3. |
| 2026-10-05 | Measurements keep both the distortion model pointer (`camera()`) and the raw keypoints (`distorted_points()`). The model is app-owned and must outlive every measurement, event and `Camera` copy (pointer rule, §6; ownership superseded 2026-10-12). | Consumers need raw pixels and the model; recomputing raw pixels from undistorted ones would lose precision and cost a `Distort` per use. |
| 2026-10-05 | `DistortionModel` (Dummy, Kannala-Brandt) moved from `features/distortion_models/` to `geometry/distortion_models/` (namespace `geometry::distortion_models`); forwarding headers (with a namespace alias) remain at the old path. | `geometry::Camera` needs the model and `geometry` must stay at the bottom of the dependency stack. |
| 2026-10-05 | (U) `DistortJacobian` for Kannala-Brandt: J = (d/r)·I + (d′(θ)·θ′(r) − d/r)·p̂p̂ᵀ, θ′(r) = 1/(1 + r²), identity for r < 1e-8. | Analytic; verified against central finite differences (the spec draft had an extra 1/r). |
| 2026-10-05 | (V) Pinhole invariance with the old normalized code holds exactly (1e-9) only for fx = fy. `OptimizePose`, `BundleAdjust` and the stereo gate now weight x and y separately in pixels, so χ² changes slightly on fx ≠ fy rigs (e.g. EuRoC). | The old code divided σ by one f; per-axis pixel weighting is the intended behaviour. |
| 2026-10-05 | The calibration-free angle allowance (σ · 1.5 · (1 + r²), window · (1 + r²)) is recorded as a fallback for lenses without a model, not implemented. | Spike 2 measured it within 0.6 % of the KB stretch up to 60° (85 tracked frames vs 53 for KB window + σ); not needed while every lens has a model (TODO). |
| 2026-10-05 | Smoke result of the camera-model branch (run 2026-10-06, corridor1, Release): mono 82 tracked / 18 re-initializations (gate 20: 137 / 14, measurement only); stereo 197/200 local-map successes (198 before), no loss, median 232 inliers. The ≤ 5 re-initializations criterion is missed; nothing was tuned. | The extra re-initializations come from the raw-pixel init matcher accepting more (mostly adjacent-frame) pairs: 19 maps vs 6; reverting only that matcher gives 6 maps / 56 tracked / 9.3 frames per map. Kept for more tracked frames (82 vs 56). Maps still die at the local-map gate without map growth (sub-project 2); init acceptance and gate tuning are in TODO. |
| 2026-10-06 | `FrameTracked` replaces `PositionUpdated` (deleted): every tracked measurement publishes pose, measurement copy, inlier pairs, visible points, reference Frame, local Frames, stage and counts. | One event gives the Mapper everything it needs to decide and the viewer the tracked / visible sets; no candidate protocol. |
| 2026-10-06 | The Mapper decides on Frame creation (`ShouldCreateFrame`, ORB-SLAM3 rule and defaults); it drains `FrameTracked` to the latest per map and counts the skipped ones as backlog (backlog 0 = idle). | The Mapper holds the authoritative observation counts, its own backlog and Frames the replica lacks; reverses the 2026-10-02 split. |
| 2026-10-06 | `NewKeyFrameCandidate` is deleted. | No candidate protocol (one in flight, rejections, timeouts) is needed once the Mapper decides. |
| 2026-10-06 | `KeyFrameCreated` is one atomic transaction (Frame, new points, observations on existing Frames, updated points), used by initialization too (mono 2 events, stereo 1); `MapPointsAdded` is deleted. | The replica applies an insertion in one `ApplyEvent`, so the Tracker never tracks against a half-applied one. |
| 2026-10-06 | The Tracker's local map is built from covisibility: K1 (Frames observing a match), K2 (top 10 covisible per K1 Frame), cap 80; local points are the local Frames' alive points. | ORB-SLAM3 `UpdateLocalKeyFrames` / `UpdateLocalPoints` without a spanning tree; keeps the search bounded as the map grows. |
| 2026-10-06 | Triangulation uses an epipolar test on rays and a raw-pixel χ² (5.991, σ = `sigma_px · level_scale^level`) in each Frame. | Rays work for pinhole and fisheye alike; the residual space matches the rest of the library. |
| 2026-10-06 | Matching for triangulation is brute force for now; the Faiss vs DBoW2/FBoW decision is deferred to sub-project 4. | Smallest working matcher; its cost is measured (smoke: about 25 ms per neighbour) and in TODO. |
| 2026-10-06 | (A) `FrameTracked::reference_frame_id` is `std::optional<FrameId>`; with none (or one the Mapper no longer has) the Mapper uses its newest Frame. (B) `sigma_px` / `level_scale` live in `MapperOptions::triangulation`. (C) The Mapper keeps its own `CovisibilityIndex` with a Mapper-thread-only `covisibility()` accessor. | (A) the Tracker may know none of the observing Frames; (B) one source of truth; (C) needed for `ref_points` and the replica-consistency test. |
| 2026-10-06 | (D, E) `DistinctiveDescriptor`: least median Hamming distance; an even count takes the mean of the two middle distances (ORB-SLAM3 takes the lower), ties go to the lowest index. | A strict winner on small sets; the plan's 1-bit example violated the triangle inequality (test uses 1/9/10). |
| 2026-10-06 | (F, spec amended) The epipolar band is the exact directional 3σ bound `asin(|n·ray_b|) ≤ k·σ·‖(JJᵀ)⁻¹J n‖` with J = `ProjectJacobian` at the unit ray (σ/f at a pinhole centre), replacing the spec's "norm of `ProjectJacobian`". | The spectral norm at z = 1 is up to 5.8× too loose at 80° and too strict along slow fisheye directions. |
| 2026-10-06 | (G, spec amended) Scale consistency compares dist_b / dist_a with level_scale^(level_a − level_b) (ORB-SLAM3 order). | The plan's dist_a / dist_b was inverted and rejected valid multi-level matches (`ScaleConsistencyFollowsOctaveRatio`). |
| 2026-10-06 | (H, spec amended) For `ref_points` / `min_obs` a stereo-matched observation counts 2 (ORB-SLAM3 `MapPoint::AddObservation`). | With a count of 1 a one-Frame stereo map has `ref_points` 0 and creates Frames only through `need_close`. |
| 2026-10-06 | (I) A later `TrackingLost` of the same map in the drained batch also supersedes a `FrameTracked`; all events keep their order. | No Frame is created for a map that is about to be cleared. |
| 2026-10-06 | (J) The mono triangulation unit test triangulates on the 4th Frame. | A noise-free initialization maps every synthetic point; new points duplicate existing ones until fusion. |
| 2026-10-06 | (K) `visible_points` includes already-matched points without re-projection (ORB-SLAM3 `SearchLocalPoints`), so visible ⊇ matched; K2 takes the top 10 neighbours of every K1 Frame (ORB-SLAM3 adds only the first new one). | The viewer and the Mapper see every tracked point as visible; the spec's wording for K2, slightly larger local map. |
| 2026-10-06 | (L) The reference Frame is the Frame sharing the most inliers among **all** observing Frames, not only K1. | Includes Frames reached through local-map matches, a better reference; occasionally a K2 Frame. |
| 2026-10-06 | Viewer colours: tracked points green, visible points blue, other points grey; local-map Frames orange, other Frames blue. | Debugging the local map and the matcher at a glance. |
| 2026-10-07 | Keyframes smoke result (corridor1, Release; "Keyframes and Map Growth"): mono 2 maps in 400 frames, first map 129 tracked frames, median lifetime 135.5, 1 re-initialization (was 82 tracked / 18 re-initializations); stereo 2 maps in 1000 frames (lost once at frame ~360; a second run: 3 maps, lost at frame ~354 and again on the new map's first frame). Both bars (first mono map ≥ 150; stereo no loss) are missed; nothing was tuned. Synthetic end-to-end error bounds loosened to 30 % of the path / 0.3 m. | Mono: the Frame rule stalls without fusion (`ref_points` stuck at 36); stereo: motion-model failure at 120-170 deg/s without a reference-Frame fallback. The synthetic error is noise random-walk without local BA (0 without noise). All in TODO. |
| 2026-10-07 | Restructure (roadmap step 0, approach C): split `voyahslam` into `voyah_common`, `voyah_events`, `voyah_algs`, `voyah_slam` under `libs/<lib>/{include,src,tests}`, each with its **own include root but unchanged `#include <voyahslam/...>` paths**; `voyahslam` is an INTERFACE umbrella; `voyahviz` links only common + events. The `OrbFeatureExtractor` / `SIFTFeatureExtractor` declarations (and `orb/image_pyramid.h`, held by value) stay in common, their definitions in algs. One test binary per library (label `unit`) plus `voyah_integration_tests` (label `integration`: every `SlamTest.*` and every test above **1 s** in the optimized build, except the fixed-cost `ObjectPoolDeathTest.PoolExhaustion`); `check` / `check_all` targets. `VOYAHSLAM_OPTIMIZE_TESTS` (default ON) adds `-O2` to the voyah libraries and tests in Debug, asserts on, dependencies unchanged; `check_all` runs from the Debug tree. | Boundaries enforced by the compiler (a wrong-direction include is "file not found") without touching a single include line or namespace; `voyah_events` usable by outside observers such as the viewer. The extractor declarations stay in common because every template, events and the viewer included, is parameterised on the extractor type (descriptor-traits refactor in TODO). Unoptimized Eigen/g2o dominated the suite: serial Debug 381 s → `check_all` 2.3 s (459 tests, `-j8`; 66.7 s at `-j8` before the option), `check` 1.5 s (449 tests); smoke runs unchanged. Spec `docs/superpowers/specs/2026-10-07-restructure-design.md`. |
| 2026-10-07 | Every Mapper event is one `MapTransaction` (map, Frames, points, observations added; points updated / replaced / removed; poses updated; observations and Frames removed), applied by every consumer in one fixed order (new map, add Frames, add points, add observations, update points, update poses, remove observations, remove points, remove Frames). Replaces `KeyFrameCreated`, `MapCreated`, `MapPointsUpdated`, `MapPointsRemoved`, `KeyFramePoseUpdated`. | Fusion and culling change many objects at once; one transaction keeps every replica consistent after each event (spec §2) and makes the apply order a single rule instead of a per-event convention. |
| 2026-10-07 | Two transactions per created Frame: T1 (tracked observations, recent-point culling, new points, fusion), T2 (local BA, Frame culling). | The Tracker gets the new Frame and points before the slow BA finishes. |
| 2026-10-07 | Local BA runs in the Mapper thread and is interrupted only by an urgent `FrameTracked` (final inliers < 1.5 x `min_local_map_inliers`), through a priority counter in the Mapper's inbox (incremented by `Push`, decremented only by `TryPop` / `Listener`). A separate BA thread is reserved for global BA (roadmap step 4). | No second writer, no locks; ordinary frames are absorbed by drain-to-latest, and only a weakly tracked frame needs the Mapper promptly. |
| 2026-10-07 | `BundleAdjust` takes `std::function<bool()> should_stop` (null = never) instead of `const std::atomic<bool>*` (ruling A), evaluated once before the first iteration and after every iteration (ruling D); a stop after the last robust iteration but before outlier deactivation reports `stage_reached` 0 and keeps the outliers (ruling E). Outliers are removed only when `stage_reached >= 1`. | The Mapper thread runs BA itself, so nobody could set a flag; the callback reads the inbox counter. A pre-set stop must leave the poses unchanged. Conservative: an outlier survives one more BA at most. |
| 2026-10-07 | The Mapper's `ObservationStorage`s are removed; its `CovisibilityIndex` is its only observation record; stereo depth and right index come from the Frame's measurement (ruling G). Reverses the 2026-10-04 storage entry. | Fusion and culling remove observations; one structure with removal support instead of two kept in sync. |
| 2026-10-07 | Pass order as ORB-SLAM3 LocalMapping (ruling H): tracked observations, then recent-point culling, then new points, then fusion. The spec's "culling before creation" was wrong. | Culling must see the new Frame's observations, or fresh points are judged one Frame short. |
| 2026-10-07 | Every point the Mapper removes, including implicit ones (no observation left), is listed in `removed_points` (ruling C). | Consumers without observation data (the viewer) would otherwise keep ghost points. |
| 2026-10-07 | Local BA: the 20-Frame cap counts the new Frame (ruling I); BA is skipped with no free Frame or no fixed Frame (ruling J, ORB-SLAM3's gauge guard). σ comes from `MapperOptions::fuse` (ruling K). | Mono BA without a fixed Frame has a free gauge. Ruling K leaves two σ sources (Mapper and Tracker) to keep in sync: TODO. |
| 2026-10-07 | Apps gain `--trajectory_out` (TUM, camera-to-world, one `# map <id>` section per map); `tools/ate.py` (pure Python, 20 ms association, Umeyama Sim3 / SE3, largest section). | The smoke runs now report ATE against the corridor1 mocap. |
| 2026-10-07 | Map-quality smoke result (corridor1, Release, two runs each; "Map Quality"): mono 400 keeps **one map** from initialization (frame 118) to the end, 281–282 tracked frames, 64–68 Frames created / 41–45 culled, ATE Sim3 0.032–0.033 m; stereo 1000 one map, no loss, 999 tracked, 280–297 Frames created / 73–89 culled, ATE SE3 0.036–0.037 m. Both Musts met; nothing tuned. Synthetic bounds tightened to 0.249 m mono / 0.037 m stereo. | Fusion lifts points to >= 3 observations, so the Frame rule keeps creating Frames (was 2 maps, first lost after 129 frames); local BA removes the noise random walk. |
| 2026-10-07 | Own OpenCV-free DBoW2 port in `voyah_algs` (`bow/`, `voyahslam::bow`): `Vocabulary<FE>` generic over the extractor's `TDescriptors` / `Distance`, flat node arrays in id order, TF-IDF + L1 only, child ties to the first child; the text loader and our checksummed binary format (`VOYBOW\0\1`, FNV-1a 64), `tools/convert_vocabulary`. Loaders never throw. Spec `docs/superpowers/specs/2026-10-07-bow-design.md`. | ORB-SLAM3's pretrained `ORBvoc.txt` applies to our ORB descriptors, without DBoW2's OpenCV dependency; the binary loads 4× faster than the text (0.13 vs 0.53 s; DBoW2's text loader 2.7 s). Reverses the 2026-10-06 "Faiss vs DBoW2/FBoW deferred" entry for matching. |
| 2026-10-07 | `Vocabulary<FE>` compiles for every `INSTANTIATE_CLASS` extractor (ORB and SIFT); the loaders reject a descriptor-size mismatch at run time (ruling A). | The Mapper is instantiated for SIFT too; only ORB has a vocabulary. |
| 2026-10-07 | The FeatureVector files a feature under the node its descent passes at depth `L − levels_up` (DBoW2); a leaf shallower than that files it under itself (ruling B, spec §1 amended). | DBoW2 leaves the node uninitialised there (UB). ORBvoc's shallowest leaf is at depth 4, so with `levels_up` 4 it never applies. |
| 2026-10-07 | Our text loader ignores trailing blank lines and creates no phantom node; DBoW2 creates one from `ORBvoc.txt`'s trailing newline (971 815 vs 971 814 words) (ruling D, spec §2 amended; TODO). | The phantom is built from uninitialised values; ours is the correct tree. Equivalence on real frames is exact (words, filing, bit-identical weights). |
| 2026-10-07 | ctest label `vocabulary` for `voyah_vocabulary_tests` (the DBoW2 equivalence test): `check` excludes it, `check_all` runs it, and it skips unless `$VOYAHSLAM_ORBVOC` is set (ruling E). | The 145 MB vocabulary is never committed; the test still compiles and runs everywhere. |
| 2026-10-07 | The vocabulary is loaded once by the app (`--vocabulary <file.bin>`) and shared as an immutable `std::shared_ptr<const bow::Vocabulary<FE>>` through `Slam` to the Mapper; null means no BoW and exactly the old behaviour. | Immutable data needs no lock and no replica; the app owns the file choice. |
| 2026-10-07 | The Mapper keeps each Frame in a private `KeyFrameRecord { frame; bow; features; }` (ruling G); BoW (`levels_up` 4) is computed for every Frame, initial ones included, and never published. | No Frame record existed and `map::Frame` is shared with the replicas; the bags of words are Mapper-only until the Frame database (step 3) is designed. |
| 2026-10-07 | `SearchForTriangulation` takes two optional FeatureVectors (`SearchByBoW`): candidates only from shared-node pairs, all other checks unchanged; its matches are drawn only from shared-node pairs and are not in general a subset of brute force (ruling F, spec §6 amended). A one-node vocabulary reproduces brute force exactly (ruling H, Mapper test). | ORB-SLAM3 `SearchForTriangulation`; fewer candidates can let the ratio test pass where brute force failed. |
| 2026-10-07 | ORB rotated BRIEF offsets are rounded with `std::lround` (ORB-SLAM3 `cvRound`) instead of truncated. | Bit-compatibility with ORB-SLAM3's vocabulary. |
| 2026-10-07 | BoW smoke result (corridor1, Release, two runs each with and without the vocabulary, plus one extra stereo run; "Bag of Words"): triangulation per Frame 26–28× faster mono (about 180 → 7 ms) and 39× stereo (about 148 → 4 ms), a Mapper pass about 290 → 140 ms. Mono: one map, 278 tracked, ATE Sim3 0.031–0.033 m (0.034 without). Stereo: one map and no loss in both planned runs, but ATE SE3 0.056 / 0.189 m (0.037–0.038 without); the extra run lost tracking at frame 325 (2 maps). The 3× Must is met; stereo accuracy regressed and nothing was tuned. | Cause is not fewer triangulated points (final review: a 150 ms sleep after triangulation restores ATE 0.037 m at the same point count); the regression follows Mapper speed, i.e. timing-dependent behaviour elsewhere (TODO). The frame-325 loss is the known fast-rotation weakness (runs without the vocabulary dip to 38 inliers there). |
| 2026-10-08 | Motion-model inlier gate: `min_motion_model_inliers` = 30; a motion-model pose with fewer final inliers counts as failed and zero motion (minimum 10) runs next. | Spike 2026-10-07: the stereo-with-vocabulary "drift" was 10–29-inlier motion-model poses in fast rotations becoming `need_close` Frames that the Tracker then locked onto. With the gate stereo 1000 + vocabulary 0.036–0.037 m, one map (was 0.056 / 0.189 m and losses); mono unchanged. |
| 2026-10-08 | Loss protocol (ORB-SLAM3 RECENTLY_LOST): with a vocabulary and a map of more than 10 Frames a failure enters `RecentlyLost` (`TrackingLost(final = false)`, nothing published, the Mapper keeps its map) and relocalizes every measurement for 5 s of measurement timestamps, then `Lost` (`final = true`, the only one that clears the map). Without a vocabulary, or on a small map, straight to `Lost` as before. | A short loss must not cost the map. Measurement time keeps the protocol deterministic and replayable. |
| 2026-10-08 | (H) A non-final `TrackingLost` does not supersede an earlier `FrameTracked` in the Mapper's drained batch; a final one does. | The map survives a non-final loss, so the last tracked frame is still worth evaluating. |
| 2026-10-08 | (C) `TrackingStage` = the source of the initial pose the local-map step refined (`MotionModel` / `ZeroMotion` / `ReferenceFrame` / `Relocalization`); `LocalMap` removed. | The local-map step always runs last, so `stage` was always `LocalMap` and could not show the recovery paths. |
| 2026-10-08 | Every added Frame carries its `BowVector` / `FeatureVector` in its `MapTransaction`; the Tracker's replica stores them and keeps its own `bow::FrameDatabase`; the Tracker gets the shared vocabulary through `Slam`. Reverses the 2026-10-07 "bags of words are Mapper-only" entry. | Relocalization runs on the Tracker thread against its replica; recomputing BoW there would duplicate the Mapper's work and the vocabulary is immutable. |
| 2026-10-08 | (D, A) `bow_vector.h` (plain data) moved from `voyah_algs` to `voyah_common` at the same include path, with a free `bow::Score`; `Vocabulary<FE>::Score` forwards to it and `FrameDatabase` is not templated. | Events (`voyah_events`) must not include algs; the database needs scoring without an extractor type. |
| 2026-10-08 | `bow::FrameDatabase`: inverted index owned by each replica, synced by transactions; `Query` keeps common ≥ 0.8 × max and returns L1 scores. Candidates grouped by the top 10 covisible Frames, kept at ≥ 0.75 × the best group (ORB-SLAM3 `DetectRelocalizationCandidates`). | Single-threaded per owner like every replica structure; no shared database, no lock. |
| 2026-10-08 | Reference-Frame tracking (ORB-SLAM3 `TrackReferenceKeyFrame`) after zero motion: `SearchByBoW` ratio 0.7, ≥ 15 matches, ≥ 10 inliers, then the local map; its values in `TrackingOptions` (ruling B amended). | The BoW fallback ORB-SLAM3 uses before declaring loss; tracking values belong with tracking. |
| 2026-10-08 | `geometry::SolvePnPRansac`: Lambda Twist P3P on unit bearings (`Camera::Unproject`), 4th point disambiguates, RANSAC 300 / 0.99 / ≥ 10, raw-pixel χ² 5.991 inliers; no OpenCV, no EPnP. MLPnP kept as an alternative (TODO). | Rays handle fisheye without undistortion; one residual space for the library. |
| 2026-10-08 | (E, F, G) `SolvePnPRansac` is a raw hypothesis generator; relocalization refines a RANSAC pose with `OptimizePose` over **all** the candidate's BoW matches (spec §4.5 step 2 amended). PnP tests assert over seeded scene sets (200 half-outlier scenes, ≥ 95 % on mask and on refined pose; 50 fisheye seeds at 1e-5), not one lucky seed. | Raw RANSAC poses meet 1 cm / 0.5° in only 34–59 % of half-outlier scenes; refining the inliers only 93 %, all matches 97.6 %. |
| 2026-10-08 | (I) Relocalization's projection passes use `features::SearchFromFrame`, ORB-exact `SearchByProjection(Frame, KeyFrame)`: radius `th · 1.2^level` px, no ratio test, levels [l−1, l+1], orientation check; `th` / distance 10 / 100 then 3 / 64. | The spec values are ORB-SLAM3's semantics; reusing `SearchFromLocalMap` multiplied the window by 2.5–4 and added a 0.8 ratio test. |
| 2026-10-08 | (J) ORB-SLAM3's post-relocalization guard: for 1 s of measurement timestamps from a relocalization, that measurement included, the Tracker's local-map gate is 50 inliers (`post_relocalization_seconds`, `post_relocalization_min_inliers` in `RelocalizationOptions`) and the Mapper creates no Frame from a `FrameTracked` within `relocalization_guard_seconds` (1.0, `KeyFrameOptions`) of the latest stage-`Relocalization` one, while its map has more than `relocalization_guard_min_frames` (20) Frames; a map clear (Mapper) or new map (Tracker) resets it. | Final review Important 1: every relocalized frame became a Frame (stereo: ~250 close points from the least-confirmed pose), so a wrong relocalization would be written into the map at once. The 20-Frame condition is ORB-SLAM3's `nKFs > mMaxFrames`; without it stereo corridor1 (18 Frames at recovery) lost tracking again 12 frames after recovery in 2 of 2 runs. |
| 2026-10-08 | Apps gain `--drop_frames a:b` (featureless measurements at the frames' own timestamps). | Reproducible blackouts for the relocalization smoke. |
| 2026-10-08 | Relocalization smoke result (corridor1, Release, `--vocabulary`, two runs each; "Relocalization and Reference-Frame Tracking"): mono 400 with frames 250–270 dropped recovers at frame 271 into map 1 in both runs (Sim3 0.0325 / 0.0336 m); stereo 1000 with 500–520 dropped recovers at 521 / 524 into map 1, no other loss (SE3 0.0351 / 0.0349 m); no drop: one map each, mono 0.0324 / 0.0323 m, stereo 0.0365 / 0.0363 m. A relocalization attempt costs about 10 ms. All Musts met; nothing tuned. Integration bounds: blackout centre error mono 1.5 × 0.0931 m, stereo 1.5 × 0.0217 m. | A short loss now keeps the map; accuracy after recovery is unchanged. |
| 2026-10-09 | Loop closing, roadmap step 4a+4b (spec `2026-10-09-loop-closing-design.md`; "Loop Closing"): the LoopCloser detects on its own replica (ORB-SLAM3 `NewDetectCommonRegions`) and publishes `LoopDetected`; the Mapper corrects (`CorrectLoop`: window, points, `SearchAndFuse`, essential graph, loop edge) as one `MapTransaction` at `map_epoch + 1`; the Tracker shifts `last_frame_` with its reference Frame. Global BA is step 4c, map merging step 5. | Keeps the single writer and lock-free replicas: a correction is one more transaction every thread applies. |
| 2026-10-09 | (A) `geometry::Sim3` is plain data with inline `Inverse` / `Compose` / `Apply` in `voyah_common` (`geometry/sim3.h`); Horn alignment and `SolveSim3Ransac` live in `voyah_algs` (`geometry/sim3_solver.h`). | `LoopDetected` (events) must carry a Sim3 without including algs. |
| 2026-10-09 | (B) Spanning tree: a new Frame sharing no points takes the previous newest Frame as parent (ORB-SLAM3 leaves none), and a culled Frame's children are re-parented by live shared-point counts with lower-id ties (ORB: cached weights ≥ 15). | Keeps the tree connected; live counts are what the covisibility index holds. Slightly different tree shapes (TODO). |
| 2026-10-09 | `SolveSim3Ransac` runs adaptive iterations (cap 300, confidence 0.99) and keeps the best hypothesis instead of ORB-SLAM3's first hypothesis over the inlier count; the pixel of each 3-D point is its projection in its own Frame (ORB `mvP1im1` / `mvP2im2`); its half-outlier test splits the noise 1/√2 px per source (residual σ 1 px, matching χ² 9.21) with a raw accuracy bar of 1° / 2 %. | Both are RANSAC with the spec's 300 / 0.99; best-hypothesis is stricter. The test's noise model, not the solver, was wrong. |
| 2026-10-09 | `SearchBySim3Projection` / `SearchPointsBySim3` compare the observing feature's descriptor and level, not a map point's distinctive descriptor; the matcher test lives in the existing `projection_matcher_test.cpp` with a trailing `ProjectionMatcherOptions` parameter; `OptimizeSim3`'s outlier test compares to the outlier-free optimum, not ground truth. | The interface has no per-point descriptor (TODO); noise alone moves the optimum 1–2.5 % in scale. |
| 2026-10-09 | `EssentialGraphProblem` splits `edges` (measured from the uncorrected poses) from `loop_connections` (measured from the corrected ones), deduplicated; loop connections keep every weight (ORB), covisibility edges need ≥ 100 shared points and exclude parent and child; the match Frame is the fixed vertex. | ORB-SLAM3 `OptimizeEssentialGraph` treats LoopConnections this way; the spec fixes the match Frame (ORB-SLAM2; ORB-SLAM3 fixes the map's first Frame), both valid. |
| 2026-10-09 | Loop thresholds follow the ORB-SLAM3 source where spec §4 first differed (RANSAC inliers 15, `OptimizeSim3` 20, projection 80, coarse projection 50 before `OptimizeSim3`, region dropped after 2 failed later queries; spec amended); `S_query_match` is ORB Scm (x_query = S · x_match), renamed from `S_query_to_match`. | The spec says thresholds follow ORB and the plan verifies each against the source; the old name contradicted the direction. |
| 2026-10-09 | Detection step 4 follows ORB: after RANSAC, the candidate window's points are projected into the query through the RANSAC Sim3 into a fresh match list (th 8, ratio 1.5, ≥ 50) before `OptimizeSim3`; a later query's refinement re-projects with the optimized Sim3; ORB's match-window widening for fewer than 10 covisibles is skipped. | ORB's source re-projects with the unoptimized guess (apparently a bug); the widening only matters on sparse maps (TODO). |
| 2026-10-09 | The LoopCloser does not suppress duplicate loops and is not wired to `TrackingLost` in `Slam`; the Mapper ignores a closed loop and another map's loop. Features without points are dropped from `matched_points`. | Avoids blocking Tracker events on the LoopCloser; costs some detection work and some fusable pairs (TODO). |
| 2026-10-09 | Mono correction: point reference distances are not rescaled (ORB `UpdateNormalAndDepth`); the middle Frames' rotation error under the identity-weighted essential graph (2.0° → 3.2° in the ring test) is accepted; the Tracker's relative last-to-reference translation and velocity stay in the old scale for one frame. | Mapper and replicas stay consistent; the cost is scale-gated misses until points are re-observed and one weaker prediction (TODO). |
| 2026-10-09 | `LoopCloserOptions::enabled` (default true at first; default false since 2026-10-10, below) and the apps' `--no_loop_closing` (replaced by `--loop_closing`): disabled, the LoopCloser behaves as without a vocabulary while the Tracker and Mapper keep theirs. | A no-loop baseline with otherwise identical behaviour (BoW triangulation, relocalization) for the smoke runs and the integration tests. |
| 2026-10-09 | `Mapper::num_frame_evaluations()` counts a `FrameTracked` once the Frame it created, if any, is published and refined; `stale_epoch_skipped()` counts `FrameTracked` of an older epoch. The integration tests' `RunSetup::lock_step` feeds each frame once the previous is tracked and the Mapper has handled every `FrameTracked` (evaluated, skipped or stale) and refined every Frame it created. | With the count before the creation a lock-step run raced the Mapper; lock-step takes the Tracker and Mapper out of thread timing (the LoopCloser is not gated, so when a correction arrives still can vary; three runs agree to four digits). |
| 2026-10-09 | `SlamTest.{Mono,Stereo}ClosesLoop`: a synthetic outward-looking loop (450° at 1.5° per frame) with place words; end error (last 20 frames, relative to the first-lap Frame of the same true pose in the final map, mono in its local scale; required to be measured in both runs) with loop closing ≤ 1/3 of without, no tracking loss across the correction; mono over seeds 1–3, where every seed whose end-of-run scale drift against truth exceeds 2 % without loop closing (some must) has it cut to at most a third with it. Measured mono 0.0676 / 0.0101, 0.1139 / 0.0157, 0.1429 / 0.0060 m and scale drift 5.5 / 0.4, 0.7 / 1.4, 4.6 / 0.5 %; stereo 0.0157 / 0.0023 m (without / with). | In an exact synthetic scene drift is a random walk; its scale part varies by scene (0.3–6 % over seeds 1–5), so the scale check needs a seed range, and it checks the correction, not only the detected Sim3 scale. |
| 2026-10-09 | (Task 8 ruling) Before any fix for the corridor1 false loops, diagnose whether they come from a fidelity bug or from an inherent weakness of ORB-SLAM3's gates. Result (`.superpowers/sdd/2026-10-09-loop-closing/false-loop-diagnosis.md`): inherent. Every deviation from ORB-SLAM3's source makes our chain stricter; an ORB-exact chain accepts the stereo 2262→1587 loop with 81 final matches (ours 82), and a full ORB-exact run accepts another false loop at exactly 80 (SE3 0.789 m against 0.077 m without loop closing). | A fix aimed at a fidelity bug would have missed; the gates must be extended instead. |
| 2026-10-09 | **User decision:** the LoopCloser adds a normal-consistency gate that ORB-SLAM3 does not have. A candidate (after step 5), a covisible confirmation, or a later query's refinement is rejected when more than `LoopCloserOptions::max_normal_disagreement` (0.5) of its final matched pairs (own map point, matched map point) have normals more than 60° apart. Pairs with an undefined normal are skipped. Not addressed: same-heading look-alikes (14.9 m translational repeats, normals consistent), and verifying a correction before publishing it (rejected option); both in TODO. | The diagnosis measured 0.96–1.00 of the pairs disagreeing on opposite-facing false loops, against 0.00–0.05 on true loops: a wide margin on both sides of 0.5. A deliberate deviation from ORB-SLAM3. |
| 2026-10-09 | Loop-closing smoke result (corridor1 full, Release, `--vocabulary`, two runs each with and without `--no_loop_closing`; "Loop Closing"). Before the normal gate (e529d07): every loop-closing run corrected an opposite-facing false loop that broke the map (mono ATE Sim3 0.652 m against 0.316 m without, and 2 maps in run 2; stereo 4 maps in both runs against 1 map at SE3 0.077 m). With the gate (fix round 1): every opposite-facing false loop was rejected (0.84–1.00) and every run keeps one map, but each run corrected a same-heading false loop 7–30 m apart, after which, in three of the four runs, the gate also rejected the true returns: mono 0.679 / 0.669 m against 0.341 / 0.500 m, stereo 1.295 / 18.16 m against 0.098 / 0.138 m. The Musts are **missed**; nothing was tuned. | The cause is established by the diagnosis (inherent in ORB-SLAM3's visual gates) and by the smoke: same-heading look-alikes are the remaining false loops, left for a user decision (TODO). |
| 2026-10-10 | **User decision:** loop closing is **opt-in and off by default**. `LoopCloserOptions::enabled` defaults to false, so a `Slam` with a vocabulary detects no loops unless it is enabled; relocalization and BoW triangulation keep using the vocabulary. The apps' `--no_loop_closing` is replaced by an opt-in `--loop_closing`. The branch merges as "implemented, opt-in". | Fix-round-1 smoke (corridor1, with the normal gate): every run keeps one map, but each corrects at least one same-heading look-alike (7–30 m apart), and the ATE with loop closing is worse than without (mono Sim3 0.679 / 0.669 m against 0.341 / 0.500 m; stereo SE3 1.295 / 18.16 m against 0.098 / 0.138 m). On synthetic loops it works (end error 0.04–0.15 of the error without). Next: look-alike rejection as the first item of roadmap step 6 (TUM-VI tuning). |
| 2026-10-10 | (Final-review fix wave, controller ruling on I1) The LoopCloser applies every `MapTransaction` on receipt (releasing it) and queues only the new Frames' ids; `ProcessPending()` detects one query per call; beyond `LoopCloserOptions::max_query_backlog` (8) the oldest queries are dropped, counted and logged. The Mapper never drops transactions and gets no back-pressure. A transaction raising the map epoch drops the pending region. A `LoopDetected` has priority unless a loop was already published at the replica's current epoch. | A lagging detector held the Mapper's pooled transactions (256) and `Create()` aborts on exhaustion; dropping queries only costs detections. One query per call makes `Stop()` prompt. A repeat found before its correction arrives would stop a local BA only for the Mapper to drop it. |
| 2026-10-10 | The g2o recipe's version is `20241228.1` (upstream tag + recipe revision), bumped whenever the recipe changes what it builds. | `types_sim3` was added without a version change, so a cache holding the old package resolved and then lacked the component. |
| 2026-10-10 | Global BA merge (roadmap step 4c, spec `2026-10-10-global-ba-design.md` section 3; `Mapper::MergeGlobalBa`): a `GlobalBAFinished` is merged only in `Mapping` state, of the current map and at the current epoch (else counted in `num_global_ba_rejected`). ORB-SLAM3's post-GBA block: a breadth-first walk of the spanning tree from its root, parents first; a Frame of the result takes its pose, another with a corrected parent `T_cur T_parent,cur^-1 T_parent,new`; an alive point of the result takes its position, another moves with its corrected reference Frame (`T_ref,new^-1 T_ref,cur X`). Unlike ORB-SLAM3's GBA block, the moved points' normals are recomputed, as in a loop correction. One `MapTransaction` at `map_epoch + 1`, never with a loop edge. | One more transaction every thread applies; the epoch makes the Tracker follow, and with no loop edge GlobalBA does not start another run on it. Normals follow the moved poses so the viewing-angle gates stay right. |
| 2026-10-10 | Global BA wiring (spec `2026-10-10-global-ba-design.md` section 4): `Slam` takes `GlobalBaOptions` after `LoopCloserOptions` and holds the `GlobalBundleAdjuster` as a `std::optional`, created only when `LoopCloserOptions::enabled && GlobalBaOptions::enabled` (since Task 5 also a vocabulary: row below; `global_ba()` is null otherwise). Wired Mapper → GlobalBA and GlobalBA → Mapper; started after the Mapper and before the LoopCloser and Tracker, stopped in reverse. The apps take `--no_global_ba`. | With loop closing off (the default) nothing is created, so behaviour is unchanged; the flag compares the essential graph alone with it followed by global BA. |
| 2026-10-10 | Global BA integration bars per mode (spec `2026-10-10-global-ba-design.md` section 5, controller ruling): stereo keeps points RMS ≤ 0.8× the essential-graph-only run and end error ≤ 1.0×. Mono, revised after measurement, is points RMS ≤ 0.95× per seed and ≤ 0.92× on average over seeds 1-3, and end error ≤ 1.05× per seed. Measured mono points ratios were 0.89 / 0.87 / 0.92 and end-error ratios 0.58 / 0.35 / 1.005. Stereo measured 0.41 / 0.48 / 0.44 and 0.76 / 0.67 / 0.18. | The 0.8× bar was an unmeasured guess. The mono RMS is dominated by low-parallax points of a sparse map, and global BA has converged (10 and 100 iterations give the same result), so for mono the bar is "no worse, and better on average". |
| 2026-10-10 | **User decision:** the GlobalBA (roadmap step 4c, "Global BA") owns a replica fed by the Mapper's transactions **from the start of the run**; there is no late joining. A component either exists from the start or not at all. | A component starting mid-run would need a full-map snapshot to build its replica (serialization of the whole map); deferred (TODO). |
| 2026-10-10 | **User decision:** components stay pluggable later through gRPC: GlobalBA talks to the Mapper only through events of plain data (`GlobalBAFinished`: ids, poses, positions; no pointers into map objects), like the LoopCloser. | An out-of-process transport needs only serialization, not a redesign (TODO). |
| 2026-10-10 | (Global BA pre-flight ruling) `GlobalBaOptions` keeps its own `sigma_px` / `level_scale` with the `MapperOptions::fuse` defaults. | GlobalBA does not depend on `MapperOptions`; the cost is a third σ source to keep in sync (TODO "Two σ sources"). |
| 2026-10-10 | (Global BA Task 1 ruling) Gauge: the oldest surviving Frame (index 0 of `keyframe_ids()`) is fixed even if it has no observations, in which case nothing is fixed in practice, as in ORB-SLAM3. | The Mapper culls point-less Frames, so the oldest Frame observes points; the cost is a free gauge in a pathological map. |
| 2026-10-10 | (Global BA Task 1 ruling) The per-observation BA helpers (`BaStereoOf`, `BaObservationOf`) moved to `mapping::detail` (`mapper_internal.h`), shared by local and global BA. | No duplicated observation building; the cost is that `global_ba.cpp` includes a Mapper-private header. |
| 2026-10-10 | (Global BA Task 2 rulings) First: `should_stop` popped waiting transactions into an owned backlog (moodycamel cannot peek), keeping their order. Superseded the same day by **apply-on-poll**: each poll drains the inbox, judges every transaction for interruption against the run's `(map_id, epoch)`, then applies it to the replica and so releases it. **Snapshot contract:** `RunGlobalBa` builds a self-contained problem before its first poll and never reads the replica again; the result carries the run's captured `(map_id, epoch)`. Spec section 1 amended. | Holding pooled transactions for a run (seconds on a large map, about 2 transactions per KeyFrame) exhausts the Mapper's pool of 256, which aborts (review: about 128 KeyFrames of Mapper output during a run). The cost: the replica is ahead of the result, which the merge handles through the epoch and the tree propagation. |
| 2026-10-10 | (Global BA Task 2) Test-only seams on `GlobalBundleAdjuster`: `stop_requested()` (an atomic read, any thread), `set_run_hook()` and `set_poll_hook()` (called on the GlobalBA thread before each run and at each `should_stop` poll; set while the worker is stopped). | `StopInterruptsRunningBa` must see that `Stop()` set the flag (a test flag set before `Stop()` races with a BA finishing its 10 iterations); the poll hook injects transactions mid-run deterministically. |
| 2026-10-10 | (Global BA Task 3 rulings) The merge recomputes the moved points' normals (ORB-SLAM3's GBA block does not), as our loop correction does; and a `GlobalBAFinished` is also rejected when the Mapper is not `Mapping` (a lost map with the same id and epoch), as `OnLoopDetected` does. | Consistency with step 4b; the normals follow the moved poses so the viewing-angle gates stay right. No cost beyond a little compute. |
| 2026-10-10 | (Global BA final review, M4) A global-BA merge raises the map epoch, so it also drops the LoopCloser's pending region (deviation from ORB-SLAM3, which does not reset it). Also: `Publish` is skipped if `Stop()` was requested after the run's last poll, and the destructor requests a stop before joining. | The pending region's `S_last_world` is in pre-merge coordinates; the cost is that a partly confirmed loop restarts after each merge. A stopped adjuster publishes nothing, as its header says. |
| 2026-10-10 | (Global BA Task 5, review minor) `Slam` creates the GlobalBA only with loop closing **in effect**: a vocabulary as well as `LoopCloserOptions::enabled` and `GlobalBaOptions::enabled`. The apps print "Global BA: on/off" from `slam.global_ba()`. Spec section 4 amended (there is no `SlamOptions`: `GlobalBaOptions` is a constructor parameter). | Without a vocabulary the LoopCloser never detects a loop, so a GlobalBA thread would only idle while the app printed "off"; now the printout and the object agree by construction. |
| 2026-10-10 | Global BA smoke result (corridor1 full, Release, `--vocabulary --loop_closing`, two runs each with and without `--no_global_ba`; "Global BA"): no crash, and no tracking loss caused by a merge (every loss began 0.1–1.0 s after an essential-graph loop correction or long after any merge; the Musts are met). Global BA runs started: mono 4 and 4 (0 and 3 interrupted), completed in 3.4–28.2 s; stereo 4 and 3 (none interrupted), 3.3–85.2 s; merges 3.8–31.7 ms; results 0.56–3.31 MB; nothing dropped or rejected. Peak RSS: stereo 4.23 GB with against 2.28 / 2.33 GB without; mono 2.24 / 2.38 against 1.51 / 1.47 GB. Both mono runs and stereo run 2 with global BA lost map 1 after false-loop corrections (2, 2 and 4 maps) while the runs without kept one map: not separable from run-to-run loop variance with two runs, not tuned. ATE not comparable (map 1 only where it was lost; stereo run 1 SE3 18.68 m from a false loop; without: mono Sim3 0.707 / 0.645 m, stereo SE3 0.836 / 0.768 m). | Spec section 5 has no ATE Must: corridor1's false loops still dominate (step 6). |
| 2026-10-10 | (Map merging Task 1, spec section 1) A Frame's measurement is **shared and immutable**: `TSharedMeasurement<FE> = std::shared_ptr<const TMeasurement<FE>>` in `FrameTracked`, `NewFrame`, the Mapper's `Frame` and every `ReplicaKeyFrame` (read through `measurement()`). The Tracker creates it once per tracked frame (`AcceptPose`, after stereo matching); the Mapper wraps a new map's initial Frames (from `InitializationMeasurement`, which keeps its by-value copy). Audit: nothing writes to a measurement after creation (the only write, `SetStereoMatches`, runs before tracking). Pooled `FrameTracked` / `MapTransaction` release it in `Reset()`. | Each event and each replica copied about 1 MB per Frame; the Atlas (every replica holding every map) would multiply that. `const` by type makes lock-free sharing safe; ownership by `shared_ptr` keeps the pointer rule (no raw pointers to short-lived data). |
| 2026-10-10 | (Map merging Task 2, spec section 1) **The replica rule replaces "newest map id wins".** `MapTransaction` gains `active_map` (set on every transaction), `removed_maps` and `absorbed_map` (Task 5; asserted unset). `tracking::AtlasReplica<FE>` holds a `MapReplica` per live map: `new_map` creates a map, other transactions apply to the live map they name, unknown or removed maps are ignored, then `removed_maps` are dropped and remembered (never re-admitted), then `active_map` is taken. `MapReplica` lost its map-id admission and holds one map. The Tracker, LoopCloser and GlobalBA hold an `AtlasReplica` and work on its active map (`replica()` returns it, or an empty map); the viewer's `SceneState` applies the same rule to its own per-map scenes (voyahviz depends only on common and events) and shows the active map. The Mapper's final-loss discard is published as one transaction `removed_maps = {map}`, no `active_map`; a final `TrackingLost` no longer clears the LoopCloser. `AtlasReplica::Apply` returns whether the transaction was admitted (the plan's interface has `void`): the holders act only on admitted transactions of the active map. | ORB-SLAM3's Atlas keeps lost maps and merges them (Tasks 3–5): replicas must hold several maps and learn the active one explicitly, which "newest id wins" cannot express. Behaviour is unchanged while the Mapper still discards a lost map; the visible differences are that replicas (and the viewer) drop the lost map at the loss rather than at the next map, and that a transaction without `active_map` deactivates (test harnesses now set it, as the Mapper does). |

| 2026-10-10 | (Map merging Task 3, spec sections 1, 5) **The Mapper's Atlas.** Per-map state moves into a private `MapState` held in `std::map<MapId, MapState> maps_` with `std::optional<MapId> active_`; existing code works on `Active()` (a cached pointer to the active map's node, kept with `active_`; `std::map` nodes are stable). On a final loss of the active map: fewer than `MapperOptions::atlas.min_frames_to_keep` (10) Frames → discarded, one transaction `removed_maps = {id}`, no `active_map`; otherwise kept inactive, one transaction with no `active_map`. The next initialization takes the next id. Inactive maps are frozen through the existing "another map" guards (now `IsActive(id)`). `num_keyframes()` / `num_map_points()` (atomics) are the active map's, 0 without one; `current_map_id()` stays the newest map's id. Public Mapper-thread accessors (`covisibility()`, `Reference()`, ...) read an empty map when none is active. | ORB-SLAM3 `Tracking`: `ResetActiveMap` below 10 KeyFrames, `CreateMapInAtlas` otherwise; keeping lost maps is what map merging (Tasks 4–5) merges into. A per-point map lookup would cost a `std::map::find` on hot paths, hence the cached pointer. |
| 2026-10-10 | (Map merging Task 4, spec section 2) **Merge detection in the LoopCloser.** `DetectNBestCandidates(atlas, query, options)` queries every live map's `FrameDatabase` (one per `MapReplica`, dropped with it on `removed_maps`) and splits the grouped hits into loop candidates (the active map) and merge candidates (inactive maps), best 3 each, the common-words threshold over all maps (ORB `KeyFrameDatabase::DetectNBestCandidates`). `DetectCommonRegion(atlas, query, candidates, options)` verifies either list with the same gates, the match side read from the candidate's map (`CommonRegion::match_map`); `RefineCommonRegion` takes the match map's replica. The LoopCloser keeps a second `PendingRegion` (`pending_merge_`) with its own counters; the loop/merge order follows ORB `NewDetectCommonRegions` and `Run`: a merge confirmed at the same query as a loop is published and the loop dropped. A pending region records its match map's epoch and is dropped when that map's epoch changes or the map is removed (this subsumes the old "active epoch drops the pending region" rule). The single-map `DetectCommonRegion(replica, query, options)` stays for tests. Deviation: the merge's not-found counter resets on a confirmation, as the loop's (ORB does not reset `mnMergeNumNotFound`). | Spec section 2; rulings: one database per live map, merges verified against the inactive map's replica, mono merges free-scale (the loop path already frees it for a mono query), the merge wins over a loop, no looser gates. The not-found reset keeps one rule ("2 failures in a row") for both kinds. |
| 2026-10-10 | (Map merging Task 4, fix round 1; controller ruling) **The normal-consistency gate is not applied to merges** (candidates, covisible confirmations, later-query refinements); loops keep it unchanged. The disagreement share is still measured and logged. Spec section 2 and Constraints updated. | The gate compares map-point normals in world coordinates; each map's world is its first Frame's camera, so across maps the comparison is meaningless, and the only relation between the two worlds is the merge hypothesis itself (aligning by it makes the gate vacuous). Applied literally it rejects every true merge between worlds turned by more than 60°. Cost: false-merge protection is weaker than for loops; merging stays opt-in (TODO: a hypothesis-independent check). |
| 2026-10-11 | (Map merging Task 5, spec section 3) **The merge absorbs first.** The Mapper moves M into N by the id rule before the correction steps (ORB-SLAM3 moves the window into the merge map during the correction and the rest after), so `CorrectWindow`, `FuseLoopPoints`, `FinishCorrection` and the tree edits run unchanged on one map; only the windows are built per map (`Connected` / `MergeWindowOf` take a `MapState`). The result is the same: the windows never share a Frame and are built before any fusion. The essential graph of step 6 holds only M's Frames, the welded window fixed (`EssentialGraphProblem::also_fixed`, new): in ORB-SLAM3's merge overload the merge window has no edge to M's other Frames (no common pose to measure from), so fixing it is the same as leaving it out. Deviations: the graph also runs for mono (ORB-SLAM3 only moves mono's rest rigidly; our window start keeps the merge scale, so the edges stay consistent, and the rest follows the weld); the `LoopDetected` pairs are fused as in a loop correction (spec section 3, step 4; ORB `MergeLocal` relies on `SearchAndFuse` alone); N's own recent points are dropped (the frozen map's list is stale). "A correction or merge is running" (spec section 3) has no separate state: the Mapper handles one event at a time, so the in-flight case is a merge or loop queued behind a merge, rejected because it names the removed map (`RejectsWhileCorrecting`). The viewer's `SceneState` (its own replica rule) absorbs too (`MapScene::Absorb`, same header), so the merged map shows both parts. | One code path for loops and merges; the id rule is the only thing replicas must reproduce, and it lives in one header (`map/absorbed_point_ids.h`) used by the Mapper, `AtlasReplica` and the viewer. |
| 2026-10-11 | (Map merging Task 5, fix round 1; review Critical 1) **Merges chain.** A merge target can later be absorbed itself (map 3 into 1, then 1 into 2), so the absorbed map's Frame ids may lie below and above the target's. The Mapper's `MapState` slots Frames by id from `first_slot` (the lowest id it ever held, slots sized over both maps at a merge) and keeps the spanning tree's `root` separately; `RootKeyFrame()` (was `FirstKeyFrame()`) is the never-culled local-BA gauge and Frame-culling exception. The global BA's gauge is the root too (the oldest Frame without a parent), no longer simply the oldest Frame. The replicas (`std::map` by Frame id) and the viewer were already order-independent (`AtlasReplicaTest.MergeAbsorbsInterleavedFrameIds`). | ORB-SLAM3 merges any number of maps; the slot vector's "the source's ids come after the target's" held only for a first merge (`MapperMergeTest.ChainedMergesThreeMaps` aborted on it). The root, not the oldest Frame, is the map's origin after a merge. |
| 2026-10-11 | (Map merging Task 4, fix round 2) **`DetectNBestCandidates` group scoring counts only filtered hits**: a hit's group score adds its top-10 covisible Frames' scores only for Frames that passed the int(0.8 · max) common-words filter. ORB-SLAM3 adds every word-sharing KeyFrame's `mPlaceRecognitionScore` (set only for filtered ones, so possibly stale from an earlier query). Also: `LoopDetected::matched_points` drops a self-pair (equal ids) only for a loop; a merge's pairs span two maps whose point ids restart per map, so equal ids are two points (`KeepsPairsWithCollidingIds`). | The filtered form is ORB-SLAM2's and what the old `RelocalizationCandidates` path did; it avoids the stale-score dependence. The id filter dropped every true merge pair whose ids collided, which Task 5 fuses from. |
| 2026-10-11 | (Map merging Task 6, spec section 4) **The Tracker follows a merge instead of adopting N.** A transaction whose `absorbed_map` is the Tracker's active map and whose `active_map` is the target keeps the Tracker's state, velocity and reference Frame, shifts `last_frame_` through the loop-correction anchors (reference Frame, else the Frames observing its matches; taken in M before the transaction, looked up in N after) and translates its matches with `AtlasReplica::TranslatePoint`. Any other change of the active map still resets (`AdoptMap`). | A merge moves the Tracker's own Frames, so the 4b epoch path applies unchanged; a reset would lose tracking at every merge (spec: no tracking loss across a merge). |
| 2026-10-11 | (Map merging Task 6, fix round 1; controller ruling) **A Lost Tracker re-announces its loss after a merge.** When a merge of its map lands while the Tracker is Lost, it publishes a final `TrackingLost(N)` (timestamp of its last final loss); the Mapper needs no change. Tests: `TrackerMergeTest.LostTrackerReannouncesLossAfterMerge`, `MapperMergeTest.ReannouncedLossAfterMergeDeactivatesTarget`; a merge while Tracking or RecentlyLost publishes no loss. | Without it the race (the merge applied before the stale `TrackingLost(M)`) leaves the Mapper Mapping in N and the Tracker Lost forwarding `InitializationMeasurement`s the Mapper ignores, with nothing to end it. The Tracker-side fix is always correct: it only sees such a merge when the Mapper read its loss after the merge. |
| 2026-10-10 | (Map merging, roadmap step 5; user decisions in the design conversation, spec `2026-10-10-map-merging-design.md`) **Scope A, ORB-SLAM3 parity**: keep lost maps of at least 10 Frames inactive, detect revisits of inactive maps in the LoopCloser and merge the active map into the inactive one (`MergeLocal`, visual path); relocalization stays active-map-only (into inactive maps: TODO). **Every replica mirrors the whole Atlas**, with each Frame's measurement created once and shared read-only. **The target map survives**: the inactive map keeps its id and world and absorbs the active map, whose points take new ids appended to the target's (the id rule) and which is retired; the active map is named explicitly (`active_map`), never "the newest id". Merging is part of loop closing (opt-in). | ORB-SLAM3's Atlas behaviour with the project's rules (one writer, lock-free replicas, ids never reused); sharing measurements keeps every replica holding every map affordable. |
| 2026-10-10 | (Map merging, controller rulings not recorded above) Task 5 produces `AtlasReplica::TranslatePoint(absorbed, old)` (the id rule's owner) and Task 6 consumes it. Task 1: a pooled event releases its measurement when its slot is reused (`Reset()` on `Create`), not on recycle: no pool change, the delay is bounded by the pool size; `Mapper::KeyFrame(FrameId) const` is a Mapper-thread accessor added for the shared-measurement test. Task 2: `AtlasReplica::active_or_empty()` serves holders that work on the active map only. Task 3: the review's cheap minors were taken at once (a bounded hook in the inactive-map GlobalBA test, both orders of the deactivation test). | Small, local choices; recorded so the decisions log holds every ruling of the plan. |
| 2026-10-11 | (Map merging Task 5; controller ruling) **"A correction or merge is running" is read as queue order.** The Mapper handles one event at a time, so a merge or loop queued behind a merge is rejected because it names the removed map; a merge queued behind a *loop correction* of M is accepted: its `S_query_match` is camera to camera, so the corrected query pose S·T_match does not depend on M's world. Rejecting it would need an epoch on `LoopDetected` (TODO). | No running state exists to test; the accepted case is geometrically sound. |
| 2026-10-11 | (Map merging Task 7) **Integration and harness.** `SlamTest.{Mono,Stereo}MergesMaps` run MonoClosesLoop's loop extended to 600° (401 frames) with a blackout over frames 140–165 and `recently_lost_seconds` 1 s (the default 5 s is 150° of this 20 s run: map 2 would not reach the LoopCloser's 12 Frames before leaving map 1's area); baseline: the same run without the blackout. The whole-run error is the RMS of every final-map Frame's centre against truth under one Umeyama alignment (similarity for mono, rigid for stereo): tracked poses before a merge are in another map's world, the final map's Frames are in one. The lock-step harness now waits for the Tracker to have processed each frame (`Tracker::num_processed()`) and for the Mapper to have handled every `InitializationMeasurement` (`Mapper::num_initialization_measurements()`): without it, a Lost Tracker forwarded the whole blackout tail and the Mapper's new mono map started far behind the camera (lost at once, maps 2..6). `Mapper::other_map_skipped()` counts `FrameTracked` of a map that is not active (in flight across a merge or a loss) so the gate accounts for them. Tracker: a merge of the active map that leaves no map active falls back to `AdoptMap(target)` (a reset) instead of keeping M's ids in the last frame (the Mapper never sends that shape). The apps print the Atlas (live maps, active, merges); the merge log names both Frames' timestamps and the weld time; `tools/ate.py --map ID` evaluates one map. | The spec's integration bars on a scene where both maps reach the Atlas thresholds at their default values; counters rather than sleeps keep the harness deterministic. |
| 2026-10-11 | (Map merging final review, I1) **Lost maps are kept only when `Slam` has a LoopCloser** (loop closing enabled and a vocabulary). Otherwise `Slam` passes the Mapper `atlas.min_frames_to_keep = SIZE_MAX`, so every lost map is discarded as on master; a kept map could never be merged and would only grow memory per loss. `SlamTest.LostMapsDiscardedWithoutLoopClosing`. | Binding ruling; merging needs the LoopCloser. |
| 2026-10-11 | **Brown-Conrady distortion model, naming.** The class is `BrownConradyDistortionModel` (coefficients in OpenCV order `[k1, k2, p1, p2, k3]`); one model under four names: Brown-Conrady = OpenCV standard = ROS `plumb_bob` = Kalibr `radtan` (= EuRoC `radial-tangential`). The reader accepts all three calibration strings with 4 or 5 coefficients. `Undistort` is Newton (1e-12, ≤ 20 iterations); a point it cannot invert, or whose root lies on the folded branch, is left unchanged, as with Kannala-Brandt. Tests may use OpenCV as a reference; the library may not. | The physical model's name, not one tool's; the calibration names are listed so each file format maps to it. Leaving failures unchanged keeps the `DistortionModel` interface; they occur only outside the calibrated field of view. |
| 2026-10-11 | **FAST ported from OpenCV, bit-exact.** `features::DetectFast` (`voyah_algs`, public `features/fast.h`) is OpenCV 5's `FAST_t<16>` + `cornerScore<16>` + AVX2 kernel (5.x at `20e367198c`, 5.1.0-dev = the installed OpenCV; unchanged since tag 5.0.0), keeping OpenCV's Apache-2.0 notice and Rosten's BSD notice. Same corners, order and responses as `cv::FAST(img, kps, t, true)`, TYPE_9_16; stride-aware views; `FastCorner{x, y, response}`. The ORB extractor calls it per cell exactly where it called `cv::FAST` (unused `cv::minMaxLoc` dropped); its output is byte-identical (golden digests recorded with `cv::FAST` before the switch). | User decisions: own FAST to drop OpenCV from the library, bit-exact so the switch changes nothing downstream (vocabulary, matching, tests). The per-level rework of OQ 21 is a separate, behaviour-changing step. |
| 2026-10-11 | **FAST SIMD paths: scalar, SSE2, AVX2 (run-time), NEON.** Each path is OpenCV's algorithm for its width and all are tested against live `cv::FAST`. x86-64: SSE2 baseline; AVX2 in its own file built with `-mavx2`, chosen by `__builtin_cpu_supports` (no `-march`, still runs without AVX2). arm64: NEON baseline. `FastSimd` lets tests and the benchmark force a path. Speed (user decision: at least as fast as `cv::FAST`, single thread): 0.90–0.97 of OpenCV's time per whole level (0.94 overall), 0.57 on the extractor's per-cell workload. NEON verified only under SIMDe emulation (TODO: arm64 hardware). | OpenCV here runs its AVX2 dispatch for FAST, so SSE2 alone (~1.1×) is not enough; run-time dispatch keeps one binary portable. NEON for the Android app. |
| 2026-10-11 | **SIFT extractor removed** (user decision): header, source, tests, the `INSTANTIATE_CLASS` entry and every explicit instantiation; `Slam<SIFT>` in `SlamTest.TrackEnqueuesMonocularAndBinocularMeasurements` became `OrbSlam`. Supersedes the SIFT parts of the 2026-10-04 level / `max_distance` rows and the 2026-10-07 `Vocabulary<FE>` row (ruling A now covers ORB only). | It was the last OpenCV user in the library, had no vocabulary, and doubled every template's instantiation. |
| 2026-10-11 | **The `voyah_*` libraries are OpenCV-free.** `voyah_algs` no longer links OpenCV (`opencv_bindings.h` and the include in `orb/extract_node.h` deleted); targets that compare with OpenCV (`voyah_algs_tests`, `voyah_algs_dataset_tests`, `fast_benchmark`, `voyah_common_tests`) link it themselves; `voyahviz` keeps it `PRIVATE`. New integration binary `voyah_algs_dataset_tests` for the corridor1 tests (skip without the dataset); `fast_benchmark` is built with `all` but is not in `check` / `check_all`. | Completes the 2026-10-03 goal "drop the OpenCV dependency" for the library; a package export no longer needs `find_dependency(OpenCV)`. |
| 2026-10-11 | **Conan package `voyahslam/0.1.0` (`conanfile.py`, replaces `conanfile.txt`), build options and install/export.** `VOYAHSLAM_BUILD_TESTS` / `VOYAHSLAM_BUILD_EXAMPLES` (OFF by default; the dev installs pass `-o with_tests=True -o with_examples=True`); the four libraries and the umbrella are installed and exported as `voyahslam::*`; Conan components with matching target names, the package target being the umbrella; PUBLIC deps (Eigen, spdlog/fmt, concurrentqueue) propagate headers, PRIVATE g2o and stb do not (g2o's libraries reach static consumers at link time only); `validate()` requires C++20, so the dev installs use `profiles/linux-gcc13` (default + `gnu20`), and rejects `shared=True` with `c++_static` and on Windows; `shared=True` makes spdlog/fmt shared. Android: `profiles/android-arm64`, API 24, clang 21 (NDK r30; `profiles/settings_user.yml` adds it to Conan 2.18's settings), **`compiler.libcxx=c++_static` (user decision)**: the app has a single JNI `.so` that links the static voyahslam libraries and is built with `ANDROID_STL=c++_static`. NDK rule: `c++_static` is safe only while the app has exactly one shared library containing C++; if more are ever added, everything must switch to `c++_shared`. | The library is consumed from an Android app (Kotlin + JNI, Gradle/CMake via the NDK) and from desktop. A package build must not need the dev tools (GTest, OpenCV, Pangolin, yaml-cpp). One C++ `.so` with a static STL is the smallest, simplest APK; a second C++ `.so` with its own static libc++ would split heaps, globals, exceptions and RTTI. |
| 2026-10-12 | **Distortion models are shared, immutable: `TDistortionModelPtr = std::shared_ptr<const DistortionModel>`** (user decision; `distortion_model.h`) in `geometry::Camera::distortion`, `StereoRig::distortion_left` / `distortion_right`, `MonocularMeasurement`'s constructor (hence `BinocularMeasurement`) and the example `EurocDatasetReader` (`CameraCalibration::distortion`; its `unique_ptr` store is gone). Null still means pinhole. Functions that only use a model during a call keep `const DistortionModel*` / `&` (`DropFeaturesBeyondMaxRayAngle`, `voyahviz::ProjectToImage` / `DrawMeasurement`). Hot paths do not copy a `Camera`: the g2o edges of `BundleAdjust`, `OptimizePose` and `OptimizeSim3` hold `const Camera&` into the call's const arguments (`BaProblem::keyframe_cameras` / `keyframe_stereo` is not modified, hence not reallocated, while the edges exist), and `detail::ProjectionGate` holds `const Camera&` (always a local of a search whose measurement argument owns the camera; its rvalue constructor is deleted). Measured (Release, scratch benchmark, TUM-VI KB model set; medians of 30 BA / 300 pose runs, 3 interleaved rounds): local BA (10 keyframes, 1500 points, 7500 observations, half stereo) 44.8 / 45.6 / 45.5 ms raw pointer, 44.6 / 43.9 / 44.9 ms shared_ptr with reference edges (shared_ptr with copying edges: 44.2 / 45.0 / 48.4); pose optimization (400 observations, half stereo) 2.76 / 2.77 ms vs 2.78 / 2.76 / 2.80 ms; 4 threads concurrently 0.96-1.16 vs 0.95-1.17 ms per call: all within noise. Remaining copies on the Tracker's path per frame: the app's measurement construction (mono 1; stereo 4: the rig copy and each side's camera), and in stereo one per `OptimizeMatchedPose` call (`StereoCameras{right().camera(), T_rl}`, 2 per normally tracked frame); mono 0. | Raw pointers that "must outlive every Camera copy" were unmanageable for a library consumer (e.g. an Android JNI app) whose objects die in their own order: cameras are copied into measurements, events, every thread's replica, BA problems and edges. Shared ownership ends that; the per-copy atomic cost is kept off inner loops by references. |

## 9. Open Questions

1. **ID width:** `uint32_t` (4·10⁹ ids, enough for any realistic run) vs `uint64_t`.
   With no reuse, memory — not id width — is the real limit (see 2).
2. **Storage growth without reuse:** `MapPointStorage` is indexed directly by id,
   so culled points leave dead slots forever. ORB-SLAM culls most new points
   early, so dead slots dominate on long runs (~90 B/point for ORB with double
   precision). Options: accept it; chunked storage that frees fully-dead chunks;
   or an id→slot indirection with compaction (must then be identical across replicas
   or be per-replica).
3. ~~Large diffs~~ — answered 2026-10-09 (loop closing): a correction is one
   `MapTransaction` with every corrected pose and point, applied atomically
   by every replica; see "Loop Closing" and the decisions log.
4. ~~Mapper back-pressure~~ — resolved 2026-10-06: the Tracker no longer needs
   to know (no ORB-SLAM3 `AcceptKeyFrames()`). The Mapper decides on Frames,
   drains `FrameTracked` to the latest per map and uses the skipped count as
   its backlog (backlog 0 = idle, `c1b`); see "Keyframes and Map Growth".
5. ~~Global BA after loop closure~~ — answered 2026-10-10 (step 4c): neither
   inside the Mapper nor in a Mapper-owned worker, but in its own component
   (`GlobalBundleAdjuster`) with its own replica, fed by the Mapper's
   transactions from the start. It publishes plain-data `GlobalBAFinished`,
   which the Mapper (the only writer) merges as one `MapTransaction` at a new
   epoch, propagating to Frames and points created meanwhile; local mapping
   never waits. See "Global BA".
6. ~~`LoopDetected` payload~~ — answered 2026-10-09: both map ids, both
   Frames, `S_query_match` and the matched point pairs; see §6 and "Loop
   Closing".
7. ~~Map-level versioning~~ — answered 2026-10-09: a per-map `map_epoch`,
   raised by each loop correction, carried by `MapTransaction` and
   `FrameTracked`; the Mapper drops a `FrameTracked` of an older epoch, the
   Tracker shifts on a newer one; see "Loop Closing".
8. ~~Covisibility graph / spanning tree / essential graph~~ — answered
   2026-10-09: covisibility from shared points (since step 2), a spanning tree
   (`parent`, `children`) and loop edges kept by every replica, the essential
   graph built from them for the correction; see "Loop Closing", "Map graph".
9. Keyframe culling and how `Frame` removal propagates (`KeyFrameRemoved` event?).
10. IMU support — out of scope for now. ~~Multi-map (Atlas)~~ — done 2026-10-11 (roadmap step 5, map merging): see "Mapper" (§4), the replica rule (§6) and "Loop Closing" (merge detection, the merge, the Tracker across a merge).
15. ~~Tracker threading~~ — decided 2026-10-02: own thread, two queues (§4).
16. ~~Measurement naming and timestamps~~ — decided 2026-10-02:
    `MonocularMeasurement` / `BinocularMeasurement`, timestamp in the measurement.
17. **Mixed mono/binocular keyframes:** `Frame<FE, TCameraFrameType>` fixes one
    type per map; Mapper/LoopCloser are templated only on `FE`. Needs a design
    that lets both keyframe types coexist (goal in §1).
18. ~~Component construction~~ — decided 2026-10-02: configuration (rig,
    extractor settings, pool sizes) is done by the user application; the
    reference application will live in `examples/full_slam/`.
19. **Tracker measurement policy:** track every measurement in order, or skip
    to the newest when falling behind (real-time)? Possibly configurable.
20. ~~Tracker idle wait~~ — resolved 2026-10-03: per-component `WakeSignal`
    (atomic counter, C++20 wait/notify) signalled by `AsyncObserver::Push()` and
    `Tracker::Track()`; `WorkerThread` sleeps on it (§4).
21. **ORB extraction speed.** Was ≈ 210 ms per 1024² image in Release with
    Conan's OpenCV 4.12; with OpenCV 5.1 (no per-call resize in `cv::FAST`) it
    is ≈ 70 ms (see decisions log). ORB-SLAM3 reference numbers are for 512²
    images with CLAHE applied first — not comparable to 1024². With OpenCV 5.1
    the 8-bit working image is ~8% *slower* than float (≈ 76 vs 70 ms mono);
    suspects: per-pixel `std::round` in `BilinearInterpolation`, 8-bit → float
    conversion per tap in `Filter` — re-profile before the extractor rework.
    Profile of the
    4.12 build, 2026-10-03 (perf, DWARF unwinding):
    - **57% `cv::FAST`:** the OpenCV 4.12 we link does `keypoints.resize(10000)`
      (280 KB, value-initialized) on every call before detecting; we call it per
      35-px cell (~1500–2500 calls/image, most cells twice because the dark
      TUM-VI images need the threshold-7 retry). Fresh memory → page faults.
      The allocator churn also makes timings depend on glibc's dynamic
      mmap/trim thresholds (stereo was 2× slower than 2× mono until fixed via
      `GLIBC_TUNABLES`).
    - 17% descriptors incl. our `images::Filter` blur (allocates two images per
      level per frame, per-tap border clamping); rest: allocations, pyramid
      resize, PNG decode.

    **Comparison with ORB-SLAM3** (`ORBextractor.cc`), same algorithm otherwise:
    - Cells: ORB-SLAM3 grows each cell by 6 px so FAST's 3-px border doesn't
      leave blind strips; ours don't overlap → undetectable 6-px strips every
      35 px.
    - ~~Descriptor sampling: ORB-SLAM3 rounds rotated pattern offsets (`cvRound`),
      we truncate (`cast<int>`) → descriptors differ bit-wise (matters for using
      ORB-SLAM3 DBoW2 vocabularies).~~ — done 2026-10-07: `std::lround` (see
      "Bag of Words").
    - ORB-SLAM3 keeps each keypoint's pyramid level (`octave`) and size;
      ~~`ImageFeatures` loses the level~~ — done: `ImageFeatures` carries
      `levels()` (used by the initialization matcher).
    - ORB-SLAM3 pads levels with a 19-px reflected border; we have none.
    - ORB-SLAM3 blurs into `uint8` with `cv::GaussianBlur`; our blurred image
      is float. ~~Unused `cv::minMaxLoc` per cell in ours.~~ — removed
      2026-10-11 with the FAST port.

    **Plan:** ~~own FAST-9-16~~ (done 2026-10-11: OpenCV's FAST ported
    bit-exact, still run per cell; see "FAST Corner Detector") run once per
    level, corners assigned to cells (≥ 20 if any, else ≥ 7) — fixes speed and
    blind strips; preallocated blur/pyramid buffers; ~~round descriptor offsets~~ (done);
    keep the level in `ImageFeatures`. Then re-profile. Parallel left/right
    extraction after that.
22. **Fisheye rays beyond 90°:** `MonocularMeasurement` stores undistorted
    *normalized* points (z = 1), which cannot represent rays at ≥ 90° off-axis;
    TUM-VI's image corners are there. **Interim (2026-10-03):** features whose
    ray is more than `MonocularMeasurement::kMaxRayAngle` = 85° off-axis are
    dropped at construction. **Long term:** store unit bearing vectors instead of
    z = 1 points (needed for full fisheye FOV and mixed cameras) — decide with
    the Tracker design. Since 2026-10-05 residuals and gates are in raw pixels
    (only the geometry stays on z = 1 rays), so keeping features beyond 85°
    needs bearing vectors only on the ray side (TODO).
23. ~~**Radial-tangential distortion** (EuRoC MAV) is not implemented~~ —
    resolved 2026-10-11: `BrownConradyDistortionModel` (see "Camera models
    (lens distortion)").
24. ~~Mono/stereo variants share an event tag~~ — resolved 2026-10-03:
    measurements travel as `features::TMeasurement<FE>`, a `std::variant`
    inside the events (§6). Mixed keyframes in one map remain OQ 17.
11. ~~Payload buffers are not reused~~ — resolved 2026-10-02 (objects stay alive
    in the pool, payloads in grow-only `ColumnBuffer`s). Related: chunking large
    loop-closure diffs (Q3) would also bound the buffers' peak capacity.
12. ~~Event pool takes a mutex~~ — resolved 2026-10-02 (lock-free Treiber stack).
13. ~~Ownership in `Notify()` depends on wiring~~ — resolved 2026-10-02
    (`Notify()` always takes ownership).
14. **Pool sizing.** Slot memory is now only committed on first use, but the
    index array (4 B/slot) is written up front, and the default `max_count` is
    still 1,000,000. Pools should be sized per event type from the worst-case
    backlog (consumer rule 3 in §6.1).
25. **Event pool lifetime.** Components own the pools of the events they
    publish, but consumers (e.g. the `MapViewer` inbox) can still hold those
    events. The apps declare the viewer before `slam`, so on exception unwinding
    `Slam` is destroyed first — while the viewer thread may still be running
    and draining its inbox (`DrainInbox()` recycles into the Tracker/Mapper
    pools) — and the viewer's destructor drain then recycles into freed pools
    too. The apps now guard against this: a guard declared right after `slam`
    runs `slam.Stop(); viewer.Stop()` on every exit path, so the viewer is
    stopped and fully drained before `~Slam` (as `libs/slam/tests/integration/slam_test.cpp`
    does). This is a caller-side workaround; the library-level design stays
    open: pools that outlive all producers and consumers.

## 10. Status and Next Steps

**Done:**
- Ported core modules from slamlab2 (78 tests).
- Core data layer + event system (merge `91b56aa`).
- Lock-free event pool, `Notify()` ownership, payload buffer reuse (merge `2e10c59`).
- One header per event; `Slam` / `Tracker` / `Mapper` / `LoopCloser` /
  `EventNode` skeleton with wiring; measurement rename + timestamps; Tracker
  measurement queue (branches `refactor/event-per-file`,
  `feat/slam-threads-skeleton` — merge pending).

- `examples/`: `voyahslam_dataset_reader` (EuRoC/TUM-VI), `monocular_full_slam`,
  `binocular_full_slam` (branch `feat/full-slam-example`). Fixed an infinite
  loop in `KannalaBrandtDistortionModel::PrecomputeInitialGuessLookup` for
  fisheye lenses wider than the image.

- Two-view geometry toolkit (branch `feat/two-view-geometry`, merge pending):
  `RigidTransformation`, homography/essential estimators with LM refinement,
  generic `Ransac`, decomposition, `SelectPose`, `TwoViewReconstructor`
  (see "Geometry Toolkit"). Now wired into `MapInitializer`.

- Monocular map initialization (branch `feat/mono-initialization`, merge
  pending): keypoint levels, `FeatureGrid`, `InitializationMatcher`, real mono
  `MapInitializer`, Mapper stores and publishes the initial map with
  observations, `TPose` -> `RigidTransformation` migration (see "Monocular
  Initialization").

- Stereo map initialization (branch `feat/stereo-initialization`, merge
  pending): `StereoRig::T_rl`, ORB-SLAM3 fisheye-style `MatchStereo`, stereo
  matching on the Tracker thread, stereo `MapInitializer`, Mapper stereo
  observation storage (see "Stereo Initialization").

- Pangolin 3D map viewer (branch `feat/pangolin-viewer`, merge pending):
  `SceneState` decoding, `MapViewer` window lifecycle, `PangolinRenderer`,
  apps `--viewer` (see "Map Viewer").

- Tracker/Mapper state machines and initialization protocol (branch
  `feat/tracking-states`, merge pending): `MapId` + `Event::map_id` consumer
  rule, `TMeasurement` variant in events, `InitializationMeasurement` /
  `MapCreated` / `TrackingLost` events, `WakeSignal` + `WorkerThread`
  component threads (OQ 20, 24 resolved), stub `PoseEstimator` /
  `MapInitializer`, end-to-end `Slam` protocol test; apps print component
  states.

- Tracking against the map (branch `feat/tracking`, merge pending):
  `MapReplica`, `OptimizePose`, projection matching, frame-to-map Tracker, the
  `PoseEstimator` stub removed (see "Tracking Against the Map").

- Camera model (branch `feat/camera-model`, merge pending): `DistortionModel`
  moved to `geometry/` with `DistortJacobian`, `geometry::Camera`, measurements
  keep raw keypoints and `camera()`, all five residual / gate consumers in raw
  pixels (see "Camera Model and Raw-Pixel Residuals").

- Keyframes and map growth, tracking sub-project 2 (branch `feat/keyframes`,
  merge pending): `FrameTracked`, the Mapper's Frame decision and creation
  (stereo close points, triangulation against neighbours) as one
  `KeyFrameCreated`, covisibility local map and reference Frame, viewer
  highlights (see "Keyframes and Map Growth").

- Restructure, roadmap step 0 (branch `feat/restructure`, merge pending):
  four libraries with separate include roots, per-library test binaries with
  `unit` / `integration` labels, `check` / `check_all`,
  `VOYAHSLAM_OPTIMIZE_TESTS` (see §7 "Library layout"; `check_all` 2.3 s).

- Map quality, roadmap step 1 / tracking sub-project 3 (branch
  `feat/map-quality`, merge pending): `MapTransaction` as the only Mapper
  event, urgent `FrameTracked` and the inbox priority counter, stereo and
  interruptible `BundleAdjust`, `FuseMatches`, recent-point and Frame culling,
  local BA, `--trajectory_out` and `tools/ate.py` (see "Map Quality"; mono
  corridor1 keeps one map for all 400 frames).

- Bag of words, roadmap step 2 (branch `feat/bow`, merge pending): an
  OpenCV-free DBoW2 port with text and binary loaders and
  `convert_vocabulary`, exact equivalence with DBoW2 on corridor1 frames, ORB
  offset rounding, BoW per Mapper Frame and `SearchByBoW` for triangulation,
  `--vocabulary` in the apps (see "Bag of Words"; triangulation 26–39× faster per
  Frame, stereo ATE regressed: TODO).

- Relocalization and reference-Frame tracking, roadmap step 3 / tracking
  sub-project 4 (branch `feat/relocalization`, merge pending): motion-model
  inlier gate (fixes the stereo-with-vocabulary regression), BoW in
  `MapTransaction`, `bow::FrameDatabase`, `features::SearchByBoW`,
  `geometry::SolvePnPRansac` (Lambda Twist P3P), `RecentlyLost` with
  `TrackingLost::final`, reference-Frame tracking, relocalization into the
  same map, `--drop_frames` (see "Relocalization and Reference-Frame
  Tracking").

- Loop detection and correction, roadmap step 4a+4b: implemented (opt-in,
  off by default), merged 2026-10-10; corridor1 Musts missed → step 6.
  Spanning tree, loop edges and map
  epoch; `geometry::Sim3`, `SolveSim3Ransac`, `OptimizeSim3`,
  `OptimizeEssentialGraph` (g2o `types_sim3`); LoopCloser replica and ORB-SLAM3
  common-region detection; Mapper correction as one transaction; Tracker
  shift; the normal-consistency gate (user decision); **opt-in**
  (`--loop_closing`, off by default; user decision 2026-10-10) (see "Loop
  Closing"). The synthetic loop tests pass; the corridor1 smoke Musts are
  missed (same-heading look-alikes, roadmap step 6).

- Global BA, roadmap step 4c (merged 9245f92):
  `RunGlobalBa` (ORB-SLAM3 `GlobalBundleAdjustemnt`, `bRobust = false`),
  `GlobalBAFinished`, the `GlobalBundleAdjuster` thread with its own replica
  (loop-edge trigger, apply-on-poll, interruption), the Mapper merge (tree
  propagation, epoch + 1), `Slam` wiring and `--no_global_ba` (see "Global
  BA"). Synthetic loops: points RMS 0.41–0.48× (stereo), 0.87–0.92× (mono)
  of the essential graph's; corridor1 smoke Musts met, with long runs and
  more memory (TODO).

- Map merging, roadmap step 5 (merged 9326160, 2026-10-11): the
  Atlas (lost maps of at least 10 Frames kept inactive), the replica rule
  (replacing "newest id wins"), shared measurements, merge detection in the
  LoopCloser, the Mapper's merge (`MergeLocal`) with the id rule, the Tracker
  following a merge (see "Mapper", §6 and "Loop Closing"). Synthetic merges
  meet the spec's bars (thin margins, TODO); corridor1 merge smoke: every
  merge true, no tracking loss caused by a merge; default stereo peak RSS
  1.23 GB (was 1.7 GB).

**Required follow-up:**
- Decide `MapPointStorage` growth / compaction (Open Question 2); ids are
  append-only since 2026-10-05.
- Decide on Open Question 14 (pool sizing per event type).

**Next:**
- ORB extractor plan in Open Question 21 (FAST once per level, buffers;
  own FAST and descriptor rounding done). The library no longer needs OpenCV
  (2026-10-11).
- Tracker measurement policy (19).
- Roadmap (`docs/ROADMAP.md`): TUM-VI tuning (step 6); relocalization into
  inactive maps is TODO ("Map merging follow-ups").
- Event pool lifetime (Open Question 25).
- Test suite (2026-10-11, FAST port): 764 tests (739 `unit`, 24
  `integration`, 1 `vocabulary`); the FAST unit tests add ~40 ms and
  `voyah_algs_dataset_tests` (corridor1) ~1 s.
- Test suite (2026-10-11, map merging Task 7): 735 tests (713 `unit`, 21
  `integration`, 1 `vocabulary`); `check_all` about 26–38 s, set by
  `SlamTest.{Mono,Stereo}MergesMaps` (about 20–26 s each, six lock-step runs
  of 401 frames), `SlamTest.MonoClosesLoop` (14–25 s: its lock-step runs are
  bursty, so the time follows the CPU's frequency scaling) and the two
  `*GlobalBaRefinesLoop` tests (about 17–20 s each), run in parallel.
  ThreadSanitizer (2026-10-11, scratch tree `-fsanitize=thread -O1 -g`,
  `setarch -R`): `voyah_slam_tests` 266 and `voyah_integration_tests` 21
  passed, no warnings.
