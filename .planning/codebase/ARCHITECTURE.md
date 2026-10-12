---
last_mapped_commit: b0e957ce59ec197bb96ec9ccbcc97cf037513c8e
last_mapped_at: 2026-10-12
---
<!-- refreshed: 2026-10-12 -->

# Architecture

**Analysis Date:** 2026-10-12

## System Overview

voyahslam is a lock-free, event-driven visual SLAM (Simultaneous Localization and Mapping) library inspired by ORB-SLAM3. The core principle: **no locks anywhere on the map data**. Threads communicate exclusively through publish/subscribe events over lock-free queues, and every component maintains its own replica of the map.

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                         Slam Orchestrator                               │
│                    (slam::Slam<TFeatureExtractor>)                      │
│  Wires four components async via Observable/AsyncObserver pattern       │
└─────────────────┬────────────────┬─────────────────┬────────────────────┘
                  │                │                 │
        ┌─────────▼────────┐ ┌─────▼────────┐ ┌──────▼──────────┐
        │     Tracker      │ │    Mapper    │ │  LoopCloser     │
        │  (tracking/)     │ │ (mapping/)   │ │ (loop_closing/) │
        │                  │ │              │ │                 │
        │ • Track frames   │ │ • Writes map │ │ • Detects loops │
        │ • Motion-only BA │ │ • Fuses pts  │ │ • Verifies w/BA │
        │ • Local mapping  │ │ • Local BA   │ │ • Never modifies│
        └────────┬─────────┘ └───────┬──────┘ └────────┬────────┘
                 │      FrameTracked  │ MapTransaction  │ LoopDetected
                 │                    │                 │
                 │      ┌─────────────┴────────────┬────┘
                 │      │                          │
                 └─────►│  AtlasReplica + Maps     │
                        │  (Tracker's local copy)  │
                        └──────────────────────────┘

        ┌────────────────────────────────────────────────────────┐
        │          GlobalBA (optional, with loop closing)        │
        │           (global_ba/global_bundle_adjuster.h)         │
        │  • Optimizes map after loop corrections                │
        │  • Publishes GlobalBAFinished                          │
        └────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│                         Event Infrastructure                           │
│           (libs/events: Observable, AsyncObserver, WakeSignal)        │
│  • Lock-free queues (moodycamel::ConcurrentQueue)                     │
│  • Pooled events with reference counting (Recyclable<T>)             │
│  • WorkerThread per component, woken by atomic wait/notify           │
└────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│          Memory & Data Structures (libs/common, libs/algs)             │
│  • MapPointStorage (SoA): positions, descriptors, observations        │
│  • Frame (Keyframe): id, pose, measurement, observations              │
│  • CovisibilityIndex: point→frames and frame→points mappings          │
│  • Maps identified by MapId; IDs never reused                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Component Responsibilities

| Component | Responsibility | File |
|-----------|----------------|------|
| **Tracker** | Estimates camera pose for each frame; reads own map replica; publishes `FrameTracked` events | `libs/slam/include/voyahslam/tracking/tracker.h` |
| **Mapper** | **Single map writer**: creates keyframes, map points, fuses, culls, runs local BA; applies every change as `MapTransaction` | `libs/slam/include/voyahslam/mapping/mapper.h` |
| **LoopCloser** | Place recognition on local replica; geometric verification (Sim3 RANSAC); publishes `LoopDetected` (never modifies map) | `libs/slam/include/voyahslam/loop_closing/loop_closer.h` |
| **GlobalBA** | Optimizes map after loop corrections; publishes `GlobalBAFinished` (Mapper applies the poses) | `libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h` |
| **Event System** | Pub/sub with lock-free queues, pooled events, worker threads per component | `libs/events/include/voyahslam/events/` |
| **Common Data** | IDs, geometry types, cameras, distortion models, memory pools, map storage structures | `libs/common/include/voyahslam/` |
| **Algorithms** | Feature extraction (ORB, FAST), matching, geometry (essential matrix, two-view), optimization (BA, pose) | `libs/algs/include/voyahslam/` |

## Pattern Overview

**Overall:** Event-driven actor model with replicated state.

**Key Characteristics:**
- **No locks on map data.** Cross-thread communication only via lock-free event queues.
- **One replica per thread.** Tracker, Mapper, LoopCloser each own a local copy (`AtlasReplica`, `MapReplica`).
- **Single writer (Mapper).** All map mutations happen in Mapper; other threads are read-only and publish events.
- **Atomic transactions.** `MapTransaction` encodes every Mapper change in order: new map, add Frames, add points, add observations, update points, update poses, remove observations, remove points, remove Frames.
- **Non-reusing IDs.** `FrameId` unique for the run; `MapPointId` unique per map, so `(map_id, point_id)` identifies a point globally.
- **Lock-free event pools.** Events pre-allocated; `Recyclable<T>` with ref counting; zero allocations per event in steady state (pool grows if needed).

## Layers

**Tier 1: Memory & Geometry (voyah_common)**
- Purpose: Basic types, memory management, geometry operations, logging.
- Location: `libs/common/`
- Contains: IDs, cameras, distortion models, transformations, feature measurements, image types, object pools, recyclable ref counting.
- Depends on: Eigen, spdlog, external headers only.
- Used by: Everything.

**Tier 2: Events & Pub/Sub (voyah_events)**
- Purpose: Event types, observable/observer pattern, worker threads, wake signals (no virtual dispatch).
- Location: `libs/events/`
- Contains: `Event`, `Observable`, `AsyncObserver`, `FrameTracked`, `MapTransaction`, `LoopDetected`, `GlobalBAFinished`, `TrackingLost`, `InitializationMeasurement`, `WorkerThread`, `WakeSignal`.
- Depends on: common, moodycamel::ConcurrentQueue.
- Used by: Slam components, visualizer.

**Tier 3: Algorithms (voyah_algs)**
- Purpose: Computer vision and geometry algorithms: feature extraction, matching, estimators, triangulation, bundle adjustment.
- Location: `libs/algs/`
- Contains: ORB/FAST extractors, brute-force/BoW matchers, essential/homography estimators, RANSAC, two-view reconstruction, Sim3 solver, pose/BA optimization, keyframe decisions.
- Depends on: common, g2o (private), stb (private).
- Used by: Mapper, Tracker, LoopCloser.

**Tier 4: SLAM Components (voyah_slam)**
- Purpose: Tracker, Mapper, LoopCloser, GlobalBA, and the `Slam` orchestrator.
- Location: `libs/slam/`
- Contains: Threading, state machines, replica management, map initialization, map quality decisions, loop closing, global BA.
- Depends on: common, events, algs.
- Used by: Applications.

**Visualization (voyahviz, separate)**
- Purpose: Development-only map viewer and feature window.
- Location: `vizualization/`
- Contains: `MapViewer`, scene state, Pangolin rendering.
- Depends on: common, events, OpenCV (private), Pangolin (private).
- Used by: `full_slam` example apps.

## Data Flow

### Primary Request Path (one tracked frame)

1. **User calls `Tracker::Track(measurement)`** (`libs/slam/src/tracking/tracker.h`)
   - Enqueues measurement into `Tracker::measurements_` queue, notifies the Tracker's wake signal.
   - Returns immediately (non-blocking).

2. **Tracker thread wakes, applies Mapper diffs to replica** (`libs/slam/src/tracking/tracker.cpp`)
   - Drains `AsyncObserver::inbox()` (map diffs from Mapper) and applies them to `AtlasReplica`.
   - Replica is now current.

3. **Tracker dequeues one measurement, tracks it** (`libs/slam/src/tracking/tracking.cpp`)
   - Executes motion model or pose-from-reference frame (optimization).
   - Projects local map points, matches features, runs motion-only BA (`PoseOptimization`).
   - If successful: publishes `FrameTracked`; if lost: publishes `TrackingLost`.

4. **`FrameTracked` event** (from `libs/events/frame_tracked.h`)
   - Payload: pose, measurement, inlier (feature, point) pairs, visible points, reference Frame, local Frames.
   - Multiple async subscribers: Mapper, LoopCloser, optional visualizer.
   - Marked `urgent` if inliers < `urgent_inlier_factor × min_local_map_inliers` (cuts Mapper's local BA short).

5. **Mapper thread wakes on `FrameTracked`, decides keyframe** (`libs/slam/src/mapping/mapper.cpp`)
   - Checks `ShouldCreateFrame` (motion, tracked points, time since last keyframe).
   - If keyframe: creates `Frame`, triangulates against covisible neighbors, fuses, culls, runs local BA.
   - Publishes one to two `MapTransaction` events (key frame + BA result).

6. **`MapTransaction` event** (from `libs/events/map_transaction.h`)
   - Payload: all mutations as separate parts: new map, added Frames, new points, added observations, updated points, updated poses, removed observations, removed points, removed Frames.
   - Atomic application: consumers apply in fixed order (new map → Frames → points → observations → updates → removals).
   - `map_id` specifies which map; `active_map` field announces the active map.
   - Every async observer (Tracker, LoopCloser, GlobalBA, visualizer) receives it.

7. **Tracker applies transaction to replica** (`libs/slam/src/tracking/atlas_replica.cpp`)
   - Updates `AtlasReplica::maps_[map_id]`.
   - Next measurement tracks against the updated map (lag accepted; ORB-SLAM3 behaves similarly).

8. **LoopCloser applies transaction, queues new Frames as queries** (`libs/slam/src/loop_closing/loop_closer.cpp`)
   - Applies to its `MapReplica`.
   - Adds new Frames to its `FrameDatabase` for place recognition.
   - Detects loop candidate: geometric verification with Sim3 RANSAC.
   - If loop found: publishes `LoopDetected`.

9. **Mapper receives `LoopDetected`** (if Mapper state is `Mapping` and map id matches active map)
   - Validates: ensures query and match Frames still exist.
   - Executes loop closure: fuses points, optimizes local window, essential graph, publishes corrected `MapTransaction`.
   - If `GlobalBA` enabled: loop edge causes `GlobalBAFinished` to trigger optimization.

### Initialization Flow

1. **Tracker in state `WaitingForMap` receives first measurement**
   - Publishes `InitializationMeasurement(map_id=0)` into `Mapper::inbox()`.

2. **Mapper in state `Initializing` feeds to `MapInitializer`** (`libs/slam/src/mapping/map_initializer.cpp`)
   - Monocular: matches two views, performs two-view reconstruction (triangulates, scales).
   - Stereo: single measurement with stereo matches.
   - Both: create initial Frames and map points.

3. **On success: Mapper creates map** 
   - Increments `next_map_id_`, takes `current_map_id_ = next_map_id_++`.
   - Publishes `MapTransaction` with `new_map` + initial Frames + initial points.
   - Moves to state `Mapping`.

4. **Tracker receives `MapTransaction` with `new_map`**
   - Creates its `AtlasReplica`, adds the map.
   - Moves from `WaitingForMap` to `Tracking`.

### Map Deactivation & Restart

1. **Tracker publishes `TrackingLost(map_id, final=true)`**
   - Indicates tracking has failed on map `map_id`.

2. **Mapper receives `TrackingLost` while in `Mapping`**
   - If map has < `atlas.min_frames_to_keep` Frames: **discards** it (publishes `MapTransaction` with `removed_maps`).
   - Otherwise: **keeps it inactive** (publishes `MapTransaction` with no `active_map`, no changes to the inactive map).
   - Moves to state `Initializing`, resets `MapInitializer`.

3. **All replicas drop the removed map or mark it inactive**
   - Next `InitializationMeasurement` triggers map 2 (ids never reuse).

### Map Merging (Merge Loop Closure)

1. **LoopCloser detects loop between two different maps**
   - Publishes `LoopDetected` with `query_map_id ≠ match_map_id`.

2. **Mapper receives merge `LoopDetected`**
   - Both maps must be live (query must be active; match must be live and inactive).
   - **Absorbs** query map M into match map N:
     - Translates M's point ids to N's range via id rule (`AbsorbedPointIds`).
     - Translates M's Frame ids (slotted over both ranges; maps may interleave after chained merges).
     - Moves observations, references, counters.
   - Corrects query window's poses via Sim3.
   - Inverts query's parent chain to root.
   - Fuses loop points; welds windows with BA (current free, merge fixed).
   - Essential graph BA for rest of map.
   - Publishes one `MapTransaction`: `absorbed_map`, `removed_maps`, `active_map` (match N), poses, point fusions, loop edge.

3. **All replicas apply merge atomically**
   - After the transaction, map M is gone; all its Frames and points are in N with new ids.
   - Tracker translates old point ids via `AtlasReplica::TranslatePoint()` (resolves Task 6).

### State Management

- **Tracker state** (`TrackerState`, readable via `state()`): `WaitingForMap` → `Tracking` ↔ `RecentlyLost` ↔ `Lost`.
- **Mapper state** (`MapperState`, readable via `state()`): `Initializing` ↔ `Mapping`.
- **Replica state** (each thread): mirrors Mapper's Atlas (maps by id, active map) via `MapTransaction` application.
- **Map state** (in Mapper): per-map `MapState` with Frames, points, covisibility, recent points, spanning tree, loop edges, correction epoch.

## Key Abstractions

**Frame (Keyframe)**
- Purpose: Immutable snapshot of a camera pose and the features that triggered its creation.
- Examples: `libs/common/include/voyahslam/map/frame.h`, `MonoFrame<FE>`, `StereoFrame<FE>`.
- Pattern: Strong id (`FrameId`), pose (`T_cw`), immutable measurement (shared via `std::shared_ptr<const TMeasurement<FE>>`), metadata (id, timestamp, map).

**MapPoint**
- Purpose: A 3D position reconstructed from multiple observations.
- Examples: Stored in `MapPointStorage<FE>` (structure-of-arrays), indexed by `MapPointId`.
- Pattern: Position, descriptor, normal, observation count, validity flag; updated in place by Mapper (replaced/culled), queried read-only by others.

**MapTransaction**
- Purpose: Atomic batch of mutations broadcast to all replicas.
- Examples: `libs/events/map_transaction.h`.
- Pattern: Divided into parts (new map, add/update/remove Frames, add/update/remove points, etc.) applied in fixed order; each part may be empty.

**Event (pooled, reference-counted)**
- Purpose: Inter-thread communication without locks.
- Examples: `FrameTracked`, `MapTransaction`, `LoopDetected`, `GlobalBAFinished`.
- Pattern: `Recyclable<T>` mixin, `ObjectPool<T, ThreadSafePolicy>`, `SetRefCount(n)` at publish for fan-out, auto-recycle on drop.

**CovisibilityIndex**
- Purpose: Observation graph: per point its observing Frames, per Frame its points.
- Examples: `libs/common/include/voyahslam/map/covisibility_index.h`.
- Pattern: Bidirectional (point→frames, frame→points); `Remove` keeps both sides in step; used by local mapping (find covisible neighbors) and BA (extract point-frame pairs).

**AtlasReplica**
- Purpose: Tracker's, LoopCloser's, GlobalBA's local copy of the map.
- Examples: `libs/slam/include/voyahslam/tracking/atlas_replica.h`.
- Pattern: Mirrors Mapper's `Atlas` (multiple maps, active map id); applies `MapTransaction`s verbatim; validates incoming ids before use.

## Entry Points

**`Slam::Slam<TFeatureExtractor>` constructor:**
- Location: `libs/slam/include/voyahslam/slam/slam.h`
- Triggers: Application initialization.
- Responsibilities: Creates Tracker, Mapper, LoopCloser, (optionally) GlobalBA; wires events; does NOT start threads.

**`Slam::Start()`:**
- Location: `libs/slam/src/slam/slam.cpp`
- Triggers: After constructor; starts consumer threads first (Mapper, GlobalBA, LoopCloser, Tracker).
- Responsibilities: Boots worker threads; begins processing events.

**`Tracker::Track(measurement)`:**
- Location: `libs/slam/include/voyahslam/tracking/tracker.h`
- Triggers: User application, once per frame.
- Responsibilities: Enqueues measurement, wakes Tracker thread.

**`Slam::Stop()` & `~Slam()`:**
- Location: `libs/slam/src/slam/slam.cpp`
- Triggers: Shutdown; stops producer threads first (Tracker, LoopCloser, GlobalBA, Mapper).
- Responsibilities: Gracefully drains queues, stops worker threads, destroys components.

**Observer attachment (e.g., visualizer):**
- Location: `slam.tracker()->AddObserver(observer)` (or mapper/loop_closer/global_ba).
- Triggers: Before `Start()`.
- Responsibilities: Attaches extra observers (e.g., viewer) to event streams.

## Architectural Constraints

- **Threading:** One worker thread per component (Tracker, Mapper, LoopCloser, GlobalBA). Threads pinned to cores via `WorkerThread` (if supported). Each component runs its `ProcessPending()` loop on its thread.
- **Global state:** No locks. Immutable shared data (vocabulary, camera calibration, algorithm parameters) may be shared raw or via `shared_ptr<const>`. Mutable state is partitioned: each thread owns its map replica.
- **Circular imports:** None enforced by library structure (common ← events ← algs ← slam). Events (Tier 2) depend on common; algs (Tier 3) depend on common (not events); slam (Tier 4) depends on events and algs.
- **Event pool sizing:** Configurable per event type; pools grow if needed (default: FrameTracked 256, MapTransaction 256, LoopDetected 16, GlobalBAFinished 2).
- **Frame measurement capture:** Feature extraction happens outside SLAM (in user app); measurements enqueued by value (take rvalue to avoid copy). `Tracker::Track()` is non-blocking.
- **Map replica lag:** Accepted design. A tracked frame may use a map one or two transactions behind; ORB-SLAM3's LocalMapping also exhibits lag.

## Anti-Patterns

### Mutating Map Directly (Outside Mapper)

**What happens:** Code in Tracker, LoopCloser or GlobalBA reaches into the map and changes a point or Frame.

**Why it's wrong:** 
- Breaks the single-writer contract: other replicas have stale data.
- Race with Mapper: concurrent mutation without locks leads to corruption.
- Replicas out of sync; viewer shows garbage.

**Do this instead:** Publish an event (e.g., `FrameTracked` with inlier info) and let Mapper decide mutations.

### Holding Event References Across ProcessPending()

**What happens:** Consumer saves `const FrameTracked&` from one `GetNextEvent()` and uses it in the next loop iteration.

**Why it's wrong:** Event is recycled between `GetNextEvent()` calls; the reference is now stale (object reused, payload overwritten).

**Do this instead:** Copy payload data you need before the next `GetNextEvent()` call (or before the `Listener` destructor, which triggers `Recycle()`).

### Calling `Recycle()` Explicitly

**What happens:** Consumer code calls `event->Recycle()` directly after processing.

**Why it's wrong:** 
- Breaks ref counting: other async observers haven't read the event yet.
- Pool corruption: event returned early, overwritten by next producer.

**Do this instead:** Let `Listener::GetNextEvent()` / `~Listener()` recycle automatically.

### Reusing FrameId or MapPointId Across Maps

**What happens:** Code assumes point id 42 in map 1 is the same object as point id 42 in map 2.

**Why it's wrong:** Map ids restart per map; `(map_id, point_id)` is the true identifier.

**Do this instead:** Always use `(map_id, point_id)` pair; validate ids are still alive before use (Mapper's `IsValid()` check on incoming events).

## Error Handling

**Strategy:** Asserts on programmer errors; exceptions propagated on runtime failures (e.g., allocation, I/O).

**Patterns:**
- **ID validation:** Before using an id from an old event, check `IsValid()` on the replica.
- **Event pool overflow:** Pool grows; logs one warning per growth (not per event).
- **Replica rules:** Unknown/removed map ids are silently ignored (stale events from lagging replicas).
- **Measurement validation:** Tracker validates keypoints are in image bounds; undefined behavior if violated.
- **Geometry failure:** Two-view initialization may never succeed (e.g., planar scene); returns `false`; Mapper stays `Initializing`.

## Cross-Cutting Concerns

**Logging:** `spdlog` (header-only) with named loggers per component (e.g., `"ObjectPool"`, `"Mapper"`, `"Tracker"`). Logger obtained via `voyahslam::logging::GetLogger()`. No log-level API; built at compile time (`-DSPDLOG_ACTIVE_LEVEL` in CMake).

**Validation:** Feature extraction happens outside SLAM; Tracker validates undistorted/normalized points and poses. Mapper validates ids, fuses/culls with validity checks. LoopCloser validates Frames before Sim3 computation.

**Authentication:** N/A (library, no user accounts).

**Configuration:** Options passed at component construction (`MapperOptions`, `TrackerOptions`, `LoopCloserOptions`, `GlobalBaOptions`). No runtime config files; C++ types fully specified at compile time.

---

*Architecture analysis: 2026-10-12*
